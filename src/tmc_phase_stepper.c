// Group-ISR phase stepper for TMC5160 XDIRECT mode.
//
// Continuous emission model: while phase stepping is active, the group
// ISR fires every tick and writes XDIRECT for every motor regardless of
// whether the host queue has fresh data.  When the queue drains, the
// position polynomial holds (vel=0, accel=0) and the same XDIRECT bytes
// are re-written each tick — equivalent to "idle" but with no special
// case in the ISR, no resume detection, no boundary discontinuity.
//
// The group ISR builds all motor messages into a per-bus tx_buf, then
// kicks one DMA per bus (per-motor CS sequencing handled in the DMA-TC
// callback).  Buffers live in the .dma_buf section, which on STM32H7
// resolves to D2 SRAM — DTCM at 0x20000000 is CPU-private.
//
// AWD pair atomicity: the host emits reset_phase_clock_group with a
// list of OIDs and a single waketime; the MCU resets all of them inside
// one irq_disable() window so no host-ordering race can desync the pair.
//
// Copyright (C) 2026  klipper contributors
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memset
#include "autoconf.h" // CONFIG_*
#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "command.h" // DECL_COMMAND

// Firmware-version marker so the host can confirm a fresh reflash carries
// the latest phase-stepping fixes.  Bump this string when MCU code changes
// in a way the host needs to detect.  Host reads via `MCU.get_constant`.
DECL_CONSTANT_STR("PHASE_STEPPER_VER", "v10-polled-spi-default");
#include "sched.h" // sched_add_timer
#include "spicmds.h" // spidev_kick_dma_tx
#include "trsync.h" // trsync_add_signal

// 256-entry quarter-wave sine table; values 0..248 (TMC5160 XDIRECT 9-bit
// signed range, with current_scale=256 acting as identity).
static const uint8_t sine_table[256] = {
      0,   2,   3,   5,   6,   8,   9,  11,  12,  14,  15,  17,  18,  20,
     21,  23,  24,  26,  27,  29,  30,  32,  34,  35,  37,  38,  40,  41,
     43,  44,  46,  47,  49,  50,  52,  53,  55,  56,  58,  59,  60,  62,
     63,  65,  66,  68,  69,  71,  72,  74,  75,  77,  78,  80,  81,  82,
     84,  85,  87,  88,  90,  91,  92,  94,  95,  97,  98,  99, 101, 102,
    104, 105, 106, 108, 109, 111, 112, 113, 115, 116, 117, 119, 120, 121,
    123, 124, 125, 127, 128, 129, 131, 132, 133, 134, 136, 137, 138, 140,
    141, 142, 143, 145, 146, 147, 148, 149, 151, 152, 153, 154, 155, 157,
    158, 159, 160, 161, 163, 164, 165, 166, 167, 168, 169, 170, 172, 173,
    174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187,
    188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201,
    202, 202, 203, 204, 205, 206, 207, 208, 208, 209, 210, 211, 212, 212,
    213, 214, 215, 216, 216, 217, 218, 218, 219, 220, 221, 221, 222, 223,
    223, 224, 225, 225, 226, 227, 227, 228, 228, 229, 230, 230, 231, 231,
    232, 232, 233, 233, 234, 234, 235, 235, 236, 236, 237, 237, 238, 238,
    239, 239, 239, 240, 240, 241, 241, 241, 242, 242, 242, 243, 243, 243,
    243, 244, 244, 244, 245, 245, 245, 245, 246, 246, 246, 246, 246, 246,
    247, 247, 247, 247, 247, 247, 247, 248, 248, 248, 248, 248, 248, 248,
    248, 248, 248, 248,
};

static int16_t
phase_sin(uint16_t phase)
{
    uint16_t idx = phase & 0x3FF;
    uint8_t quadrant = idx >> 8;
    uint8_t offset = idx & 0xFF;
    int16_t val = (quadrant & 1) ? sine_table[255 - offset] : sine_table[offset];
    return (quadrant >= 2) ? -val : val;
}

static int16_t
phase_cos(uint16_t phase)
{
    return phase_sin(phase + 256);
}

// Phase move segment: quadratic position model evaluated at fixed intervals.
struct phase_move {
    struct move_node node;
    int32_t start_position;
    int32_t velocity;
    int32_t acceleration;
    uint16_t count;
};

enum { PM_MODE_DIRECT_CURRENT = 0, PM_MODE_POSITION_TARGET = 1 };
enum {
    PSF_NEED_RESET = 1<<0,
    PSF_RESET_ARMED = 1<<1,
};

struct tmc_phase_stepper {
    struct spidev_s *spi;
    struct move_queue_head mq;
    int32_t position;            // 16.16 fixed-point microstep position
    int32_t velocity;            // per-tick delta
    int32_t acceleration;        // per-tick velocity delta
    uint16_t count;              // ticks remaining in current segment
    uint8_t mode;
    uint16_t current_scale;      // 0..256; 256 = identity (table peak 248 passes through)
    uint8_t flags;
    uint8_t bus_index;           // which group_isr.buses[] entry
    uint8_t bus_slot;            // index within bus's motors[]
    struct trsync_signal stop_signal;
    // Diagnostics
    uint32_t event_count;        // ISR ticks processed for this motor
    uint32_t write_count;        // accepted SPI write bursts for this motor
    uint32_t skip_count;         // write drops / stale pending chains
    uint16_t last_phase;
    int32_t last_emitted_position;
};

#define MAX_BUSES 4
#define MAX_MOTORS_PER_BUS 6
#define BYTES_PER_MOTOR 5

// One bus_group per SPI bus.  All motors on one bus serialize through a
// per-motor CS pulse sequence: the group ISR builds all messages into
// tx_buf, kicks the first motor's DMA; the DMA-TC callback raises that
// motor's CS and kicks the next.  In a future custom-board configuration
// with one bus per driver, motor_count==1 and the sequence collapses to
// one DMA + one CS toggle per tick.
struct phase_bus_group {
    struct tmc_phase_stepper *motors[MAX_MOTORS_PER_BUS];
    uint8_t motor_count;
    uint8_t cur_motor;           // index of motor whose DMA is in flight
    uint8_t pending_count;       // # motors whose messages are queued this tick
    uint8_t pending_motors[MAX_MOTORS_PER_BUS];  // ordered list of indices to send
};

// tx_buf must live in DMA-coherent SRAM.  On STM32H7, .dma_buf maps to
// D2 SRAM.  On other boards it falls into normal BSS (which is the same
// as main RAM and DMA-reachable on F4/F7/RP2040/etc).
static uint8_t bus_tx_buf[MAX_BUSES][MAX_MOTORS_PER_BUS * BYTES_PER_MOTOR]
    __attribute__((section(".dma_buf"))) __attribute__((aligned(32)));

static struct {
    struct timer timer;
    uint32_t interval;
    struct phase_bus_group buses[MAX_BUSES];
    uint8_t bus_count;
    uint8_t active;
    uint32_t group_tick_count;
} group_isr;

// Forward declarations
static void bus_dma_done(void *ctx);
static void kick_next_dma_in_bus(struct phase_bus_group *bg);

static uint8_t
group_has_armed_motors(void)
{
    for (uint8_t b = 0; b < group_isr.bus_count; b++) {
        struct phase_bus_group *bg = &group_isr.buses[b];
        for (uint8_t i = 0; i < bg->motor_count; i++) {
            struct tmc_phase_stepper *ps = bg->motors[i];
            if (ps && !(ps->flags & PSF_NEED_RESET))
                return 1;
        }
    }
    return 0;
}

static void
clear_pending_dma_chains(void)
{
    for (uint8_t b = 0; b < MAX_BUSES; b++) {
        group_isr.buses[b].pending_count = 0;
        group_isr.buses[b].cur_motor = 0;
    }
}

static void
phase_group_stop_if_unarmed(void)
{
    if (group_has_armed_motors())
        return;
    clear_pending_dma_chains();
    if (group_isr.active) {
        sched_del_timer(&group_isr.timer);
        group_isr.active = 0;
    }
    group_isr.interval = 0;
}

// Build one motor's 5-byte XDIRECT message into the bus's tx_buf at the
// given byte offset.  Returns the next byte offset.  Always emits — the
// continuous-emission model writes every tick regardless of whether the
// phase changed since last tick.
static inline uint8_t
build_motor_message(struct tmc_phase_stepper *ps, uint8_t *tx, uint16_t phase)
{
    int16_t cur_a = ((int16_t)phase_cos(phase) * (int16_t)ps->current_scale) >> 8;
    int16_t cur_b = ((int16_t)phase_sin(phase) * (int16_t)ps->current_scale) >> 8;
    uint16_t ua = (uint16_t)cur_a & 0x1FF;
    uint16_t ub = (uint16_t)cur_b & 0x1FF;
    tx[0] = 0x2D | 0x80;     // XDIRECT register (0x2D), write bit
    tx[1] = (uint8_t)(ub >> 8);
    tx[2] = (uint8_t)ub;
    tx[3] = (uint8_t)(ua >> 8);
    tx[4] = (uint8_t)ua;
    return BYTES_PER_MOTOR;
}

// Pop the next segment from a motor's queue.  If the queue is empty,
// hold position by zeroing velocity and acceleration (the polynomial
// becomes a constant — the same XDIRECT bytes will be written every
// tick until a new segment arrives).  We do NOT stop the timer here —
// the group ISR runs continuously while phase stepping is active.
static void
phase_stepper_load_next(struct tmc_phase_stepper *ps)
{
    if (move_queue_empty(&ps->mq)) {
        ps->velocity = 0;
        ps->acceleration = 0;
        ps->count = 0;
        return;
    }
    struct move_node *mn = move_queue_pop(&ps->mq);
    struct phase_move *pm = container_of(mn, struct phase_move, node);
    ps->position = pm->start_position;
    ps->velocity = pm->velocity;
    ps->acceleration = pm->acceleration;
    ps->count = pm->count;
    move_free(pm);
}

// Per-tick advance: emit current position, advance polynomial.
// Mirrors the host's mirror_advance closed form bit-for-bit.
static inline int32_t
phase_stepper_advance(struct tmc_phase_stepper *ps)
{
    int32_t emitted = ps->position;
    ps->position += ps->velocity;
    ps->velocity += ps->acceleration;
    if (ps->count) {
        if (--ps->count == 0)
            phase_stepper_load_next(ps);
    }
    return emitted;
}

// Group ISR: walk all buses, build all motor messages, kick first DMA per
// bus.  The DMA-TC callback chains subsequent motors on the same bus.
static uint_fast8_t
group_phase_stepper_event(struct timer *t)
{
    if (!group_has_armed_motors()) {
        clear_pending_dma_chains();
        group_isr.active = 0;
        group_isr.interval = 0;
        return SF_DONE;
    }
    group_isr.group_tick_count++;
    for (uint8_t b = 0; b < group_isr.bus_count; b++) {
        struct phase_bus_group *bg = &group_isr.buses[b];
        if (bg->motor_count == 0)
            continue;
        // If the previous tick's DMA chain hasn't finished, skip this bus
        // for this tick (motor will resume next tick).  Should not happen
        // at sane update rates with DMA — present as a safety net.
        if (bg->pending_count != 0) {
            // Drop this tick's writes for this bus.  Continue advancing the
            // polynomials so the host's mirror stays in sync.
            for (uint8_t i = 0; i < bg->motor_count; i++) {
                struct tmc_phase_stepper *ps = bg->motors[i];
                if (ps && !(ps->flags & PSF_NEED_RESET)) {
                    ps->event_count++;
                    ps->skip_count++;
                    phase_stepper_advance(ps);
                }
            }
            continue;
        }
        uint8_t bytes = 0;
        uint8_t pending = 0;
        for (uint8_t i = 0; i < bg->motor_count; i++) {
            struct tmc_phase_stepper *ps = bg->motors[i];
            if (!ps || (ps->flags & PSF_NEED_RESET))
                continue;
            ps->event_count++;
            int32_t emitted = phase_stepper_advance(ps);
            uint16_t phase = (uint16_t)((emitted >> 16) & 0x3FF);
            ps->last_phase = phase;
            ps->last_emitted_position = emitted;
            build_motor_message(ps, &bus_tx_buf[b][bytes], phase);
            bg->pending_motors[pending++] = i;
            bytes += BYTES_PER_MOTOR;
        }
        if (pending) {
            bg->pending_count = pending;
            bg->cur_motor = 0;
            kick_next_dma_in_bus(bg);
        }
    }
    group_isr.timer.waketime += group_isr.interval;
    return SF_RESCHEDULE;
}

// Kick the next DMA in this bus's pending sequence.  Per-motor CS: each
// motor has its own CS pin (different spidev_s), so we issue one DMA of
// 5 bytes per motor with CS pulses between.  spidev_kick_dma_tx pulls CS
// low; bus_dma_done raises it and kicks the next.
static void
kick_next_dma_in_bus(struct phase_bus_group *bg)
{
    if (bg->cur_motor >= bg->pending_count) {
        // All motors done.
        bg->pending_count = 0;
        bg->cur_motor = 0;
        return;
    }
    uint8_t motor_idx = bg->pending_motors[bg->cur_motor];
    struct tmc_phase_stepper *ps = bg->motors[motor_idx];
    uint8_t b = ps->bus_index;
    uint8_t *tx = &bus_tx_buf[b][bg->cur_motor * BYTES_PER_MOTOR];
    int rc = spidev_kick_dma_tx(ps->spi, tx, BYTES_PER_MOTOR,
                                bus_dma_done, bg);
    if (rc != 0) {
        // Bus contention — drop this and remaining motors for this tick.
        for (uint8_t i = bg->cur_motor; i < bg->pending_count; i++) {
            uint8_t skipped_idx = bg->pending_motors[i];
            struct tmc_phase_stepper *skipped = bg->motors[skipped_idx];
            if (skipped)
                skipped->skip_count++;
        }
        bg->pending_count = 0;
        bg->cur_motor = 0;
        return;
    }
    ps->write_count++;
}

// DMA-TC callback (runs in DMA-TC IRQ at NVIC priority 1).  CS for the
// current motor was raised by spidev_kick_dma_tx's wrap; advance and
// kick the next motor in this bus's sequence.
static void
bus_dma_done(void *ctx)
{
    struct phase_bus_group *bg = ctx;
    bg->cur_motor++;
    kick_next_dma_in_bus(bg);
}

// =====================================================================
// Public MCU commands
// =====================================================================

// config_tmc_phase_stepper oid=%c spi_oid=%c mode=%c
void
command_config_tmc_phase_stepper(uint32_t *args)
{
    struct tmc_phase_stepper *ps = oid_alloc(
        args[0], command_config_tmc_phase_stepper, sizeof(*ps));
    ps->spi = spidev_oid_lookup(args[1]);
    ps->mode = args[2];
    ps->current_scale = 256;  // identity (table peak 248 passes through)
    ps->flags = PSF_NEED_RESET;  // not yet armed by reset_phase_clock
    move_queue_setup(&ps->mq, sizeof(struct phase_move));

    // Default placement: bus_index=0, slot=motor_count.  Host can override
    // via config_phase_bus before phase stepping starts.
    if (group_isr.buses[0].motor_count >= MAX_MOTORS_PER_BUS)
        shutdown("Too many phase steppers on default bus");
    if (group_isr.bus_count == 0)
        group_isr.bus_count = 1;
    struct phase_bus_group *bg = &group_isr.buses[0];
    ps->bus_index = 0;
    ps->bus_slot = bg->motor_count;
    bg->motors[bg->motor_count++] = ps;

    if (group_isr.timer.func == NULL)
        group_isr.timer.func = group_phase_stepper_event;
}
DECL_COMMAND(command_config_tmc_phase_stepper,
             "config_tmc_phase_stepper oid=%c spi_oid=%c mode=%c");

// config_phase_bus oid=%c bus_index=%c
//
// Reassign a motor to a specific bus_group.  Sent by host at config time
// AFTER config_tmc_phase_stepper.  Default placement is buses[0]; this
// command moves the motor to a different bus (used for future per-driver
// SPI bus configurations).
void
command_config_phase_bus(uint32_t *args)
{
    uint8_t oid = args[0];
    struct tmc_phase_stepper *ps = oid_lookup(
        oid, command_config_tmc_phase_stepper);
    uint8_t target_bus = args[1];
    if (target_bus >= MAX_BUSES)
        shutdown("Invalid phase_bus index");

    irq_disable();
    // Remove from current bus.
    struct phase_bus_group *src = &group_isr.buses[ps->bus_index];
    for (uint8_t i = 0; i < src->motor_count; i++) {
        if (src->motors[i] == ps) {
            // Shift down to fill the gap.
            for (uint8_t j = i; j < src->motor_count - 1; j++)
                src->motors[j] = src->motors[j + 1];
            src->motor_count--;
            // Renumber slots in src.
            for (uint8_t j = 0; j < src->motor_count; j++)
                src->motors[j]->bus_slot = j;
            break;
        }
    }
    // Add to target bus.
    struct phase_bus_group *dst = &group_isr.buses[target_bus];
    if (dst->motor_count >= MAX_MOTORS_PER_BUS) {
        irq_enable();
        shutdown("Too many phase steppers on target bus");
    }
    if (target_bus >= group_isr.bus_count)
        group_isr.bus_count = target_bus + 1;
    ps->bus_index = target_bus;
    ps->bus_slot = dst->motor_count;
    dst->motors[dst->motor_count++] = ps;
    irq_enable();
}
DECL_COMMAND(command_config_phase_bus,
             "config_phase_bus oid=%c bus_index=%c");

// set_phase_stepper_current oid=%c scale=%hu
void
command_set_phase_stepper_current(uint32_t *args)
{
    uint8_t oid = args[0];
    struct tmc_phase_stepper *ps = oid_lookup(
        oid, command_config_tmc_phase_stepper);
    irq_disable();
    ps->current_scale = args[1];
    irq_enable();
}
DECL_COMMAND(command_set_phase_stepper_current,
             "set_phase_stepper_current oid=%c scale=%hu");

// queue_phase_move oid=%c interval=%u start_pos=%i velocity=%i accel=%i
//                  count=%hu
void
command_queue_phase_move(uint32_t *args)
{
    uint8_t oid = args[0];
    struct tmc_phase_stepper *ps = oid_lookup(
        oid, command_config_tmc_phase_stepper);
    struct phase_move *pm = move_alloc();
    pm->start_position = args[2];
    pm->velocity = args[3];
    pm->acceleration = args[4];
    pm->count = args[5];
    if (!pm->count)
        shutdown("Invalid phase_move count");

    uint32_t interval = args[1];
    irq_disable();
    if (ps->flags & PSF_NEED_RESET) {
        if (!(ps->flags & PSF_RESET_ARMED)) {
            // Motor not armed — host must call reset_phase_clock first.
            move_free(pm);
            irq_enable();
            return;
        }
        // First segment after reset: load it while the motor is still masked
        // from the ISR, then arm the motor atomically.  This prevents a reset
        // clock from re-enabling stale phase output before the first segment
        // reaches the MCU.
        ps->position = pm->start_position;
        ps->velocity = pm->velocity;
        ps->acceleration = pm->acceleration;
        ps->count = pm->count;
        ps->flags &= ~(PSF_NEED_RESET | PSF_RESET_ARMED);
        move_free(pm);
    } else if (ps->count) {
        // Currently running a segment — queue this one.
        move_queue_push(&pm->node, &ps->mq);
    } else {
        // Idle (count=0, vel=0, accel=0).  Load directly.
        ps->position = pm->start_position;
        ps->velocity = pm->velocity;
        ps->acceleration = pm->acceleration;
        ps->count = pm->count;
        move_free(pm);
    }
    if (group_isr.interval == 0)
        group_isr.interval = interval;
    if (!group_isr.active) {
        group_isr.active = 1;
        sched_add_timer(&group_isr.timer);
    }
    irq_enable();
}
DECL_COMMAND(command_queue_phase_move,
             "queue_phase_move oid=%c interval=%u start_pos=%i"
             " velocity=%i accel=%i count=%hu");

// Helper used by both reset_phase_clock and reset_phase_clock_group.
// Caller holds irq_disable().
static void
reset_one_motor_locked(struct tmc_phase_stepper *ps)
{
    ps->count = 0;
    ps->velocity = 0;
    ps->acceleration = 0;
    while (!move_queue_empty(&ps->mq)) {
        struct move_node *mn = move_queue_pop(&ps->mq);
        struct phase_move *pm = container_of(mn, struct phase_move, node);
        move_free(pm);
    }
    ps->flags |= PSF_NEED_RESET | PSF_RESET_ARMED;
}

// reset_phase_clock oid=%c clock=%u
void
command_reset_phase_clock(uint32_t *args)
{
    uint8_t oid = args[0];
    struct tmc_phase_stepper *ps = oid_lookup(
        oid, command_config_tmc_phase_stepper);
    uint32_t waketime = args[1];
    irq_disable();
    reset_one_motor_locked(ps);
    phase_group_stop_if_unarmed();
    if (!group_isr.active) {
        group_isr.timer.waketime = waketime;
        group_isr.group_tick_count = 0;
    }
    irq_enable();
}
DECL_COMMAND(command_reset_phase_clock,
             "reset_phase_clock oid=%c clock=%u");

// reset_phase_clock_group oids=%*s clock=%u
//
// Atomically reset waketime for an arbitrary list of motors.  Used to
// keep AWD pairs synchronized without depending on host flush order.
void
command_reset_phase_clock_group(uint32_t *args)
{
    uint8_t oid_count = args[0];
    uint8_t *oids = command_decode_ptr(args[1]);
    uint32_t waketime = args[2];
    if (oid_count == 0 || oid_count > MAX_BUSES * MAX_MOTORS_PER_BUS)
        shutdown("Invalid reset_phase_clock_group oid count");

    irq_disable();
    for (uint8_t i = 0; i < oid_count; i++) {
        struct tmc_phase_stepper *ps = oid_lookup(
            oids[i], command_config_tmc_phase_stepper);
        reset_one_motor_locked(ps);
    }
    phase_group_stop_if_unarmed();
    if (!group_isr.active) {
        group_isr.timer.waketime = waketime;
        group_isr.group_tick_count = 0;
    }
    irq_enable();
}
DECL_COMMAND(command_reset_phase_clock_group,
             "reset_phase_clock_group oids=%*s clock=%u");

// stop_phase_stepper oid=%c
void
command_stop_phase_stepper(uint32_t *args)
{
    uint8_t oid = args[0];
    struct tmc_phase_stepper *ps = oid_lookup(
        oid, command_config_tmc_phase_stepper);
    irq_disable();
    ps->count = 0;
    ps->velocity = 0;
    ps->acceleration = 0;
    ps->flags = PSF_NEED_RESET;
    while (!move_queue_empty(&ps->mq)) {
        struct move_node *mn = move_queue_pop(&ps->mq);
        struct phase_move *pm = container_of(mn, struct phase_move, node);
        move_free(pm);
    }
    phase_group_stop_if_unarmed();
    // Leave XDIRECT at its last held value — the host clears GCONF.direct_mode
    // separately, which restores normal step/dir behavior.
    irq_enable();
}
DECL_COMMAND(command_stop_phase_stepper, "stop_phase_stepper oid=%c");

// get_phase_stepper_status oid=%c
void
command_get_phase_stepper_status(uint32_t *args)
{
    uint8_t oid = args[0];
    struct tmc_phase_stepper *ps = oid_lookup(
        oid, command_config_tmc_phase_stepper);
    irq_disable();
    uint32_t event_count = ps->event_count;
    uint32_t write_count = ps->write_count;
    uint32_t skip_count = ps->skip_count;
    uint16_t last_phase = ps->last_phase;
    int32_t position = ps->position;
    irq_enable();
    sendf("phase_stepper_status oid=%c event_count=%u write_count=%u"
          " skip_count=%u last_phase=%hu position=%i",
          oid, event_count, write_count, skip_count, last_phase, position);
}
DECL_COMMAND(command_get_phase_stepper_status,
             "get_phase_stepper_status oid=%c");

// trsync stop callback for homing trigger.
static void
phase_stepper_stop(struct trsync_signal *tss, uint8_t reason)
{
    struct tmc_phase_stepper *ps = container_of(
        tss, struct tmc_phase_stepper, stop_signal);
    ps->count = 0;
    ps->velocity = 0;
    ps->acceleration = 0;
    ps->flags = PSF_NEED_RESET;
    while (!move_queue_empty(&ps->mq)) {
        struct move_node *mn = move_queue_pop(&ps->mq);
        struct phase_move *pm = container_of(mn, struct phase_move, node);
        move_free(pm);
    }
    phase_group_stop_if_unarmed();
    // Leave XDIRECT held; host restores GCONF.
}

void
command_tmc_phase_stepper_stop_on_trigger(uint32_t *args)
{
    uint8_t oid = args[0];
    struct tmc_phase_stepper *ps = oid_lookup(
        oid, command_config_tmc_phase_stepper);
    struct trsync *ts = trsync_oid_lookup(args[1]);
    trsync_add_signal(ts, &ps->stop_signal, phase_stepper_stop);
}
DECL_COMMAND(command_tmc_phase_stepper_stop_on_trigger,
             "tmc_phase_stepper_stop_on_trigger oid=%c trsync_oid=%c");

void
tmc_phase_stepper_shutdown(void)
{
    if (group_isr.active) {
        sched_del_timer(&group_isr.timer);
        group_isr.active = 0;
    }
    uint8_t i;
    struct tmc_phase_stepper *ps;
    foreach_oid(i, ps, command_config_tmc_phase_stepper) {
        move_queue_clear(&ps->mq);
        ps->count = 0;
        ps->velocity = 0;
        ps->acceleration = 0;
        ps->flags = PSF_NEED_RESET;
    }
    for (uint8_t b = 0; b < MAX_BUSES; b++) {
        group_isr.buses[b].pending_count = 0;
        group_isr.buses[b].cur_motor = 0;
    }
}
DECL_SHUTDOWN(tmc_phase_stepper_shutdown);
