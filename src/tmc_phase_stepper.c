// Timer-driven SPI phase stepper for TMC5160 XDIRECT mode
//
// Each phase-stepped motor keeps its own absolute due time, matching the
// step/dir model in stepper.c. Motors share the MCU clock, but they do not
// inherit another motor's cadence just because it is already active.
//
// Copyright (C) 2026  klipper contributors
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memset
#include "autoconf.h" // CONFIG_*
#include "basecmd.h" // oid_alloc
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "command.h" // DECL_COMMAND
#include "sched.h" // sched_add_timer
#include "spicmds.h" // spidev_transfer
#include "trsync.h" // trsync_add_signal

// 256-entry quarter-wave sine table (0..255 maps to 0..PI/2)
// Values are 0..248 (full-range for TMC5160 XDIRECT 9-bit signed current).
// With current_scale=256, output = table_val * 256 >> 8 = table_val (no loss).
// Full sine is reconstructed by symmetry.
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

// Lookup sine value for a full 1024-step electrical cycle.
// Input: 0..1023 representing 0..2*PI in microstep units.
// Output: signed value -248..+248 (TMC5160 XDIRECT current range).
static int16_t
phase_sin(uint16_t phase)
{
    // Map 1024-step cycle to quarter-wave lookup
    uint16_t idx = phase & 0x3FF; // mod 1024
    uint8_t quadrant = idx >> 8;  // 0..3
    uint8_t offset = idx & 0xFF;  // 0..255
    int16_t val;
    if (quadrant & 1)
        val = sine_table[255 - offset]; // mirror
    else
        val = sine_table[offset];
    if (quadrant >= 2)
        val = -val; // negate for 2nd half of cycle
    return val;
}

// Cosine = sine shifted by 256 (quarter cycle)
static int16_t
phase_cos(uint16_t phase)
{
    return phase_sin(phase + 256);
}

// Phase move segment: quadratic position model evaluated at fixed intervals
struct phase_move {
    struct move_node node;
    int32_t start_position;     // Fixed-point microstep position (16.16)
    int32_t velocity;           // Position delta per interval (16.16)
    int32_t acceleration;       // Velocity delta per interval
    uint16_t count;             // Number of intervals in this segment
};

// Driver output mode
enum { PM_MODE_DIRECT_CURRENT = 0, PM_MODE_POSITION_TARGET = 1 };

struct tmc_phase_stepper {
    struct timer timer;
    struct spidev_s *spi;
    struct move_queue_head mq;
    uint32_t interval;          // Ticks between SPI writes
    int32_t position;           // Current microstep position (16.16 fixed-point)
    int32_t last_emitted_position; // Last phase position actually written
    int32_t velocity;           // Current velocity per interval (16.16)
    int32_t acceleration;       // Current acceleration per interval
    uint16_t count;             // Steps remaining in current segment
    uint8_t mode;               // PM_MODE_DIRECT_CURRENT or PM_MODE_POSITION_TARGET
    uint16_t current_scale;     // Run current scaling factor (0..248)
    uint8_t flags;
    struct trsync_signal stop_signal;
    // Diagnostic counters
    uint32_t event_count;       // Total ISR events processed
    uint32_t write_count;       // Successful SPI writes
    uint32_t skip_count;        // Skipped writes (bus busy)
    uint16_t last_phase;        // Last phase value written
};

enum { PSF_NEED_RESET = 1<<0, PSF_IDLE_HOLD = 1<<1 };

// Load next phase_move segment from the queue
static uint_fast8_t
phase_stepper_load_next(struct tmc_phase_stepper *ps)
{
    if (move_queue_empty(&ps->mq)) {
        // Natural idle mirrors step/dir semantics: keep writing the held phase
        // until a new move arrives or an explicit stop/reset tears the timer
        // down. This avoids an implicit deactivate/reactivate boundary between
        // ordinary commands.
        ps->count = 1;
        ps->velocity = 0;
        ps->acceleration = 0;
        ps->position = ps->last_emitted_position;
        ps->flags &= ~PSF_NEED_RESET;
        ps->flags |= PSF_IDLE_HOLD;
        return SF_RESCHEDULE;
    }
    struct move_node *mn = move_queue_pop(&ps->mq);
    struct phase_move *pm = container_of(mn, struct phase_move, node);
    ps->position = pm->start_position;
    ps->velocity = pm->velocity;
    ps->acceleration = pm->acceleration;
    ps->count = pm->count;
    ps->flags &= ~PSF_IDLE_HOLD;
    move_free(pm);
    return SF_RESCHEDULE;
}

// Timer event: evaluate position and write SPI.
// The write-first order ensures the first tick outputs start_position
// (not start + velocity), preventing cumulative drift at move boundaries.
static uint_fast8_t
phase_stepper_event(struct timer *t)
{
    struct tmc_phase_stepper *ps = container_of(
        t, struct tmc_phase_stepper, timer);
    int32_t emitted_position = ps->position;

    ps->event_count++;

    // Write coil currents via SPI FIRST, using current position
    if (!spidev_is_bus_busy()) {
        uint16_t phase = (uint16_t)((ps->position >> 16) & 0x3FF);

        if (ps->mode == PM_MODE_DIRECT_CURRENT) {
            // TMC5160 MSCNT tracks phase B on the sine wave and phase A on the
            // cosine wave. XTARGET uses A in bits 8..0 and B in bits 24..16.
            int16_t cur_a = (phase_cos(phase)
                             * (int16_t)ps->current_scale) >> 8;
            int16_t cur_b = (phase_sin(phase)
                             * (int16_t)ps->current_scale) >> 8;
            uint16_t ua = (uint16_t)cur_a & 0x1FF;
            uint16_t ub = (uint16_t)cur_b & 0x1FF;
            uint8_t msg[5];
            msg[0] = 0x2D | 0x80;
            msg[1] = (uint8_t)(ub >> 8);
            msg[2] = (uint8_t)(ub);
            msg[3] = (uint8_t)(ua >> 8);
            msg[4] = (uint8_t)(ua);
            spidev_transfer(ps->spi, 0, sizeof(msg), msg);
            ps->write_count++;
            ps->last_phase = phase;
        }
    } else {
        ps->skip_count++;
    }
    ps->last_emitted_position = emitted_position;

    // Advance position polynomial AFTER SPI write
    ps->position += ps->velocity;
    ps->velocity += ps->acceleration;

    // Segment management
    ps->timer.waketime += ps->interval;
    if (--ps->count == 0) {
        ps->position = emitted_position;
        return phase_stepper_load_next(ps);
    }
    return SF_RESCHEDULE;
}

// MCU command: config_tmc_phase_stepper oid=%c spi_oid=%c mode=%c
void
command_config_tmc_phase_stepper(uint32_t *args)
{
    struct tmc_phase_stepper *ps = oid_alloc(
        args[0], command_config_tmc_phase_stepper, sizeof(*ps));
    ps->spi = spidev_oid_lookup(args[1]);
    ps->mode = args[2];
    ps->timer.func = phase_stepper_event;
    ps->current_scale = 256; // 256 = no additional scaling (output = table value)
    ps->last_emitted_position = 0;
    move_queue_setup(&ps->mq, sizeof(struct phase_move));
}
DECL_COMMAND(command_config_tmc_phase_stepper,
             "config_tmc_phase_stepper oid=%c spi_oid=%c mode=%c");

// MCU command: set_phase_stepper_current oid=%c scale=%hu
void
command_set_phase_stepper_current(uint32_t *args)
{
    uint8_t oid = args[0];
    struct tmc_phase_stepper *ps = oid_lookup(
        oid, command_config_tmc_phase_stepper);
    ps->current_scale = args[1];
}
DECL_COMMAND(command_set_phase_stepper_current,
             "set_phase_stepper_current oid=%c scale=%hu");

// MCU command: queue_phase_move oid=%c interval=%u count=%hu
//              start_pos=%i velocity=%i accel=%i
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
    if (ps->count) {
        // Currently active -- queue the move
        move_queue_push(&pm->node, &ps->mq);
    } else if (ps->flags & PSF_NEED_RESET) {
        // Motor's queue drained — host must send reset_phase_clock
        // to set waketime before new moves can be processed.
        move_free(pm);
    } else {
        // Not active and no reset needed — load immediately
        ps->interval = interval;
        ps->position = pm->start_position;
        ps->velocity = pm->velocity;
        ps->acceleration = pm->acceleration;
        ps->count = pm->count;
        move_free(pm);
        // Ensure waketime is safely in the future (it may be stale
        // if the queue drained and time has passed since last event)
        uint32_t now = timer_read_time();
        if ((int32_t)(ps->timer.waketime - now) < (int32_t)interval)
            ps->timer.waketime = now + interval;
        sched_add_timer(&ps->timer);
    }
    irq_enable();
}
DECL_COMMAND(command_queue_phase_move,
             "queue_phase_move oid=%c interval=%u start_pos=%i"
             " velocity=%i accel=%i count=%hu");

// MCU command: stop_phase_stepper oid=%c
void
command_stop_phase_stepper(uint32_t *args)
{
    uint8_t oid = args[0];
    struct tmc_phase_stepper *ps = oid_lookup(
        oid, command_config_tmc_phase_stepper);
    irq_disable();
    sched_del_timer(&ps->timer);
    ps->count = 0;
    ps->velocity = 0;
    ps->acceleration = 0;
    ps->flags = PSF_NEED_RESET;
    irq_enable();
    // Zero coil currents
    if (ps->mode == PM_MODE_DIRECT_CURRENT) {
        uint8_t msg[5] = { 0x2D | 0x80, 0, 0, 0, 0 };
        spidev_transfer(ps->spi, 0, sizeof(msg), msg);
    }
    // Flush move queue
    while (!move_queue_empty(&ps->mq)) {
        struct move_node *mn = move_queue_pop(&ps->mq);
        struct phase_move *pm = container_of(mn, struct phase_move, node);
        move_free(pm);
    }
}
DECL_COMMAND(command_stop_phase_stepper, "stop_phase_stepper oid=%c");

// MCU command: reset_phase_clock oid=%c clock=%u
void
command_reset_phase_clock(uint32_t *args)
{
    uint8_t oid = args[0];
    struct tmc_phase_stepper *ps = oid_lookup(
        oid, command_config_tmc_phase_stepper);
    uint32_t waketime = args[1];
    irq_disable();
    if (ps->count) {
        sched_del_timer(&ps->timer);
        ps->count = 0;
    }
    ps->velocity = 0;
    ps->acceleration = 0;
    ps->count = 0;
    // Flush any queued moves
    while (!move_queue_empty(&ps->mq)) {
        struct move_node *mn = move_queue_pop(&ps->mq);
        struct phase_move *pm = container_of(mn, struct phase_move, node);
        move_free(pm);
    }
    // Store waketime for queue_phase_move to use when starting the timer
    ps->timer.waketime = waketime;
    ps->flags = 0;  // Clear all flags (ACTIVE, NEED_RESET)
    irq_enable();
}
DECL_COMMAND(command_reset_phase_clock,
             "reset_phase_clock oid=%c clock=%u");

// MCU command: get_phase_stepper_status oid=%c
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

// Stop callback for homing trigger
static void
phase_stepper_stop(struct trsync_signal *tss, uint8_t reason)
{
    struct tmc_phase_stepper *ps = container_of(
        tss, struct tmc_phase_stepper, stop_signal);
    sched_del_timer(&ps->timer);
    ps->count = 0;
    ps->velocity = 0;
    ps->acceleration = 0;
    ps->flags = PSF_NEED_RESET;
    // Zero the coil currents on stop
    if (ps->mode == PM_MODE_DIRECT_CURRENT) {
        uint8_t msg[5] = { 0x2D | 0x80, 0, 0, 0, 0 };
        spidev_transfer(ps->spi, 0, sizeof(msg), msg);
    }
    while (!move_queue_empty(&ps->mq)) {
        struct move_node *mn = move_queue_pop(&ps->mq);
        struct phase_move *pm = container_of(mn, struct phase_move, node);
        move_free(pm);
    }
}

// MCU command: tmc_phase_stepper_stop_on_trigger oid=%c trsync_oid=%c
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

// Shutdown handler: zero XDIRECT currents
void
tmc_phase_stepper_shutdown(void)
{
    uint8_t i;
    struct tmc_phase_stepper *ps;
    foreach_oid(i, ps, command_config_tmc_phase_stepper) {
        move_queue_clear(&ps->mq);
        phase_stepper_stop(&ps->stop_signal, 0);
    }
}
DECL_SHUTDOWN(tmc_phase_stepper_shutdown);
