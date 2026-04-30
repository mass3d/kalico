// Group-ISR phase stepper for TMC5160 XDIRECT mode
//
// A single timer processes all registered phase steppers on each tick,
// eliminating per-motor timer overhead and SPI bus contention. Each
// motor retains its own move queue, position polynomial, and counters.
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

enum {
    PSF_NEED_RESET = 1<<0,
    PSF_IDLE_HOLD = 1<<1,
    PSF_XDIRECT_VALID = 1<<2,
};

struct tmc_phase_stepper {
    struct spidev_s *spi;
    struct move_queue_head mq;
    uint32_t interval;          // Ticks between SPI writes (for queue compat)
    int32_t position;           // Current microstep position (16.16 fixed-point)
    int32_t last_emitted_position; // Last phase position actually written
    int32_t velocity;           // Current velocity per interval (16.16)
    int32_t acceleration;       // Current acceleration per interval
    uint16_t count;             // Steps remaining in current segment
    uint8_t mode;               // PM_MODE_DIRECT_CURRENT or PM_MODE_POSITION_TARGET
    uint16_t current_scale;     // Run current scaling factor (0..248)
    uint8_t slot;               // Bus scheduling slot for this motor
    uint8_t flags;
    uint8_t group_index;        // Index in group_isr.motors[]
    struct trsync_signal stop_signal;
    // Diagnostic counters
    uint32_t event_count;       // Total ISR events processed
    uint32_t write_count;       // Successful SPI writes
    uint32_t skip_count;        // Skipped writes (bus busy)
    uint16_t last_phase;        // Last phase value written
    uint16_t last_current_scale; // Current scale used for last write
    uint8_t pending;
    uint16_t pending_phase;
    uint16_t pending_current_scale;
    int32_t pending_position;
};

// Group ISR singleton -- single timer manages all phase steppers
#define MAX_GROUP_MOTORS 6

static struct {
    struct timer timer;
    uint32_t interval;              // Ticks between group ticks
    struct tmc_phase_stepper *motors[MAX_GROUP_MOTORS];
    uint8_t motor_count;            // Number of registered motors
    uint8_t slot_count;             // Number of bus slots in round-robin
    uint8_t force_all_once;         // Write all motors on the next tick
    uint8_t active;                 // Group timer is running
    uint32_t group_tick_count;      // Total group ticks
} group_isr;

static struct task_wake phase_stepper_wake;

// Load next phase_move segment from the queue
static uint_fast8_t
phase_stepper_load_next(struct tmc_phase_stepper *ps)
{
    if (move_queue_empty(&ps->mq)) {
        // Natural idle mirrors step/dir semantics: hold the last target phase
        // until a new move arrives or an explicit stop/reset tears the timer
        // down. Do not rewind to the last successful SPI write here: if a bus
        // conflict skipped the final move write, the next idle tick must catch
        // up to the intended held phase instead of staying behind.
        ps->count = 1;
        ps->velocity = 0;
        ps->acceleration = 0;
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

// Helper: check if all motors are fully stopped (PSF_NEED_RESET)
static uint8_t
all_motors_stopped(void)
{
    for (uint8_t i = 0; i < group_isr.motor_count; i++) {
        struct tmc_phase_stepper *m = group_isr.motors[i];
        if (m && !(m->flags & PSF_NEED_RESET))
            return 0;
    }
    return 1;
}

// Helper: stop the group timer if all motors are stopped
static void
maybe_stop_group_timer(void)
{
    if (group_isr.active && all_motors_stopped()) {
        sched_del_timer(&group_isr.timer);
        group_isr.active = 0;
    }
}

static uint8_t
all_motors_idle_or_stopped(void)
{
    for (uint8_t i = 0; i < group_isr.motor_count; i++) {
        struct tmc_phase_stepper *m = group_isr.motors[i];
        if (!m)
            continue;
        if (m->count && !(m->flags & PSF_IDLE_HOLD))
            return 0;
    }
    return 1;
}

static void
phase_stepper_advance(struct tmc_phase_stepper *ps, int32_t emitted_position)
{
    ps->position += ps->velocity;
    ps->velocity += ps->acceleration;

    if (--ps->count == 0) {
        ps->position = emitted_position;
        phase_stepper_load_next(ps);
    }
}

static uint8_t
phase_stepper_needs_xdirect_target(struct tmc_phase_stepper *ps, uint16_t phase)
{
    if (ps->mode != PM_MODE_DIRECT_CURRENT)
        return 0;
    if (ps->pending
            && ps->pending_phase == phase
            && ps->pending_current_scale == ps->current_scale)
        return 0;
    return !(ps->flags & PSF_XDIRECT_VALID)
        || ps->last_phase != phase
        || ps->last_current_scale != ps->current_scale;
}

static uint8_t
phase_stepper_slot_due(struct tmc_phase_stepper *ps, uint8_t active_slot)
{
    return group_isr.slot_count <= 1 || ps->slot == active_slot;
}

static void
phase_stepper_schedule_xdirect(struct tmc_phase_stepper *ps, uint16_t phase,
                               int32_t emitted_position)
{
    if (ps->pending
            && (ps->pending_phase != phase
                || ps->pending_current_scale != ps->current_scale))
        ps->skip_count++;
    ps->pending = 1;
    ps->pending_phase = phase;
    ps->pending_current_scale = ps->current_scale;
    ps->pending_position = emitted_position;
    sched_wake_task(&phase_stepper_wake);
}

static void
phase_stepper_write_xdirect(struct tmc_phase_stepper *ps, uint16_t phase,
                            uint16_t current_scale, int32_t emitted_position)
{
    int16_t cur_a = (phase_cos(phase) * (int16_t)current_scale) >> 8;
    int16_t cur_b = (phase_sin(phase) * (int16_t)current_scale) >> 8;
    uint16_t ua = (uint16_t)cur_a & 0x1FF;
    uint16_t ub = (uint16_t)cur_b & 0x1FF;
    uint8_t msg[5];
    msg[0] = 0x2D | 0x80;
    msg[1] = (uint8_t)(ub >> 8);
    msg[2] = (uint8_t)(ub);
    msg[3] = (uint8_t)(ua >> 8);
    msg[4] = (uint8_t)(ua);

    spidev_transfer_prepared(ps->spi, sizeof(msg), msg);

    ps->write_count++;
    ps->last_phase = phase;
    ps->last_current_scale = current_scale;
    ps->last_emitted_position = emitted_position;
    ps->flags |= PSF_XDIRECT_VALID;
}

void
tmc_phase_stepper_task(void)
{
    if (!sched_check_wake(&phase_stepper_wake))
        return;
    if (spidev_is_bus_busy()) {
        sched_wake_task(&phase_stepper_wake);
        return;
    }

    uint8_t spi_prepared = 0, pending_more = 0;
    spidev_set_bus_busy(1);
    uint8_t n = group_isr.motor_count;
    for (uint8_t i = 0; i < n; i++) {
        struct tmc_phase_stepper *ps = group_isr.motors[i];
        if (!ps)
            continue;

        irq_disable();
        uint8_t pending = ps->pending;
        uint16_t phase = ps->pending_phase;
        uint16_t current_scale = ps->pending_current_scale;
        int32_t emitted_position = ps->pending_position;
        ps->pending = 0;
        irq_enable();
        if (!pending)
            continue;

        if (!spi_prepared) {
            spidev_prepare_bus(ps->spi);
            spi_prepared = 1;
        }
        phase_stepper_write_xdirect(ps, phase, current_scale,
                                    emitted_position);
    }
    spidev_set_bus_busy(0);

    irq_disable();
    for (uint8_t i = 0; i < n; i++) {
        struct tmc_phase_stepper *ps = group_isr.motors[i];
        if (ps && ps->pending) {
            pending_more = 1;
            break;
        }
    }
    irq_enable();
    if (pending_more)
        sched_wake_task(&phase_stepper_wake);
}
DECL_TASK(tmc_phase_stepper_task);

// Group timer event: process all motors in a single ISR tick.
// Calls spidev_prepare_bus() once, then spidev_transfer_prepared()
// per motor — eliminates redundant spi_prepare() calls.
static uint_fast8_t
group_phase_stepper_event(struct timer *t)
{
    uint8_t slot_count = group_isr.slot_count ? group_isr.slot_count : 1;
    uint8_t active_slot = group_isr.group_tick_count % slot_count;
    uint8_t force_all = group_isr.force_all_once;
    group_isr.group_tick_count++;

    uint8_t any_active = 0;
    uint8_t n = group_isr.motor_count;
    for (uint8_t i = 0; i < n; i++) {
        struct tmc_phase_stepper *ps = group_isr.motors[i];
        if (!ps || ps->count == 0)
            continue;
        any_active = 1;
        ps->event_count++;

        // Compute phase from current position
        int32_t emitted_position = ps->position;
        uint16_t phase = (uint16_t)((ps->position >> 16) & 0x3FF);

        if ((force_all || phase_stepper_slot_due(ps, active_slot))
                && phase_stepper_needs_xdirect_target(ps, phase))
            phase_stepper_schedule_xdirect(ps, phase, emitted_position);

        phase_stepper_advance(ps, emitted_position);
    }

    group_isr.force_all_once = 0;

    // Stop group timer if all motors are fully stopped
    if (!any_active && all_motors_stopped()) {
        group_isr.active = 0;
        return SF_DONE;
    }

    group_isr.timer.waketime += group_isr.interval;
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
    ps->current_scale = 256; // 256 = no additional scaling (output = table value)
    ps->slot = 0;
    ps->last_current_scale = 0;
    ps->last_emitted_position = 0;
    move_queue_setup(&ps->mq, sizeof(struct phase_move));

    // Register in group ISR
    uint8_t idx = group_isr.motor_count;
    if (idx >= MAX_GROUP_MOTORS)
        shutdown("Too many phase steppers");
    group_isr.motors[idx] = ps;
    ps->group_index = idx;
    group_isr.motor_count = idx + 1;

    if (idx == 0)
        group_isr.timer.func = group_phase_stepper_event;
    if (!group_isr.slot_count)
        group_isr.slot_count = 1;
}
DECL_COMMAND(command_config_tmc_phase_stepper,
             "config_tmc_phase_stepper oid=%c spi_oid=%c mode=%c");

// MCU command: set_phase_stepper_slot oid=%c slot=%c slot_count=%c
void
command_set_phase_stepper_slot(uint32_t *args)
{
    uint8_t oid = args[0];
    struct tmc_phase_stepper *ps = oid_lookup(
        oid, command_config_tmc_phase_stepper);
    uint8_t slot = args[1], slot_count = args[2];
    if (!slot_count || slot >= slot_count)
        shutdown("Invalid phase stepper slot");
    irq_disable();
    ps->slot = slot;
    group_isr.slot_count = slot_count;
    irq_enable();
}
DECL_COMMAND(command_set_phase_stepper_slot,
             "set_phase_stepper_slot oid=%c slot=%c slot_count=%c");

// MCU command: set_phase_stepper_current oid=%c scale=%hu
void
command_set_phase_stepper_current(uint32_t *args)
{
    uint8_t oid = args[0];
    struct tmc_phase_stepper *ps = oid_lookup(
        oid, command_config_tmc_phase_stepper);
    irq_disable();
    ps->current_scale = args[1];
    ps->pending = 0;
    ps->flags &= ~PSF_XDIRECT_VALID;
    irq_enable();
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
        // Start group timer if not already running
        if (!group_isr.active) {
            group_isr.interval = interval;
            // Preserve waketime from reset_phase_clock if still valid
            uint32_t now = timer_read_time();
            if ((int32_t)(group_isr.timer.waketime - now)
                    < (int32_t)interval)
                group_isr.timer.waketime = now + interval;
            group_isr.group_tick_count = 0;
            group_isr.force_all_once = 1;
            sched_add_timer(&group_isr.timer);
            group_isr.active = 1;
        }
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
    ps->count = 0;
    ps->velocity = 0;
    ps->acceleration = 0;
    ps->flags = PSF_NEED_RESET;
    ps->pending = 0;
    // Flush move queue
    while (!move_queue_empty(&ps->mq)) {
        struct move_node *mn = move_queue_pop(&ps->mq);
        struct phase_move *pm = container_of(mn, struct phase_move, node);
        move_free(pm);
    }
    // Leave XDIRECT at its last held value — the host will restore
    // GCONF (clearing direct_mode) which returns the TMC to normal
    // step/dir with its own current regulation. Zeroing coil currents
    // here causes an audible click from the momentary torque loss.
    maybe_stop_group_timer();
    irq_enable();
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
    ps->count = 0;
    ps->velocity = 0;
    ps->acceleration = 0;
    ps->pending = 0;
    // Flush any queued moves
    while (!move_queue_empty(&ps->mq)) {
        struct move_node *mn = move_queue_pop(&ps->mq);
        struct phase_move *pm = container_of(mn, struct phase_move, node);
        move_free(pm);
    }
    // Re-anchor the group timer when it is only holding idle phases. The host
    // sends one reset per motor before queueing post-idle moves; the first
    // reset tears down the idle timer, and subsequent queues share this same
    // waketime so AWD/CoreXY partners start on the same tick.
    if (group_isr.active && all_motors_idle_or_stopped()) {
        sched_del_timer(&group_isr.timer);
        group_isr.active = 0;
    }
    if (!group_isr.active) {
        group_isr.timer.waketime = waketime;
        group_isr.group_tick_count = 0;
        group_isr.force_all_once = 1;
    }
    ps->flags = 0;  // Clear all flags (NEED_RESET, IDLE_HOLD)
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
    ps->count = 0;
    ps->velocity = 0;
    ps->acceleration = 0;
    ps->flags = PSF_NEED_RESET;
    ps->pending = 0;
    // Leave XDIRECT at last held value (see stop_phase_stepper comment)
    while (!move_queue_empty(&ps->mq)) {
        struct move_node *mn = move_queue_pop(&ps->mq);
        struct phase_move *pm = container_of(mn, struct phase_move, node);
        move_free(pm);
    }
    maybe_stop_group_timer();
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

// Shutdown handler
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
        ps->pending = 0;
    }
}
DECL_SHUTDOWN(tmc_phase_stepper_shutdown);
