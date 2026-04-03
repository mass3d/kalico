// Trapezoidal velocity movement queue
//
// Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // unlikely
#include "trapq.h" // move_get_coord

// Allocate a new 'move' object
struct move *
move_alloc(void)
{
    struct move *m = malloc(sizeof(*m));
    memset(m, 0, sizeof(*m));
    return m;
}

// Return the distance moved given a time in a move
inline double
move_get_distance(struct move *m, double move_time)
{
    return (m->start_v + (m->half_accel + m->sixth_jerk * move_time)
            * move_time) * move_time;
}

// Return the XYZ coordinates given a time in a move
inline struct coord
move_get_coord(struct move *m, double move_time)
{
    double move_dist = move_get_distance(m, move_time);
    return (struct coord) {
        .x = m->start_pos.x + m->axes_r.x * move_dist,
        .y = m->start_pos.y + m->axes_r.y * move_dist,
        .z = m->start_pos.z + m->axes_r.z * move_dist };
}

#define NEVER_TIME 9999999999999999.9

// Allocate a new 'trapq' object
struct trapq * __visible
trapq_alloc(void)
{
    struct trapq *tq = malloc(sizeof(*tq));
    memset(tq, 0, sizeof(*tq));
    list_init(&tq->moves);
    list_init(&tq->history);
    struct move *head_sentinel = move_alloc(), *tail_sentinel = move_alloc();
    tail_sentinel->print_time = tail_sentinel->move_t = NEVER_TIME;
    list_add_head(&head_sentinel->node, &tq->moves);
    list_add_tail(&tail_sentinel->node, &tq->moves);
    return tq;
}

// Free memory associated with a 'trapq' object
void __visible
trapq_free(struct trapq *tq)
{
    while (!list_empty(&tq->moves)) {
        struct move *m = list_first_entry(&tq->moves, struct move, node);
        list_del(&m->node);
        free(m);
    }
    while (!list_empty(&tq->history)) {
        struct move *m = list_first_entry(&tq->history, struct move, node);
        list_del(&m->node);
        free(m);
    }
    free(tq);
}

// Update the list sentinels
void
trapq_check_sentinels(struct trapq *tq)
{
    struct move *tail_sentinel = list_last_entry(&tq->moves, struct move, node);
    if (tail_sentinel->print_time)
        // Already up to date
        return;
    struct move *m = list_prev_entry(tail_sentinel, node);
    struct move *head_sentinel = list_first_entry(&tq->moves, struct move,node);
    if (m == head_sentinel) {
        // No moves at all on this list
        tail_sentinel->print_time = NEVER_TIME;
        return;
    }
    tail_sentinel->print_time = m->print_time + m->move_t;
    tail_sentinel->start_pos = move_get_coord(m, m->move_t);
}

#define MAX_NULL_MOVE 1.0

// Add a move to the trapezoid velocity queue
void
trapq_add_move(struct trapq *tq, struct move *m)
{
    struct move *tail_sentinel = list_last_entry(&tq->moves, struct move, node);
    struct move *prev = list_prev_entry(tail_sentinel, node);
    if (prev->print_time + prev->move_t < m->print_time) {
        // Add a null move to fill time gap
        struct move *null_move = move_alloc();
        null_move->start_pos = m->start_pos;
        if (!prev->print_time && m->print_time > MAX_NULL_MOVE)
            // Limit the first null move to improve numerical stability
            null_move->print_time = m->print_time - MAX_NULL_MOVE;
        else
            null_move->print_time = prev->print_time + prev->move_t;
        null_move->move_t = m->print_time - null_move->print_time;
        list_add_before(&null_move->node, &tail_sentinel->node);
    }
    list_add_before(&m->node, &tail_sentinel->node);
    tail_sentinel->print_time = 0.;
}

// Fill and add a move to the trapezoid velocity queue
void __visible
trapq_append(struct trapq *tq, double print_time
             , double accel_t, double cruise_t, double decel_t
             , double start_pos_x, double start_pos_y, double start_pos_z
             , double axes_r_x, double axes_r_y, double axes_r_z
             , double start_v, double cruise_v, double accel)
{
    struct coord start_pos = { .x=start_pos_x, .y=start_pos_y, .z=start_pos_z };
    struct coord axes_r = { .x=axes_r_x, .y=axes_r_y, .z=axes_r_z };
    if (accel_t) {
        struct move *m = move_alloc();
        m->print_time = print_time;
        m->move_t = accel_t;
        m->start_v = start_v;
        m->half_accel = .5 * accel;
        m->start_pos = start_pos;
        m->axes_r = axes_r;
        trapq_add_move(tq, m);

        print_time += accel_t;
        start_pos = move_get_coord(m, accel_t);
    }
    if (cruise_t) {
        struct move *m = move_alloc();
        m->print_time = print_time;
        m->move_t = cruise_t;
        m->start_v = cruise_v;
        m->half_accel = 0.;
        m->start_pos = start_pos;
        m->axes_r = axes_r;
        trapq_add_move(tq, m);

        print_time += cruise_t;
        start_pos = move_get_coord(m, cruise_t);
    }
    if (decel_t) {
        struct move *m = move_alloc();
        m->print_time = print_time;
        m->move_t = decel_t;
        m->start_v = cruise_v;
        m->half_accel = -.5 * accel;
        m->start_pos = start_pos;
        m->axes_r = axes_r;
        trapq_add_move(tq, m);
    }
}

// Helper to add a single sub-move to the trapq and advance state
static void
scurve_add_phase(struct trapq *tq, double *print_time, struct coord *start_pos
                 , struct coord *axes_r, double duration
                 , double start_v, double half_accel, double sixth_jerk)
{
    if (duration <= 0.)
        return;
    struct move *m = move_alloc();
    m->print_time = *print_time;
    m->move_t = duration;
    m->start_v = start_v;
    m->half_accel = half_accel;
    m->sixth_jerk = sixth_jerk;
    m->start_pos = *start_pos;
    m->axes_r = *axes_r;
    trapq_add_move(tq, m);
    *print_time += duration;
    *start_pos = move_get_coord(m, duration);
}

// Fill and add up to 7 sub-moves for a jerk-limited S-curve profile.
// Phase durations are pre-computed by the Python motion planner:
//   tj1: jerk phase duration (accel side)
//   ta:  constant acceleration duration
//   tc:  cruise duration
//   tj2: jerk phase duration (decel side)
//   td:  constant deceleration duration
// The 7 phases are:
//   1. Accel ramp-up   (tj1): jerk = +j_max
//   2. Const accel      (ta): jerk = 0, accel = a_max
//   3. Accel ramp-down (tj1): jerk = -j_max
//   4. Cruise           (tc): jerk = 0, accel = 0
//   5. Decel ramp-up   (tj2): jerk = -j_max
//   6. Const decel      (td): jerk = 0, accel = -a_max
//   7. Decel ramp-down (tj2): jerk = +j_max
void __visible
trapq_append_scurve(struct trapq *tq, double print_time
                    , double start_pos_x, double start_pos_y
                    , double start_pos_z
                    , double axes_r_x, double axes_r_y, double axes_r_z
                    , double start_v, double jerk_max
                    , double accel_max, double cruise_v
                    , double tj1, double ta, double tc
                    , double tj2, double td)
{
    struct coord start_pos = {.x=start_pos_x, .y=start_pos_y, .z=start_pos_z};
    struct coord axes_r = { .x=axes_r_x, .y=axes_r_y, .z=axes_r_z };
    double j6 = jerk_max / 6.;
    // Actual peak acceleration for each side (equals accel_max when ta>0/td>0,
    // but less in the triangular case where ta=0/td=0)
    double a_peak_accel = jerk_max * tj1;
    double a_peak_decel = jerk_max * tj2;
    double ha_accel = .5 * a_peak_accel;
    double ha_decel = .5 * a_peak_decel;
    double v = start_v;
    // Phase 1: accel ramp-up (jerk = +j_max)
    scurve_add_phase(tq, &print_time, &start_pos, &axes_r,
                     tj1, v, 0., j6);
    v += .5 * jerk_max * tj1 * tj1;
    // Phase 2: constant accel (accel = a_peak_accel)
    scurve_add_phase(tq, &print_time, &start_pos, &axes_r,
                     ta, v, ha_accel, 0.);
    v += a_peak_accel * ta;
    // Phase 3: accel ramp-down (jerk = -j_max)
    scurve_add_phase(tq, &print_time, &start_pos, &axes_r,
                     tj1, v, ha_accel, -j6);
    v += a_peak_accel * tj1 - .5 * jerk_max * tj1 * tj1;
    // Phase 4: cruise
    scurve_add_phase(tq, &print_time, &start_pos, &axes_r,
                     tc, cruise_v, 0., 0.);
    // Phase 5: decel ramp-up (jerk = -j_max)
    v = cruise_v;
    scurve_add_phase(tq, &print_time, &start_pos, &axes_r,
                     tj2, v, 0., -j6);
    v -= .5 * jerk_max * tj2 * tj2;
    // Phase 6: constant decel (accel = -a_peak_decel)
    scurve_add_phase(tq, &print_time, &start_pos, &axes_r,
                     td, v, -ha_decel, 0.);
    v -= a_peak_decel * td;
    // Phase 7: decel ramp-down (jerk = +j_max)
    scurve_add_phase(tq, &print_time, &start_pos, &axes_r,
                     tj2, v, -ha_decel, j6);
}

// Expire any moves older than `print_time` from the trapezoid velocity queue
void __visible
trapq_finalize_moves(struct trapq *tq, double print_time
                     , double clear_history_time)
{
    struct move *head_sentinel = list_first_entry(&tq->moves, struct move,node);
    struct move *tail_sentinel = list_last_entry(&tq->moves, struct move, node);
    // Move expired moves from main "moves" list to "history" list
    for (;;) {
        struct move *m = list_next_entry(head_sentinel, node);
        if (m == tail_sentinel) {
            tail_sentinel->print_time = NEVER_TIME;
            break;
        }
        if (m->print_time + m->move_t > print_time)
            break;
        list_del(&m->node);
        if (m->start_v || m->half_accel || m->sixth_jerk)
            list_add_head(&m->node, &tq->history);
        else
            free(m);
    }
    // Free old moves from history list
    if (list_empty(&tq->history))
        return;
    struct move *latest = list_first_entry(&tq->history, struct move, node);
    for (;;) {
        struct move *m = list_last_entry(&tq->history, struct move, node);
        if (m == latest || m->print_time + m->move_t > clear_history_time)
            break;
        list_del(&m->node);
        free(m);
    }
}

// Note a position change in the trapq history
void __visible
trapq_set_position(struct trapq *tq, double print_time
                   , double pos_x, double pos_y, double pos_z)
{
    // Flush all moves from trapq
    trapq_finalize_moves(tq, NEVER_TIME, 0);

    // Prune any moves in the trapq history that were interrupted
    while (!list_empty(&tq->history)) {
        struct move *m = list_first_entry(&tq->history, struct move, node);
        if (m->print_time < print_time) {
            if (m->print_time + m->move_t > print_time)
                m->move_t = print_time - m->print_time;
            break;
        }
        list_del(&m->node);
        free(m);
    }

    // Add a marker to the trapq history
    struct move *m = move_alloc();
    m->print_time = print_time;
    m->start_pos.x = pos_x;
    m->start_pos.y = pos_y;
    m->start_pos.z = pos_z;
    list_add_head(&m->node, &tq->history);
}

// Copy the info in a 'struct move' to a 'struct pull_move'
static void
copy_pull_move(struct pull_move *p, struct move *m)
{
    p->print_time = m->print_time;
    p->move_t = m->move_t;
    p->start_v = m->start_v;
    p->accel = 2. * m->half_accel;
    p->jerk = 6. * m->sixth_jerk;
    p->start_x = m->start_pos.x;
    p->start_y = m->start_pos.y;
    p->start_z = m->start_pos.z;
    p->x_r = m->axes_r.x;
    p->y_r = m->axes_r.y;
    p->z_r = m->axes_r.z;
}

// Return history of movement queue
int __visible
trapq_extract_old(struct trapq *tq, struct pull_move *p, int max
                  , double start_time, double end_time)
{
    int res = 0;
    struct move *m;
    struct move *head_sentinel = list_first_entry(&tq->moves, struct move, node);
    m = list_last_entry(&tq->moves, struct move, node);
    for ( ; m != head_sentinel; m = list_prev_entry(m, node)) {
        if (start_time >= m->print_time + m->move_t || res >= max)
            break;
        if (end_time <= m->print_time
            || (!m->start_v && !m->half_accel && !m->sixth_jerk))
            continue;
        copy_pull_move(p, m);
        p++;
        res++;
    }
    list_for_each_entry(m, &tq->history, node) {
        if (start_time >= m->print_time + m->move_t || res >= max)
            break;
        if (end_time <= m->print_time)
            continue;
        copy_pull_move(p, m);
        p++;
        res++;
    }
    return res;
}
