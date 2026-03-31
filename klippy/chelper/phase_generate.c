// Phase stepping position generator
//
// Samples the trapq at fixed time intervals using the same kinematics
// callbacks as itersolve, producing a stream of position samples for
// the phase compressor.
//
// Copyright (C) 2026  klipper contributors
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // stepper_kinematics
#include "trapq.h" // struct move
#include "phase_generate.h" // phase_generator

#define PHASE_GENERATOR_NEVER_TIME 9999999999999999.9

// Allocate a phase generator
struct phase_generator * __visible
phase_generator_alloc(void)
{
    struct phase_generator *pg = malloc(sizeof(*pg));
    memset(pg, 0, sizeof(*pg));
    return pg;
}

// Configure the phase generator
void __visible
phase_generator_config(struct phase_generator *pg
                       , struct stepper_kinematics *sk
                       , double step_dist, double update_interval)
{
    pg->sk = sk;
    pg->step_dist = step_dist;
    pg->update_interval = update_interval;
    pg->last_flush_time = 0.;
    pg->direction = 1.;
}

// Set the last flush time (used to sync after reactivation)
void __visible
phase_generator_set_time(struct phase_generator *pg, double time)
{
    pg->last_flush_time = time;
}

// Set the last known phase position used to hold idle windows steady.
void __visible
phase_generator_set_last_position(struct phase_generator *pg, double pos)
{
    pg->last_position = pos;
    pg->have_last_position = 1;
}

// Set the phase offset (difference between klipper position and TMC phase)
void __visible
phase_generator_set_offset(struct phase_generator *pg, double offset)
{
    pg->phase_offset = offset;
}

// Set the direction multiplier (+1.0 or -1.0 for dir_pin inversion)
void __visible
phase_generator_set_direction(struct phase_generator *pg, double dir)
{
    pg->direction = dir;
}

// Extract positions from samples into a flat array, checking for NaN/inf.
// Returns num_samples on success, -1 if NaN or inf encountered.
int __visible
phase_generator_extract_positions(struct phase_sample *samples
                                  , int num_samples, double *positions)
{
    for (int i = 0; i < num_samples; i++) {
        double pos = samples[i].position;
        if (!__builtin_isfinite(pos))
            return -1;
        positions[i] = pos;
    }
    return num_samples;
}

// Find the first moving sample index where the position changes by more than
// `threshold` from positions[0]. Returns 0 if the very first sample differs,
// or num_samples if all samples are within threshold (pure standstill).
// Used to strip leading standstill from mixed flushes.
int __visible
phase_generator_find_move_start(double *positions, int num_samples
                                , double threshold)
{
    if (num_samples <= 1)
        return 0;
    double base = positions[0];
    for (int i = 1; i < num_samples; i++) {
        double diff = positions[i] - base;
        if (diff > threshold || diff < -threshold)
            return i;
    }
    return num_samples; // all standstill
}

static inline int
phase_generator_check_active(struct stepper_kinematics *sk, struct move *m)
{
    int af = sk->active_flags;
    return ((af & AF_X && m->axes_r.x != 0.)
            || (af & AF_Y && m->axes_r.y != 0.)
            || (af & AF_Z && m->axes_r.z != 0.));
}

static struct move *
phase_generator_find_prev_active(struct stepper_kinematics *sk
                                 , struct move *m, struct move *head)
{
    while (m != head) {
        m = list_prev_entry(m, node);
        if (m == head)
            return NULL;
        if (phase_generator_check_active(sk, m))
            return m;
    }
    return NULL;
}

static struct move *
phase_generator_find_next_active(struct stepper_kinematics *sk, struct move *m)
{
    for (;;) {
        if (phase_generator_check_active(sk, m))
            return m;
        if (m->move_t == PHASE_GENERATOR_NEVER_TIME)
            return NULL;
        m = list_next_entry(m, node);
    }
}

// Generate position samples from trapq between last_flush_time and flush_time.
// Writes samples into the provided buffer. Returns number of samples written.
int __visible
phase_generator_generate(struct phase_generator *pg
                         , double flush_time
                         , struct phase_sample *samples, int max_samples)
{
    struct stepper_kinematics *sk = pg->sk;
    if (!sk || !sk->tq)
        return 0;

    double interval = pg->update_interval;
    double inv_step_dist = 1. / pg->step_dist;
    double t = pg->last_flush_time;
    int count = 0;

    // Walk through the trapq, sampling position at each interval
    trapq_check_sentinels(sk->tq);
    struct move *head = list_first_entry(&sk->tq->moves, struct move, node);
    struct move *m = head;
    while (t >= m->print_time + m->move_t)
        m = list_next_entry(m, node);
    struct move *last_active = phase_generator_find_prev_active(sk, m, head);
    struct move *next_active = phase_generator_find_next_active(sk, m);

    while (t < flush_time && count < max_samples) {
        // Advance to the move containing time t
        while (t >= m->print_time + m->move_t) {
            if (phase_generator_check_active(sk, m))
                last_active = m;
            m = list_next_entry(m, node);
        }
        if (next_active && t >= next_active->print_time + next_active->move_t)
            next_active = phase_generator_find_next_active(sk, m);

        // Only evaluate active moves (or kinematics pre/post-active windows).
        // Outside those windows, hold the last valid phase instead of sampling
        // unrelated trapq moves, which can return invalid values.
        struct move *sample_move = NULL;
        double move_time = 0.;
        if (phase_generator_check_active(sk, m)) {
            sample_move = m;
            move_time = t - m->print_time;
            last_active = m;
        } else if (last_active && sk->gen_steps_post_active > 0.
                   && t < last_active->print_time + last_active->move_t
                          + sk->gen_steps_post_active) {
            sample_move = last_active;
            move_time = t - last_active->print_time;
        } else if (next_active && sk->gen_steps_pre_active > 0.
                   && t >= next_active->print_time - sk->gen_steps_pre_active) {
            sample_move = next_active;
            move_time = t - next_active->print_time;
        }

        samples[count].time = t;
        if (sample_move) {
            double pos = sk->calc_position_cb(sk, sample_move, move_time);
            double phase_pos = pos * inv_step_dist * pg->direction
                               + pg->phase_offset;
            samples[count].position = phase_pos;
            pg->last_position = phase_pos;
            pg->have_last_position = 1;
        } else if (pg->have_last_position) {
            samples[count].position = pg->last_position;
        } else {
            double pos = sk->calc_position_cb(sk, m, t - m->print_time);
            double phase_pos = pos * inv_step_dist * pg->direction
                               + pg->phase_offset;
            samples[count].position = phase_pos;
            pg->last_position = phase_pos;
            pg->have_last_position = 1;
        }
        count++;
        t += interval;
    }

    pg->last_flush_time = t;
    return count;
}
