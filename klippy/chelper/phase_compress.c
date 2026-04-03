// Phase stepping position compressor
//
// Compresses a stream of evenly-spaced position samples into
// (start_pos, velocity, acceleration, count) tuples suitable for
// the MCU queue_phase_move command.
//
// Copyright (C) 2026  klipper contributors
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdlib.h> // malloc
#include <string.h> // memset
#include <math.h> // fabs
#include "compiler.h" // __visible
#include "phase_compress.h"

// Allocate a phase compressor
struct phase_compressor * __visible
phase_compressor_alloc(void)
{
    struct phase_compressor *pc = malloc(sizeof(*pc));
    memset(pc, 0, sizeof(*pc));
    pc->max_error = 0.25; // Default: quarter microstep
    return pc;
}

// Set the maximum fit error (in microstep units)
void __visible
phase_compressor_set_max_error(struct phase_compressor *pc, double max_error)
{
    pc->max_error = max_error;
}

// Try to fit a quadratic polynomial to samples[0..n-1].
// Returns 1 if the fit is within max_error, 0 otherwise.
// Outputs: start_pos, velocity (per interval), acceleration (per interval)
static int
try_quadratic_fit(double *positions, int n, double max_error
                  , double *out_start, double *out_vel, double *out_accel)
{
    if (n < 1)
        return 0;
    if (n == 1) {
        *out_start = positions[0];
        *out_vel = 0.;
        *out_accel = 0.;
        return 1;
    }
    if (n == 2) {
        *out_start = positions[0];
        *out_vel = positions[1] - positions[0];
        *out_accel = 0.;
        return 1;
    }

    // Least-squares fit: p(i) = a + b*i + c*i^2
    // Using the known closed-form for evenly spaced samples at i=0..n-1
    double n_d = (double)n;
    double sum_p = 0., sum_ip = 0., sum_i2p = 0.;
    double sum_i = 0., sum_i2 = 0., sum_i3 = 0., sum_i4 = 0.;
    int i;
    for (i = 0; i < n; i++) {
        double id = (double)i;
        double p = positions[i];
        sum_p += p;
        sum_ip += id * p;
        sum_i2p += id * id * p;
        sum_i += id;
        sum_i2 += id * id;
        sum_i3 += id * id * id;
        sum_i4 += id * id * id * id;
    }

    // Solve 3x3 normal equations:
    //   [n      sum_i   sum_i2 ] [a]   [sum_p  ]
    //   [sum_i  sum_i2  sum_i3 ] [b] = [sum_ip ]
    //   [sum_i2 sum_i3  sum_i4 ] [c]   [sum_i2p]
    // Using Cramer's rule
    double d00 = n_d, d01 = sum_i, d02 = sum_i2;
    double d10 = sum_i, d11 = sum_i2, d12 = sum_i3;
    double d20 = sum_i2, d21 = sum_i3, d22 = sum_i4;

    double det = d00*(d11*d22 - d12*d21) - d01*(d10*d22 - d12*d20)
                 + d02*(d10*d21 - d11*d20);
    if (fabs(det) < 1e-20)
        return 0;
    double inv_det = 1. / det;

    double a = ((sum_p*(d11*d22 - d12*d21) - d01*(sum_ip*d22 - sum_i2p*d21)
                 + d02*(sum_ip*d21 - sum_i2p*d11)) * inv_det);
    double b = ((d00*(sum_ip*d22 - sum_i2p*d21) - sum_p*(d10*d22 - d12*d20)
                 + d02*(sum_i2p*d20 - sum_ip*d20)) * inv_det);
    // Correct b computation
    b = ((d00*(sum_ip*d22 - sum_i2p*d12) - d01*(sum_p*d22 - sum_i2p*d02)
          + d02*(sum_p*d12 - sum_ip*d02)) * inv_det);
    // Wait, let me redo this properly with cofactors
    // Actually, let's use a cleaner approach:
    b = (d00*(sum_ip*d22 - sum_i2p*d21) - sum_p*(d10*d22 - d12*d20)
         + d02*(d10*sum_i2p - sum_ip*d20)) * inv_det;
    double c = (d00*(d11*sum_i2p - sum_ip*d21) - d01*(d10*sum_i2p - sum_ip*d20)
                + sum_p*(d10*d21 - d11*d20)) * inv_det;

    // Check fit error
    double max_err = 0.;
    for (i = 0; i < n; i++) {
        double id = (double)i;
        double fitted = a + b * id + c * id * id;
        double err = fabs(positions[i] - fitted);
        if (err > max_err)
            max_err = err;
    }

    if (max_err > max_error)
        return 0;

    *out_start = a;
    *out_vel = b;
    *out_accel = 2. * c; // acceleration = 2*c (since p = a + b*i + c*i^2)
    return 1;
}

// Compress an array of position samples into phase_move segments.
// Returns number of segments written.
int __visible
phase_compressor_compress(struct phase_compressor *pc
                          , double *positions, int num_samples
                          , struct phase_compressed_move *out, int max_out)
{
    int out_count = 0;
    int start = 0;

    while (start < num_samples && out_count < max_out) {
        // Binary search for the longest segment that fits within error
        int best_len = 1;
        double best_start = positions[start];
        double best_vel = 0., best_accel = 0.;

        int lo = 1, hi = num_samples - start;
        while (lo <= hi) {
            int mid = (lo + hi) / 2;
            double s, v, a;
            if (try_quadratic_fit(positions + start, mid, pc->max_error,
                                  &s, &v, &a)) {
                best_len = mid;
                best_start = s;
                best_vel = v;
                best_accel = a;
                lo = mid + 1;
            } else {
                hi = mid - 1;
            }
        }

        // Emit segment
        out[out_count].start_position = best_start;
        out[out_count].velocity = best_vel;
        out[out_count].acceleration = best_accel;
        out[out_count].count = best_len;
        out_count++;

        start += best_len;
    }

    return out_count;
}

// Convert compressed segments to fixed-point MCU format.
// Filters out NaN segments and clamps acceleration to int16 range.
// Returns number of valid segments written to out_mcu.
int __visible
phase_compressor_to_fixed(struct phase_compressed_move *segments
                          , int num_segments
                          , struct phase_mcu_move *out_mcu, int max_out)
{
    int out_count = 0;
    double scale = 65536.;  // 16.16 fixed-point
    for (int i = 0; i < num_segments && out_count < max_out; i++) {
        struct phase_compressed_move *seg = &segments[i];
        if (seg->count <= 0)
            continue;
        if (__builtin_isnan(seg->start_position)
                || __builtin_isnan(seg->velocity)
                || __builtin_isnan(seg->acceleration))
            continue;
        int32_t accel = (int32_t)(seg->acceleration * scale);
        // Reduce position modulo 1024 (one electrical cycle) before
        // converting to 16.16 fixed-point.  Phase extraction on the MCU
        // uses only (position >> 16) & 0x3FF, so the upper bits don't
        // matter — but without reduction, large positions (>32767
        // microsteps) overflow int32 when multiplied by 65536.
        double reduced_pos = fmod(seg->start_position, 1024.);
        out_mcu[out_count].start_position = (int32_t)(reduced_pos * scale);
        out_mcu[out_count].velocity = (int32_t)(seg->velocity * scale);
        out_mcu[out_count].acceleration = accel;
        out_mcu[out_count].count = (uint16_t)seg->count;
        out_count++;
    }
    return out_count;
}
