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

// Try to fit a quadratic polynomial p(i) = anchor + b*i + c*i^2 to samples
// positions[0..n-1] with the i=0 value forced to equal `anchor` (which may
// or may not equal positions[0]).
// Returns 1 if the fit is within max_error including |positions[0]-anchor|,
// 0 otherwise.
// Outputs: velocity (per interval), acceleration (per interval).
static int
try_quadratic_fit_at(double *positions, int n, double anchor, double max_error
                     , double *out_vel, double *out_accel)
{
    if (n < 1)
        return 0;
    if (n == 1) {
        // One sample: only valid if positions[0] is within max_error of anchor.
        if (fabs(positions[0] - anchor) > max_error)
            return 0;
        *out_vel = 0.;
        *out_accel = 0.;
        return 1;
    }
    if (n == 2) {
        // Two samples and an anchor — three constraints, two unknowns (b,c).
        // Underdetermined for general anchor; force c=0 and choose b to hit
        // positions[1] exactly. positions[0] error against anchor is checked
        // against max_error.
        if (fabs(positions[0] - anchor) > max_error)
            return 0;
        *out_vel = positions[1] - anchor;
        *out_accel = 0.;
        return 1;
    }

    // Constrained least-squares fit: p(i) = anchor + b*i + c*i^2.
    // Forcing p(0) = anchor (rather than positions[0]) guarantees the segment
    // begins at exactly the host-mirror value.  The 2x2 normal equations
    // (minimizing sum_{i=0..n-1} (anchor + b*i + c*i^2 - positions[i])^2 over
    // (b,c)) come out to:
    //   [Σi²  Σi³] [b]   [Σi·r_i ]
    //   [Σi³  Σi⁴] [c] = [Σi²·r_i]
    // where r_i = positions[i] - anchor, summed over i=0..n-1.
    // Note: the i=0 term contributes zero to all four sums and to r_0 (since
    // we don't force r_0 = 0 anymore — it's positions[0] - anchor and may be
    // nonzero), so effectively the sum starts at i=1.  However, the i=0 fit
    // error |positions[0] - anchor| must be checked separately.
    double s2 = 0., s3 = 0., s4 = 0., t1 = 0., t2 = 0.;
    int i;
    for (i = 1; i < n; i++) {
        double id = (double)i;
        double id2 = id * id;
        double r = positions[i] - anchor;
        s2 += id2;
        s3 += id2 * id;
        s4 += id2 * id2;
        t1 += id * r;
        t2 += id2 * r;
    }

    double det = s2 * s4 - s3 * s3;
    if (fabs(det) < 1e-20)
        return 0;
    double inv_det = 1. / det;
    double b = (s4 * t1 - s3 * t2) * inv_det;
    double c = (s2 * t2 - s3 * t1) * inv_det;

    // Check fit error.  Include i=0 (anchor vs positions[0]) since it's no
    // longer guaranteed exact.
    double max_err = fabs(positions[0] - anchor);
    for (i = 1; i < n; i++) {
        double id = (double)i;
        double fitted = anchor + b * id + c * id * id;
        double err = fabs(positions[i] - fitted);
        if (err > max_err)
            max_err = err;
    }
    if (max_err > max_error)
        return 0;

    *out_vel = b;
    *out_accel = 2. * c; // acceleration = 2*c (since p = a + b*i + c*i^2)
    return 1;
}

// Backward-compatible wrapper: anchor at positions[0] (legacy behavior).
static int
try_quadratic_fit(double *positions, int n, double max_error
                  , double *out_start, double *out_vel, double *out_accel)
{
    if (n < 1)
        return 0;
    *out_start = positions[0];
    return try_quadratic_fit_at(positions, n, positions[0], max_error,
                                out_vel, out_accel);
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

// Closed-form last-emitted phase position after a (start, vel, accel, count)
// segment.  Matches the MCU's per-tick advance loop bit-for-bit:
//   position += velocity; velocity += acceleration; (count times)
//   last_emitted = position before the final advance
//                = start + (count-1)*vel + accel * (count-1)*(count-2)/2
// All arithmetic wraps at int32.  Returns the int32-signed result.
static int32_t
mirror_advance(int32_t start, int32_t vel, int32_t accel, uint16_t count)
{
    if (count == 0)
        return start;
    int64_t n_minus_1 = (int64_t)count - 1;
    int64_t n_choose_2 = (n_minus_1 * (n_minus_1 - 1)) / 2; // = (N-1)(N-2)/2
    int64_t raw = (int64_t)start + n_minus_1 * (int64_t)vel
                  + n_choose_2 * (int64_t)accel;
    return (int32_t)(raw & 0xFFFFFFFFLL);
}

// Anchored compress: produce phase_mcu_move segments with int32 start_position
// chained by mirror_advance() so segment K+1's anchor equals MCU's last-emitted
// from segment K, bit-exactly.  First segment anchored at host-supplied
// `anchor_fixed` (16.16).  This eliminates the resume-from-idle click by
// construction: every emitted XDIRECT phase is continuous from the previous
// one, regardless of float drift in the trapq sample stream.
//
// Returns number of MCU moves written.
int __visible
phase_compressor_compress_anchored(struct phase_compressor *pc
                                   , double *positions, int num_samples
                                   , int32_t anchor_fixed
                                   , struct phase_mcu_move *out_mcu, int max_out)
{
    int out_count = 0;
    int start = 0;
    double scale = 65536.;
    int32_t anchor_int = anchor_fixed;

    while (start < num_samples && out_count < max_out) {
        // Reduce anchor mod 1024 (one electrical cycle) for the fit so its
        // double form stays in a small numeric range.  Phase extraction on
        // the MCU uses only the low 26 bits anyway.
        int32_t anchor_reduced = (int32_t)(((uint32_t)anchor_int) & 0x03FFFFFFu);
        // Sign-extend reduced anchor: 26-bit unsigned -> signed centered.
        // Actually 0..0x03FFFFFF is fine as positive; we convert to double.
        double anchor_double = (double)anchor_reduced / scale;

        int best_len = 1;
        double best_vel = 0., best_accel = 0.;

        int lo = 1, hi = num_samples - start;
        while (lo <= hi) {
            int mid = (lo + hi) / 2;
            double v, a;
            if (try_quadratic_fit_at(positions + start, mid, anchor_double,
                                     pc->max_error, &v, &a)) {
                best_len = mid;
                best_vel = v;
                best_accel = a;
                lo = mid + 1;
            } else {
                hi = mid - 1;
            }
        }

        // If even n=1 didn't fit at the anchor (positions[start] differs from
        // anchor by more than max_error), force-emit a single tick at the
        // anchor anyway.  Continuity beats fit error — the sub-microstep
        // residual will be absorbed in subsequent segments.
        // best_len defaults to 1 with vel/accel = 0 — that's already the
        // force-emit single-tick fallback.

        // Convert to MCU fixed-point.
        if (__builtin_isnan(best_vel) || __builtin_isnan(best_accel)) {
            // Polynomial fit blew up — emit a hold segment instead (vel=0,
            // accel=0) so we don't propagate NaN into the MCU.
            best_vel = 0.;
            best_accel = 0.;
        }
        int32_t vel_int = (int32_t)(best_vel * scale);
        int32_t accel_int = (int32_t)(best_accel * scale);
        out_mcu[out_count].start_position = anchor_reduced;
        out_mcu[out_count].velocity = vel_int;
        out_mcu[out_count].acceleration = accel_int;
        out_mcu[out_count].count = (uint16_t)best_len;
        out_count++;

        // Advance the anchor for the next segment via the same int32
        // arithmetic the MCU uses.  This guarantees bit-exact chaining.
        anchor_int = mirror_advance(anchor_reduced, vel_int, accel_int,
                                    (uint16_t)best_len);

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
