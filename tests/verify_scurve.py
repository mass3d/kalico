#!/usr/bin/env python3
"""
End-to-end S-curve verification.
Replicates the Python _set_junction_scurve() logic, queues via C,
and verifies the output matches the commanded move distance exactly.
"""

import sys, os, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'klippy'))
import chelper

def get_ffi():
    return chelper.get_ffi()

def compute_scurve_params(move_d, start_v, cruise_v, end_v, accel, jerk_max):
    """Replicate _set_junction_scurve() from toolhead.py"""
    tj_max = accel / jerk_max

    # Acceleration side
    delta_v_accel = cruise_v - start_v
    if delta_v_accel < 1e-10:
        tj1 = ta = 0.
    else:
        v_j = 0.5 * jerk_max * tj_max * tj_max
        if delta_v_accel < 2. * v_j:
            tj1 = math.sqrt(delta_v_accel / jerk_max)
            ta = 0.
        else:
            tj1 = tj_max
            ta = (delta_v_accel - 2. * v_j) / accel

    # Deceleration side
    delta_v_decel = cruise_v - end_v
    if delta_v_decel < 1e-10:
        tj2 = td = 0.
    else:
        v_j = 0.5 * jerk_max * tj_max * tj_max
        if delta_v_decel < 2. * v_j:
            tj2 = math.sqrt(delta_v_decel / jerk_max)
            td = 0.
        else:
            tj2 = tj_max
            td = (delta_v_decel - 2. * v_j) / accel

    # Distance formulas (corrected)
    accel_d = (start_v * (2.*tj1 + ta)
               + jerk_max * tj1 * (tj1*tj1 + 1.5*tj1*ta + .5*ta*ta))
    decel_d = (cruise_v * (2.*tj2 + td)
               - jerk_max * tj2 * (tj2*tj2 + 1.5*tj2*td + .5*td*td))
    cruise_d = move_d - accel_d - decel_d

    if cruise_d < -1e-10:
        return None  # Would fall back to trapezoidal

    tc = cruise_d / cruise_v if cruise_v > 0. else 0.
    return {
        'tj1': tj1, 'ta': ta, 'tc': tc, 'tj2': tj2, 'td': td,
        'accel_d': accel_d, 'decel_d': decel_d, 'cruise_d': cruise_d,
        'total_time': 2*tj1 + ta + tc + 2*tj2 + td,
    }

def extract_moves_sorted(ffi_main, ffi_lib, tq):
    pm = ffi_main.new('struct pull_move[32]')
    count = ffi_lib.trapq_extract_old(tq, pm, 32, 0.0, 9999.0)
    moves = []
    for i in range(count):
        moves.append({
            'print_time': pm[i].print_time,
            'move_t': pm[i].move_t,
            'start_v': pm[i].start_v,
            'accel': pm[i].accel,
            'jerk': pm[i].jerk,
            'start_x': pm[i].start_x,
        })
    moves.sort(key=lambda m: m['print_time'])
    return moves

def verify_move(name, move_d, start_v, cruise_v, end_v, accel, jerk_max):
    """Verify a single S-curve move end-to-end."""
    print(f"\n{'='*60}")
    print(f"  {name}")
    print(f"  move_d={move_d}, start_v={start_v}, cruise_v={cruise_v}, "
          f"end_v={end_v}")
    print(f"  accel={accel}, jerk={jerk_max}")
    print(f"{'='*60}")

    params = compute_scurve_params(move_d, start_v, cruise_v, end_v,
                                    accel, jerk_max)
    if params is None:
        print("  SKIP: Move too short for S-curve (trapezoidal fallback)")
        return True

    print(f"  Phase durations: tj1={params['tj1']:.6f} ta={params['ta']:.6f} "
          f"tc={params['tc']:.6f} tj2={params['tj2']:.6f} td={params['td']:.6f}")
    print(f"  Distances: accel={params['accel_d']:.6f} cruise={params['cruise_d']:.6f} "
          f"decel={params['decel_d']:.6f}")
    print(f"  Sum = {params['accel_d']+params['cruise_d']+params['decel_d']:.6f} "
          f"(target: {move_d})")

    # Queue via C
    ffi_main, ffi_lib = get_ffi()
    tq = ffi_lib.trapq_alloc()
    ffi_lib.trapq_append_scurve(tq, 0.0,
                                 0.0, 0.0, 0.0,
                                 1.0, 0.0, 0.0,
                                 start_v, jerk_max,
                                 accel, cruise_v,
                                 params['tj1'], params['ta'], params['tc'],
                                 params['tj2'], params['td'])

    moves = extract_moves_sorted(ffi_main, ffi_lib, tq)
    ffi_lib.trapq_free(tq)

    errors = []

    # 1. Verify total distance
    total_dist = 0.
    for m in moves:
        t = m['move_t']
        v = m['start_v']
        a = m['accel']
        j = m['jerk']
        d = (v + (a/2. + j/6. * t) * t) * t
        total_dist += d

    dist_err = abs(total_dist - move_d)
    print(f"\n  Total distance from C: {total_dist:.6f} (error: {dist_err:.2e})")
    if dist_err > 0.001:
        errors.append(f"Distance error {dist_err:.6f} > 0.001")

    # 2. Verify velocity continuity
    max_v_err = 0.
    max_a_err = 0.
    for i, m in enumerate(moves):
        t = m['move_t']
        v0 = m['start_v']
        a0 = m['accel']
        j = m['jerk']
        end_v = v0 + a0 * t + 0.5 * j * t * t
        end_a = a0 + j * t

        if i + 1 < len(moves):
            next_v = moves[i+1]['start_v']
            next_a = moves[i+1]['accel']
            v_err = abs(end_v - next_v)
            a_err = abs(end_a - next_a)
            max_v_err = max(max_v_err, v_err)
            max_a_err = max(max_a_err, a_err)

    print(f"  Max velocity discontinuity: {max_v_err:.2e}")
    print(f"  Max accel discontinuity: {max_a_err:.2e}")
    if max_v_err > 0.01:
        errors.append(f"Velocity discontinuity {max_v_err}")
    if max_a_err > 0.1:
        errors.append(f"Accel discontinuity {max_a_err}")

    # 3. Verify start and end velocities
    first_v = moves[0]['start_v']
    last_m = moves[-1]
    last_end_v = (last_m['start_v'] + last_m['accel'] * last_m['move_t']
                  + 0.5 * last_m['jerk'] * last_m['move_t']**2)

    sv_err = abs(first_v - start_v)
    ev_err = abs(last_end_v - end_v)
    print(f"  Start velocity: {first_v:.4f} (expected {start_v}, err {sv_err:.2e})")
    print(f"  End velocity: {last_end_v:.4f} (expected {end_v}, err {ev_err:.2e})")
    if sv_err > 0.01:
        errors.append(f"Start velocity error {sv_err}")
    if ev_err > 0.01:
        errors.append(f"End velocity error {ev_err}")

    # 4. Print velocity profile at key points
    print(f"\n  {'Time':>8s} {'Vel':>10s} {'Accel':>10s} {'Phase':>6s}")
    print(f"  {'-'*38}")
    abs_time = 0.
    for i, m in enumerate(moves):
        t = m['move_t']
        v0 = m['start_v']
        a0 = m['accel']
        j = m['jerk']
        # Print start and end of each phase
        for frac in [0., 1.]:
            tt = frac * t
            vel = v0 + a0*tt + 0.5*j*tt*tt
            acc = a0 + j*tt
            print(f"  {abs_time+tt:8.5f} {vel:10.3f} {acc:10.1f} {i+1:>6d}")
        abs_time += t

    if errors:
        print(f"\n  FAIL: {errors}")
        return False
    else:
        print(f"\n  PASS")
        return True

def simulate_mcu_phase(position_fixed):
    """Simulate the MCU ISR: (position >> 16) & 0x3FF"""
    pos = position_fixed & 0xFFFFFFFF
    if pos >= 0x80000000:
        pos -= 0x100000000
    return (pos >> 16) & 0x3FF

def verify_phase_pipeline(name, move_d, cruise_v, accel, jerk_max,
                          microsteps):
    """Verify the full phase stepping pipeline produces correct MSCNT data
    for an S-curve move.

    Creates a trapq move, feeds it through phase_generator + compressor +
    to_fixed, then simulates the MCU ISR to verify phase progression."""
    print(f"\n{'='*60}")
    print(f"  {name}")
    print(f"  move_d={move_d}mm cruise_v={cruise_v}mm/s microsteps={microsteps}")
    print(f"{'='*60}")

    ffi_main, ffi_lib = get_ffi()
    errors = []

    # Motor/belt parameters: 200-step motor, GT2 20T pulley (40mm/rev)
    rotation_distance = 40.0
    full_steps_per_rev = 200
    step_dist = rotation_distance / (full_steps_per_rev * microsteps)
    update_rate = 10000.0
    update_interval = 1.0 / update_rate

    # The fix: scale step_dist for MSCNT units
    phase_step_dist = step_dist * microsteps / 256.0
    scale_factor = 256.0 / microsteps

    print(f"  step_dist={step_dist:.6f}mm  phase_step_dist={phase_step_dist:.6f}mm"
          f"  scale={scale_factor:.0f}x")

    # Compute S-curve parameters
    params = compute_scurve_params(move_d, 0., cruise_v, 0., accel, jerk_max)
    if params is None:
        print("  SKIP: Move too short for S-curve")
        return True

    start_time = 1.0  # offset from t=0
    total_time = params['total_time']
    print(f"  S-curve duration: {total_time:.4f}s")

    # Create trapq with S-curve move
    tq = ffi_lib.trapq_alloc()
    ffi_lib.trapq_append_scurve(tq, start_time,
                                 0.0, 0.0, 0.0,    # start at origin
                                 1.0, 0.0, 0.0,    # X axis
                                 0., jerk_max, accel, cruise_v,
                                 params['tj1'], params['ta'], params['tc'],
                                 params['tj2'], params['td'])

    # Set up cartesian X kinematics
    sk = ffi_lib.cartesian_stepper_alloc(b'x')
    ffi_lib.itersolve_set_trapq(sk, tq)

    # Configure phase generator with CORRECTED scaling
    pg = ffi_main.gc(ffi_lib.phase_generator_alloc(), ffi_lib.free)
    ffi_lib.phase_generator_config(pg, sk, phase_step_dist, update_interval)
    ffi_lib.phase_generator_set_direction(pg, 1.0)
    ffi_lib.phase_generator_set_offset(pg, 0.0)  # zero offset for clean test
    ffi_lib.phase_generator_set_time(pg, start_time)

    # Generate samples
    max_samples = 8192
    samples = ffi_main.new('struct phase_sample[]', max_samples)
    flush_time = start_time + total_time + 0.01
    num_samples = ffi_lib.phase_generator_generate(
        pg, flush_time, samples, max_samples)

    if num_samples <= 0:
        errors.append("Phase generator produced 0 samples")
        print(f"\n  FAIL: {errors}")
        ffi_lib.trapq_free(tq)
        return False

    # Extract positions
    positions = ffi_main.new('double[]', max_samples)
    ret = ffi_lib.phase_generator_extract_positions(
        samples, num_samples, positions)
    if ret < 0:
        errors.append("NaN in phase generator output")
        ffi_lib.trapq_free(tq)
        print(f"\n  FAIL: {errors}")
        return False

    # Compress
    max_segments = 1024
    pc = ffi_main.gc(ffi_lib.phase_compressor_alloc(), ffi_lib.free)
    max_error = 0.25 * scale_factor  # scaled to MSCNT units
    ffi_lib.phase_compressor_set_max_error(pc, max_error)
    segments = ffi_main.new('struct phase_compressed_move[]', max_segments)
    num_seg = ffi_lib.phase_compressor_compress(
        pc, positions, num_samples, segments, max_segments)

    # Convert to fixed-point MCU format
    mcu_moves = ffi_main.new('struct phase_mcu_move[]', max_segments)
    num_mcu = ffi_lib.phase_compressor_to_fixed(
        segments, num_seg, mcu_moves, max_segments)

    print(f"  Samples: {num_samples}  Segments: {num_seg}  MCU moves: {num_mcu}")

    if num_mcu <= 0:
        errors.append(f"No MCU moves (segments={num_seg})")
        ffi_lib.trapq_free(tq)
        print(f"\n  FAIL: {errors}")
        return False

    # ---- Simulate MCU ISR and extract phase progression ----
    phases = []
    velocities_fixed = []
    for i in range(num_mcu):
        m = mcu_moves[i]
        pos = m.start_position
        vel = m.velocity
        for k in range(m.count):
            pos += vel
            vel += m.acceleration
            phase = simulate_mcu_phase(pos)
            phases.append(phase)
            if k == 0:
                velocities_fixed.append(m.velocity)

    # ---- Verify: total phase rotation matches expected distance ----
    total_phase_change = 0
    for i in range(1, len(phases)):
        delta = (phases[i] - phases[i-1]) & 0x3FF
        if delta > 512:
            delta -= 1024
        total_phase_change += delta

    # Expected: move_d / step_dist * (256/microsteps) MSCNT
    expected_mscnt = move_d / step_dist * scale_factor
    phase_ratio = total_phase_change / expected_mscnt if expected_mscnt else 0

    print(f"\n  Phase rotation: {total_phase_change:.0f} MSCNT"
          f"  (expected {expected_mscnt:.0f})")
    print(f"  Ratio: {phase_ratio:.4f} (should be ~1.0)")

    if abs(phase_ratio - 1.0) > 0.02:
        errors.append(f"Phase rotation ratio {phase_ratio:.4f} "
                      f"(off by {abs(phase_ratio-1)*100:.1f}%)")

    # ---- Verify: cruise velocity matches expected MSCNT rate ----
    # Find the largest constant-velocity MCU segment
    cruise_seg = None
    for i in range(num_mcu):
        m = mcu_moves[i]
        if m.count > 50 and abs(m.acceleration) <= 2:
            if cruise_seg is None or m.count > cruise_seg.count:
                cruise_seg = m

    expected_vel = cruise_v / step_dist * scale_factor / update_rate * 65536.
    if cruise_seg and abs(cruise_seg.velocity) > abs(expected_vel) * 0.5:
        actual_vel = cruise_seg.velocity
        vel_ratio = actual_vel / expected_vel if expected_vel else 0

        print(f"\n  Cruise velocity (16.16): {actual_vel}"
              f"  expected: {expected_vel:.0f}"
              f"  ratio: {vel_ratio:.4f}")

        if abs(vel_ratio - 1.0) > 0.01:
            errors.append(f"Cruise velocity ratio {vel_ratio:.4f}")

        # Show what the OLD (broken) velocity would have been
        old_vel = cruise_v / step_dist / update_rate * 65536.
        print(f"  OLD (broken) velocity would be: {old_vel:.0f}"
              f"  ({scale_factor:.0f}x too slow)")
    else:
        print(f"  (no pure cruise segment found for velocity check)")
        print(f"  Expected cruise velocity (16.16): {expected_vel:.0f}")
        old_vel = cruise_v / step_dist / update_rate * 65536.
        print(f"  OLD (broken) velocity would be: {old_vel:.0f}"
              f"  ({scale_factor:.0f}x too slow)")

    # ---- Print phase progression at key moments ----
    n_phases = len(phases)
    print(f"\n  Phase progression ({n_phases} ISR events):")
    print(f"  {'Event':>7s} {'Time(ms)':>9s} {'Phase':>6s} {'Delta':>6s}")
    print(f"  {'-'*32}")

    step_size = max(1, n_phases // 15)
    prev_phase = phases[0] if phases else 0
    for i in range(0, n_phases, step_size):
        t_ms = i * update_interval * 1000.
        delta = (phases[i] - prev_phase) & 0x3FF
        if delta > 512:
            delta -= 1024
        print(f"  {i:7d} {t_ms:9.1f} {phases[i]:6d} {delta:+6d}")
        prev_phase = phases[i]
    # Always show last event
    if (n_phases - 1) % step_size != 0 and n_phases > 0:
        i = n_phases - 1
        t_ms = i * update_interval * 1000.
        delta = (phases[i] - prev_phase) & 0x3FF
        if delta > 512:
            delta -= 1024
        print(f"  {i:7d} {t_ms:9.1f} {phases[i]:6d} {delta:+6d}")

    # ---- Verify: no large phase jumps (would cause motor stall) ----
    max_jump = 0
    for i in range(1, n_phases):
        delta = (phases[i] - phases[i-1]) & 0x3FF
        if delta > 512:
            delta -= 1024
        if abs(delta) > max_jump:
            max_jump = abs(delta)

    # Limit: cruise velocity MSCNT/interval * safety factor.
    # Segment boundaries can have small jumps from quadratic fit error,
    # so allow a generous margin.  Anything over 50 MSCNT (5% of one
    # electrical cycle) in a single 0.1ms interval would indicate a bug.
    max_expected = cruise_v / step_dist * scale_factor / update_rate
    jump_limit = max(max_expected * 3, 50)
    print(f"\n  Max phase jump: {max_jump} MSCNT/interval"
          f"  (expected ~{max_expected:.1f}, limit {jump_limit:.0f})")

    if max_jump > jump_limit:
        errors.append(f"Phase jump {max_jump} exceeds safe limit {jump_limit:.0f}")

    ffi_lib.trapq_free(tq)

    if errors:
        print(f"\n  FAIL: {errors}")
        return False
    print(f"\n  PASS")
    return True


if __name__ == '__main__':
    print("Building C helper library...")
    ffi_main, ffi_lib = get_ffi()
    print("OK\n")

    results = []

    # Test 1: Full 7-phase profile (symmetric)
    results.append(verify_move(
        "Full 7-phase (symmetric accel/decel)",
        move_d=80., start_v=0., cruise_v=100., end_v=0.,
        accel=500., jerk_max=5000.))

    # Test 2: Triangular accel (short delta_v)
    results.append(verify_move(
        "Triangular accel (delta_v < 2*v_j)",
        move_d=50., start_v=0., cruise_v=20., end_v=0.,
        accel=500., jerk_max=5000.))

    # Test 3: High jerk (realistic printer values)
    results.append(verify_move(
        "High jerk (printer-like: accel=3000, jerk=50000)",
        move_d=100., start_v=0., cruise_v=100., end_v=0.,
        accel=3000., jerk_max=50000.))

    # Test 4: Asymmetric (different start/end velocities)
    results.append(verify_move(
        "Asymmetric (start_v=20, end_v=50)",
        move_d=100., start_v=20., cruise_v=100., end_v=50.,
        accel=3000., jerk_max=50000.))

    # Test 5: Junction velocity (mid-print move)
    results.append(verify_move(
        "Junction velocity (start_v=80, cruise_v=100, end_v=5)",
        move_d=50., start_v=80., cruise_v=100., end_v=5.,
        accel=3000., jerk_max=50000.))

    # Test 6: Very short move (might fail -> trapezoidal fallback)
    results.append(verify_move(
        "Very short move (2.83mm, high decel needed)",
        move_d=2.83, start_v=90., cruise_v=100., end_v=0.,
        accel=3000., jerk_max=50000.))

    # Test 7: No accel needed (start_v == cruise_v)
    results.append(verify_move(
        "No accel (start_v == cruise_v, decel only)",
        move_d=50., start_v=100., cruise_v=100., end_v=0.,
        accel=3000., jerk_max=50000.))

    # Test 8: No decel needed
    results.append(verify_move(
        "No decel (accel only, end_v == cruise_v)",
        move_d=50., start_v=0., cruise_v=100., end_v=100.,
        accel=3000., jerk_max=50000.))

    # ================================================================
    # Phase Stepping Pipeline Verification
    # ================================================================
    print(f"\n{'='*60}")
    print(f"  PHASE STEPPING PIPELINE VERIFICATION")
    print(f"{'='*60}")

    # Test 9: Full pipeline with S-curve move at 16 microsteps
    results.append(verify_phase_pipeline(
        "S-curve 100mm/s at 16 microsteps",
        move_d=10., cruise_v=100., accel=3000., jerk_max=50000.,
        microsteps=16))

    # Test 10: Full pipeline at 256 microsteps (scaling factor = 1)
    results.append(verify_phase_pipeline(
        "S-curve 100mm/s at 256 microsteps (no scaling)",
        move_d=10., cruise_v=100., accel=3000., jerk_max=50000.,
        microsteps=256))

    # Test 11: Slow move at 16 microsteps
    results.append(verify_phase_pipeline(
        "Slow move 10mm/s at 16 microsteps",
        move_d=5., cruise_v=10., accel=500., jerk_max=5000.,
        microsteps=16))

    # Test 12: Fast move at 32 microsteps
    results.append(verify_phase_pipeline(
        "Fast move 300mm/s at 32 microsteps",
        move_d=50., cruise_v=300., accel=5000., jerk_max=100000.,
        microsteps=32))

    # Summary
    passed = sum(1 for r in results if r)
    total = len(results)
    print(f"\n{'='*60}")
    print(f"  Results: {passed}/{total} passed")
    print(f"{'='*60}")
    sys.exit(0 if passed == total else 1)
