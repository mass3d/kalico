#!/usr/bin/env python3
"""
Standalone test for S-curve motion planning and phase compression.
Runs entirely on the host -- no MCU or hardware needed.

Tests:
  1. trapq_append_scurve() position continuity and smoothness
  2. move_get_distance() backward compatibility (sixth_jerk=0)
  3. Phase compressor round-trip accuracy
  4. S-curve vs trapezoidal profile comparison
"""

import sys, os, math

# Add klippy to path so we can import chelper
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'klippy'))
import chelper

def get_ffi():
    return chelper.get_ffi()

def extract_moves_sorted(ffi_main, ffi_lib, tq, max_moves=16,
                         start_time=0.0, end_time=20.0):
    """Extract pull_moves from trapq and return as a list sorted by print_time.
    trapq_extract_old returns moves in reverse order (newest first)."""
    pm = ffi_main.new('struct pull_move[%d]' % max_moves)
    count = ffi_lib.trapq_extract_old(tq, pm, max_moves, start_time, end_time)
    # Copy to python dicts and sort by print_time
    moves = []
    for i in range(count):
        moves.append({
            'print_time': pm[i].print_time,
            'move_t': pm[i].move_t,
            'start_v': pm[i].start_v,
            'accel': pm[i].accel,
            'jerk': pm[i].jerk,
            'start_x': pm[i].start_x,
            'start_y': pm[i].start_y,
            'start_z': pm[i].start_z,
            'x_r': pm[i].x_r,
            'y_r': pm[i].y_r,
            'z_r': pm[i].z_r,
        })
    moves.sort(key=lambda m: m['print_time'])
    return moves

def test_trapezoidal_backward_compat():
    """Verify that trapq_append still works identically (no jerk)."""
    print("Test 1: Trapezoidal backward compatibility...")
    ffi_main, ffi_lib = get_ffi()
    tq = ffi_lib.trapq_alloc()

    # Simple trapezoidal move: accel 0.5s, cruise 1.0s, decel 0.5s
    # start_v=0, cruise_v=100mm/s, accel=200mm/s^2
    ffi_lib.trapq_append(tq, 0.0,
                         0.5, 1.0, 0.5,       # accel_t, cruise_t, decel_t
                         0.0, 0.0, 0.0,        # start_pos
                         1.0, 0.0, 0.0,        # axes_r (X only)
                         0.0, 100.0, 200.0)    # start_v, cruise_v, accel

    # Extract moves and verify
    moves = extract_moves_sorted(ffi_main, ffi_lib, tq)
    assert len(moves) == 3, f"Expected 3 sub-moves, got {len(moves)}"

    # Verify jerk is zero for all sub-moves
    for i, m in enumerate(moves):
        assert m['jerk'] == 0.0, f"Move {i}: jerk should be 0, got {m['jerk']}"

    # Verify total distance: accel_d + cruise_d + decel_d
    # accel_d = 0.5 * 200 * 0.5^2 = 25mm
    # cruise_d = 100 * 1.0 = 100mm
    # decel_d = 100*0.5 - 0.5*200*0.5^2 = 50-25 = 25mm
    # total = 150mm
    total_d = 0.0
    for m in moves:
        t = m['move_t']
        v = m['start_v']
        a = m['accel']
        j = m['jerk']
        d = (v + (a/2.0 + j/6.0 * t) * t) * t
        total_d += d
    expected = 150.0
    assert abs(total_d - expected) < 0.001, \
        f"Total distance {total_d:.4f} != expected {expected}"

    ffi_lib.trapq_free(tq)
    print(f"  PASS: 3 sub-moves, total distance = {total_d:.4f}mm, jerk = 0")

def test_scurve_basic():
    """Test trapq_append_scurve with a simple S-curve profile."""
    print("Test 2: S-curve basic profile...")
    ffi_main, ffi_lib = get_ffi()
    tq = ffi_lib.trapq_alloc()

    # Parameters for a full 7-phase S-curve move
    start_v = 0.0       # mm/s
    cruise_v = 100.0     # mm/s
    accel_max = 500.0    # mm/s^2
    jerk_max = 5000.0    # mm/s^3

    # Compute phase durations
    tj = accel_max / jerk_max  # 0.1s - jerk phase duration
    v_j = 0.5 * jerk_max * tj * tj  # 25 mm/s - velocity from one jerk phase

    delta_v_accel = cruise_v - start_v  # 100 mm/s
    # Full trapezoidal accel: delta_v = 2*v_j + a_max*ta
    ta = (delta_v_accel - 2.0 * v_j) / accel_max  # (100 - 50) / 500 = 0.1s

    # Same for decel (symmetric)
    tj2 = tj
    td = ta

    # Cruise: give it 0.5s
    tc = 0.5

    ffi_lib.trapq_append_scurve(tq, 0.0,
                                0.0, 0.0, 0.0,       # start_pos
                                1.0, 0.0, 0.0,       # axes_r
                                start_v, jerk_max,
                                accel_max, cruise_v,
                                tj, ta, tc,
                                tj2, td)

    # Extract and count sub-moves
    moves = extract_moves_sorted(ffi_main, ffi_lib, tq)
    print(f"  Sub-moves: {len(moves)}")

    # Print each phase
    total_time = 0.0
    total_dist = 0.0
    for i, m in enumerate(moves):
        t = m['move_t']
        v = m['start_v']
        a = m['accel']
        j = m['jerk']
        d = (v + (a/2.0 + j/6.0 * t) * t) * t
        end_v = v + a * t + 0.5 * j * t * t
        total_dist += d
        total_time += t
        print(f"    Phase {i+1}: t={t:.4f}s, v0={v:.2f}, a={a:.1f}, "
              f"j={j:.1f}, d={d:.4f}mm, v_end={end_v:.2f}")

    # Verify expected total time
    expected_time = 2*tj + ta + tc + 2*tj2 + td
    assert abs(total_time - expected_time) < 0.0001, \
        f"Total time {total_time:.4f} != expected {expected_time:.4f}"

    print(f"  Total time: {total_time:.4f}s, distance: {total_dist:.4f}mm")

    ffi_lib.trapq_free(tq)
    print("  PASS")

def test_scurve_position_continuity():
    """Verify position and velocity are continuous at phase boundaries."""
    print("Test 3: S-curve position/velocity continuity...")
    ffi_main, ffi_lib = get_ffi()
    tq = ffi_lib.trapq_alloc()

    start_v = 10.0
    cruise_v = 80.0
    accel_max = 400.0
    jerk_max = 4000.0

    tj = accel_max / jerk_max  # 0.1s
    v_j = 0.5 * jerk_max * tj * tj  # 20 mm/s
    delta_v = cruise_v - start_v  # 70 mm/s
    ta = (delta_v - 2.0 * v_j) / accel_max  # (70-40)/400 = 0.075s
    tc = 0.3
    tj2 = tj
    td = ta

    ffi_lib.trapq_append_scurve(tq, 0.0,
                                0.0, 0.0, 0.0,
                                1.0, 0.0, 0.0,
                                start_v, jerk_max,
                                accel_max, cruise_v,
                                tj, ta, tc, tj2, td)

    moves = extract_moves_sorted(ffi_main, ffi_lib, tq)

    # Check velocity continuity at each boundary
    max_v_error = 0.0
    max_p_error = 0.0
    cumulative_pos = moves[0]['start_x']  # should be 0.0
    for i, m in enumerate(moves):
        t = m['move_t']
        v0 = m['start_v']
        a = m['accel']
        j = m['jerk']

        # Position at end of this phase
        end_pos = (v0 + (a/2.0 + j/6.0 * t) * t) * t
        cumulative_pos += end_pos

        # Velocity at end of this phase
        end_v = v0 + a * t + 0.5 * j * t * t

        # Check against start of next phase
        if i + 1 < len(moves):
            next_v0 = moves[i+1]['start_v']
            next_px = moves[i+1]['start_x']
            v_err = abs(end_v - next_v0)
            p_err = abs(cumulative_pos - next_px)
            max_v_error = max(max_v_error, v_err)
            max_p_error = max(max_p_error, p_err)

    print(f"  Max velocity discontinuity: {max_v_error:.6e} mm/s")
    print(f"  Max position discontinuity: {max_p_error:.6e} mm")
    assert max_v_error < 0.01, f"Velocity discontinuity too large: {max_v_error}"
    assert max_p_error < 0.001, f"Position discontinuity too large: {max_p_error}"

    ffi_lib.trapq_free(tq)
    print("  PASS")

def test_scurve_triangular():
    """Test S-curve with short move that never reaches max accel."""
    print("Test 4: S-curve triangular accel profile...")
    ffi_main, ffi_lib = get_ffi()
    tq = ffi_lib.trapq_alloc()

    start_v = 0.0
    cruise_v = 20.0
    accel_max = 500.0
    jerk_max = 5000.0

    tj = accel_max / jerk_max  # 0.1s
    v_j = 0.5 * jerk_max * tj * tj  # 25 mm/s

    # delta_v = 20 < 2*v_j = 50, so triangular
    delta_v = cruise_v - start_v
    tj_tri = math.sqrt(delta_v / jerk_max)  # sqrt(20/5000) = 0.0632s
    ta = 0.0  # no constant accel phase

    tc = 0.2  # some cruise
    tj2 = tj_tri
    td = 0.0

    ffi_lib.trapq_append_scurve(tq, 0.0,
                                0.0, 0.0, 0.0,
                                1.0, 0.0, 0.0,
                                start_v, jerk_max,
                                accel_max, cruise_v,
                                tj_tri, ta, tc, tj2, td)

    moves = extract_moves_sorted(ffi_main, ffi_lib, tq)

    # With ta=0 and td=0, phases 2 and 6 are skipped
    # Expect: phase1(tj), phase3(tj), cruise, phase5(tj), phase7(tj) = 5 moves
    # But phases with zero duration are skipped in scurve_add_phase
    print(f"  Sub-moves: {len(moves)} (triangular, no const-accel phases)")

    total_time = sum(m['move_t'] for m in moves)
    expected = 2*tj_tri + tc + 2*tj_tri
    print(f"  Total time: {total_time:.4f}s (expected {expected:.4f}s)")
    assert abs(total_time - expected) < 0.001

    ffi_lib.trapq_free(tq)
    print("  PASS")

def test_phase_compressor():
    """Test phase compression round-trip accuracy."""
    print("Test 5: Phase compressor round-trip...")
    ffi_main, ffi_lib = get_ffi()

    pc = ffi_lib.phase_compressor_alloc()
    ffi_lib.phase_compressor_set_max_error(pc, 0.25)

    # Generate a known position curve: parabolic (constant accel)
    # pos(t) = 100*t + 500*t^2  (v0=100, a=1000)
    n = 200
    dt = 0.0001  # 10kHz sample rate
    positions = ffi_main.new('double[]', n)
    for i in range(n):
        t = i * dt
        positions[i] = 100.0 * t + 500.0 * t * t

    segments = ffi_main.new('struct phase_compressed_move[]', 64)
    num_seg = ffi_lib.phase_compressor_compress(pc, positions, n, segments, 64)

    print(f"  {n} samples compressed to {num_seg} segment(s)")

    # Reconstruct and check error
    max_error = 0.0
    sample_idx = 0
    for s in range(num_seg):
        seg = segments[s]
        for i in range(seg.count):
            reconstructed = seg.start_position + seg.velocity * i \
                            + 0.5 * seg.acceleration * i * i
            error = abs(positions[sample_idx] - reconstructed)
            max_error = max(max_error, error)
            sample_idx += 1

    print(f"  Max reconstruction error: {max_error:.6f} microsteps")
    assert max_error <= 0.25, f"Error {max_error} exceeds threshold 0.25"
    # For a pure quadratic, should compress to 1 segment with near-zero error
    print(f"  Compression ratio: {n}/{num_seg} = {n/max(num_seg,1):.0f}x")

    ffi_lib.free(pc)
    print("  PASS")

def test_phase_compressor_sine():
    """Test compressor with a sine wave (harder to fit)."""
    print("Test 6: Phase compressor with sine wave...")
    ffi_main, ffi_lib = get_ffi()

    pc = ffi_lib.phase_compressor_alloc()
    ffi_lib.phase_compressor_set_max_error(pc, 0.25)

    # Sine wave: pos = 100 * sin(2*pi*50*t)  (50Hz, amplitude 100 microsteps)
    n = 1000
    dt = 0.0001
    positions = ffi_main.new('double[]', n)
    for i in range(n):
        t = i * dt
        positions[i] = 100.0 * math.sin(2.0 * math.pi * 50.0 * t)

    segments = ffi_main.new('struct phase_compressed_move[]', 512)
    num_seg = ffi_lib.phase_compressor_compress(pc, positions, n, segments, 512)

    # Reconstruct and check
    max_error = 0.0
    sample_idx = 0
    for s in range(num_seg):
        seg = segments[s]
        for i in range(seg.count):
            reconstructed = seg.start_position + seg.velocity * i \
                            + 0.5 * seg.acceleration * i * i
            error = abs(positions[sample_idx] - reconstructed)
            max_error = max(max_error, error)
            sample_idx += 1

    print(f"  {n} samples -> {num_seg} segments "
          f"(compression {n/max(num_seg,1):.1f}x)")
    print(f"  Max reconstruction error: {max_error:.6f} microsteps")
    assert max_error <= 0.25 + 0.001, f"Error {max_error} exceeds threshold"

    ffi_lib.free(pc)
    print("  PASS")


def test_phase_generator_move_start_trimming():
    """Verify mixed standstill flushes trim to the first moving sample."""
    print("Test 7: Phase generator move-start trimming...")
    ffi_main, ffi_lib = get_ffi()

    positions = ffi_main.new('double[]', [0.0, 0.0, 0.2, 0.8, 1.2, 2.0])
    move_start = ffi_lib.phase_generator_find_move_start(
        positions, 6, 1.0)
    assert move_start == 4, \
        f"Expected first moving sample at 4, got {move_start}"

    standstill = ffi_main.new('double[]', [5.0, 5.0, 5.0, 5.0])
    move_start = ffi_lib.phase_generator_find_move_start(
        standstill, 4, 1.0)
    assert move_start == 4, \
        f"Expected pure standstill to return 4, got {move_start}"

    print("  PASS")


def test_phase_generator_extract_rejects_nonfinite():
    """Verify invalid trapq samples are rejected before compression."""
    print("Test 8: Phase generator non-finite sample rejection...")
    ffi_main, ffi_lib = get_ffi()

    samples = ffi_main.new('struct phase_sample[]', 3)
    positions = ffi_main.new('double[]', 3)

    samples[0].position = 0.0
    samples[1].position = float('inf')
    samples[2].position = 1.0
    ret = ffi_lib.phase_generator_extract_positions(samples, 3, positions)
    assert ret == -1, f"Expected inf sample rejection, got {ret}"

    samples[1].position = float('nan')
    ret = ffi_lib.phase_generator_extract_positions(samples, 3, positions)
    assert ret == -1, f"Expected NaN sample rejection, got {ret}"

    samples[1].position = 0.5
    ret = ffi_lib.phase_generator_extract_positions(samples, 3, positions)
    assert ret == 3, f"Expected valid samples to pass, got {ret}"

    print("  PASS")


def test_phase_generator_holds_last_position_on_inactive_moves():
    """Verify inactive moves hold the last phase instead of resampling."""
    print("Test 9: Phase generator inactive-move hold...")
    ffi_main, ffi_lib = get_ffi()

    tq = ffi_lib.trapq_alloc()
    ffi_lib.trapq_append(
        tq, 2.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        20.0, 20.0, 0.0)

    sk = ffi_lib.cartesian_stepper_alloc(b'x')
    ffi_lib.itersolve_set_trapq(sk, tq)

    pg = ffi_main.gc(ffi_lib.phase_generator_alloc(), ffi_lib.free)
    ffi_lib.phase_generator_config(pg, sk, 1.0, 0.1)
    ffi_lib.phase_generator_set_direction(pg, 1.0)
    ffi_lib.phase_generator_set_offset(pg, 0.0)
    ffi_lib.phase_generator_set_last_position(pg, 123.0)
    ffi_lib.phase_generator_set_time(pg, 2.0)

    samples = ffi_main.new('struct phase_sample[]', 16)
    num_samples = ffi_lib.phase_generator_generate(pg, 2.5, samples, 16)
    assert num_samples > 0, "Expected idle-hold samples for inactive move"
    for i in range(num_samples):
        assert samples[i].position == 123.0, \
            f"Expected held phase 123.0, got {samples[i].position}"

    ffi_lib.trapq_free(tq)
    print("  PASS")


# =================================================================
# Phase Stepping Pipeline Tests
#
# These test the full data flow: trapq -> phase_generator -> compressor
# -> MCU fixed-point, verifying that positions are correctly scaled
# to MSCNT units for different microstep configurations.
# =================================================================

def simulate_mcu_phase(position_fixed):
    """Simulate the MCU ISR phase extraction: (position >> 16) & 0x3FF.
    position_fixed is a signed 32-bit integer in 16.16 fixed-point."""
    # Python ints are arbitrary precision, so simulate int32 arithmetic
    pos = position_fixed & 0xFFFFFFFF  # mask to 32 bits unsigned
    if pos >= 0x80000000:
        pos -= 0x100000000  # convert to signed
    integer_part = pos >> 16  # arithmetic right shift (Python preserves sign)
    return integer_part & 0x3FF

def phase_pipeline(ffi_main, ffi_lib, move_mm, speed_mm_s, step_dist,
                   microsteps, direction, phase_offset, start_time=1.0):
    """Run the full phase stepping pipeline for a constant-velocity move.

    Returns (samples_out, mcu_moves, num_mcu) where:
    - samples_out: raw phase_sample array from the generator
    - mcu_moves: phase_mcu_move array (16.16 fixed-point for MCU)
    - num_mcu: number of MCU move segments
    """
    update_rate = 10000.0
    update_interval = 1.0 / update_rate
    max_samples = 8192
    max_segments = 1024

    # Scale step_dist for MSCNT units (the fix under test)
    phase_step_dist = step_dist * microsteps / 256.0

    # Set up trapq with a constant-velocity move along X axis
    tq = ffi_lib.trapq_alloc()
    duration = abs(move_mm) / speed_mm_s
    move_dir = 1.0 if move_mm >= 0 else -1.0
    ffi_lib.trapq_append(tq, start_time,
                         0.0, duration, 0.0,      # no accel, pure cruise
                         0.0, 0.0, 0.0,            # start at origin
                         move_dir, 0.0, 0.0,       # X axis only
                         speed_mm_s, speed_mm_s, 0.0)

    # Set up cartesian X stepper kinematics and attach to trapq
    sk = ffi_lib.cartesian_stepper_alloc(b'x')
    ffi_lib.itersolve_set_trapq(sk, tq)

    # Configure phase generator
    pg = ffi_main.gc(ffi_lib.phase_generator_alloc(), ffi_lib.free)
    ffi_lib.phase_generator_config(pg, sk, phase_step_dist, update_interval)
    ffi_lib.phase_generator_set_direction(pg, direction)
    ffi_lib.phase_generator_set_offset(pg, phase_offset)
    ffi_lib.phase_generator_set_time(pg, start_time)

    # Generate samples
    samples = ffi_main.new('struct phase_sample[]', max_samples)
    flush_time = start_time + duration + 0.001
    num_samples = ffi_lib.phase_generator_generate(
        pg, flush_time, samples, max_samples)
    assert num_samples > 0, "Phase generator produced 0 samples"

    # Extract positions
    positions = ffi_main.new('double[]', max_samples)
    ret = ffi_lib.phase_generator_extract_positions(
        samples, num_samples, positions)
    assert ret >= 0, "NaN in phase generator output"

    # Compress
    pc = ffi_main.gc(ffi_lib.phase_compressor_alloc(), ffi_lib.free)
    max_error = 0.25 * (256.0 / microsteps)  # scaled to MSCNT units
    ffi_lib.phase_compressor_set_max_error(pc, max_error)
    segments = ffi_main.new('struct phase_compressed_move[]', max_segments)
    num_seg = ffi_lib.phase_compressor_compress(
        pc, positions, num_samples, segments, max_segments)

    # Convert to fixed-point MCU format
    mcu_moves = ffi_main.new('struct phase_mcu_move[]', max_segments)
    num_mcu = ffi_lib.phase_compressor_to_fixed(
        segments, num_seg, mcu_moves, max_segments)

    ffi_lib.trapq_free(tq)
    # Note: sk is allocated by the C library but not gc'd here;
    # it's a small leak in a test context, acceptable.
    return samples, num_samples, mcu_moves, num_mcu


def test_phase_pipeline_scaling():
    """Test that the full pipeline produces correct MSCNT-rate phase changes
    for different microstep configurations.

    This is the test that catches the 16x scaling bug: if step_dist is not
    adjusted for the microstep ratio, the MCU velocity will be 16x too slow
    at 16 microsteps (or Nx for N microsteps)."""
    print("Test 7: Phase pipeline microstep scaling...")
    ffi_main, ffi_lib = get_ffi()

    # Typical 200-step motor, GT2 20T pulley (40mm/rev)
    rotation_distance = 40.0  # mm
    full_steps_per_rev = 200

    # Test several microstep configs
    for microsteps in [16, 32, 64, 128, 256]:
        step_dist = rotation_distance / (full_steps_per_rev * microsteps)
        speed = 100.0  # mm/s
        move = 10.0    # mm

        _, num_samples, mcu_moves, num_mcu = phase_pipeline(
            ffi_main, ffi_lib, move_mm=move, speed_mm_s=speed,
            step_dist=step_dist, microsteps=microsteps,
            direction=1.0, phase_offset=0.0)

        assert num_mcu > 0, f"No MCU moves at microsteps={microsteps}"

        # Sum total position change across all MCU segments.
        # Each segment: pos changes by velocity*count + 0.5*accel*count^2
        # (in 16.16 fixed-point)
        total_delta = 0
        total_count = 0
        for i in range(num_mcu):
            m = mcu_moves[i]
            c = m.count
            # Exact summation of the quadratic: sum_{k=0}^{c-1} (vel*k + accel/2*k^2)
            # But simpler: end_pos - start_pos for the segment
            end_vel = m.velocity + m.acceleration * (c - 1)
            end_pos = m.start_position + m.velocity * c + (
                m.acceleration * c * (c - 1)) // 2
            # Just use the velocity of the longest constant-velocity segment
            total_count += c

        # For constant-velocity move, the dominant segment should have accel≈0.
        # Find it (largest count with near-zero accel).
        cruise_seg = None
        for i in range(num_mcu):
            m = mcu_moves[i]
            if m.count > 100 and abs(m.acceleration) <= 1:
                cruise_seg = m
                break
        assert cruise_seg is not None, \
            f"No cruise segment found at microsteps={microsteps}"

        # Expected velocity in MSCNT per interval at 10kHz:
        # speed_mm_s / step_dist = klipper steps/sec
        # * (256/microsteps) = MSCNT/sec
        # / 10000 = MSCNT per interval
        # * 65536 = 16.16 fixed-point
        expected_mscnt_per_sec = speed / step_dist * (256.0 / microsteps)
        expected_vel_fixed = expected_mscnt_per_sec / 10000.0 * 65536.0

        actual_vel = cruise_seg.velocity
        ratio = actual_vel / expected_vel_fixed if expected_vel_fixed != 0 else 0

        print(f"  microsteps={microsteps:3d}: vel_fixed={actual_vel:10d}"
              f"  expected={expected_vel_fixed:12.1f}  ratio={ratio:.4f}"
              f"  samples={num_samples} segs={num_mcu}")

        # The velocity should match within 1% (compression introduces small error)
        assert abs(ratio - 1.0) < 0.01, \
            f"Velocity ratio {ratio:.4f} at microsteps={microsteps}" \
            f" (expected ~1.0, got vel={actual_vel}" \
            f" vs expected={expected_vel_fixed:.1f})"

    print("  PASS")


def test_phase_pipeline_direction():
    """Test that direction inversion correctly reverses the phase velocity."""
    print("Test 8: Phase pipeline direction inversion...")
    ffi_main, ffi_lib = get_ffi()

    microsteps = 16
    step_dist = 40.0 / (200 * microsteps)
    speed = 50.0
    move = 5.0

    # Forward direction
    _, _, mcu_fwd, n_fwd = phase_pipeline(
        ffi_main, ffi_lib, move_mm=move, speed_mm_s=speed,
        step_dist=step_dist, microsteps=microsteps,
        direction=1.0, phase_offset=0.0)

    # Reverse direction (simulates inverted dir_pin)
    _, _, mcu_rev, n_rev = phase_pipeline(
        ffi_main, ffi_lib, move_mm=move, speed_mm_s=speed,
        step_dist=step_dist, microsteps=microsteps,
        direction=-1.0, phase_offset=0.0)

    # Find cruise segments
    fwd_cruise = None
    for i in range(n_fwd):
        if mcu_fwd[i].count > 100 and abs(mcu_fwd[i].acceleration) <= 1:
            fwd_cruise = mcu_fwd[i]
            break
    rev_cruise = None
    for i in range(n_rev):
        if mcu_rev[i].count > 100 and abs(mcu_rev[i].acceleration) <= 1:
            rev_cruise = mcu_rev[i]
            break

    assert fwd_cruise is not None, "No forward cruise segment"
    assert rev_cruise is not None, "No reverse cruise segment"

    # Velocities should be equal magnitude, opposite sign
    sum_vel = fwd_cruise.velocity + rev_cruise.velocity
    diff_vel = abs(fwd_cruise.velocity) - abs(rev_cruise.velocity)

    print(f"  Forward vel:  {fwd_cruise.velocity}")
    print(f"  Reverse vel:  {rev_cruise.velocity}")
    print(f"  Sum (expect~0): {sum_vel}")
    print(f"  Magnitude diff: {diff_vel}")

    assert abs(sum_vel) < 100, \
        f"Direction inversion failed: fwd={fwd_cruise.velocity}" \
        f" rev={rev_cruise.velocity} sum={sum_vel}"
    print("  PASS")


def test_phase_pipeline_offset():
    """Test that phase offset correctly shifts the starting phase."""
    print("Test 9: Phase pipeline offset calibration...")
    ffi_main, ffi_lib = get_ffi()

    microsteps = 16
    step_dist = 40.0 / (200 * microsteps)

    # At standstill: a very short "move" at zero speed
    # Position = 0mm, so stepper_pos_mscnt = 0.  With offset=500,
    # the MCU starting phase should be 500.
    for test_offset in [0.0, 100.0, 500.0, 1000.0]:
        # Use a tiny constant-velocity move so we get samples near standstill
        _, num_samples, mcu_moves, num_mcu = phase_pipeline(
            ffi_main, ffi_lib, move_mm=0.001, speed_mm_s=0.1,
            step_dist=step_dist, microsteps=microsteps,
            direction=1.0, phase_offset=test_offset)

        assert num_mcu > 0, f"No MCU moves with offset={test_offset}"

        # The first MCU segment's start_position encodes the initial phase.
        # Phase = (start_position >> 16) & 0x3FF
        start_phase = simulate_mcu_phase(mcu_moves[0].start_position)
        expected_phase = int(test_offset) & 0x3FF

        print(f"  offset={test_offset:6.0f}: start_pos={mcu_moves[0].start_position:12d}"
              f"  phase={start_phase:4d}  expected={expected_phase:4d}")

        # Allow +-1 for rounding in fmod/fixed-point conversion
        phase_err = min(abs(start_phase - expected_phase),
                        1024 - abs(start_phase - expected_phase))
        assert phase_err <= 1, \
            f"Phase offset error: got {start_phase}, expected {expected_phase}" \
            f" (offset={test_offset})"

    print("  PASS")


def test_phase_pipeline_mcu_phase_continuity():
    """Test that the MCU-side phase values form a smooth, continuous sequence
    across segments during a constant-velocity move.  Simulates the MCU ISR
    position accumulation and phase extraction."""
    print("Test 10: MCU phase continuity across segments...")
    ffi_main, ffi_lib = get_ffi()

    microsteps = 16
    step_dist = 40.0 / (200 * microsteps)
    speed = 80.0  # mm/s -- enough to cross multiple electrical cycles

    _, _, mcu_moves, num_mcu = phase_pipeline(
        ffi_main, ffi_lib, move_mm=5.0, speed_mm_s=speed,
        step_dist=step_dist, microsteps=microsteps,
        direction=1.0, phase_offset=100.0)

    # Walk through MCU segments, simulating the ISR position accumulation
    phases = []
    for i in range(num_mcu):
        m = mcu_moves[i]
        pos = m.start_position
        vel = m.velocity
        for k in range(m.count):
            pos += vel
            vel += m.acceleration
            phase = simulate_mcu_phase(pos)
            phases.append(phase)

    # Check phase changes between consecutive ISR events.
    # For a constant-velocity move, the phase delta per interval should be
    # approximately constant and non-zero.
    max_jump = 0
    nonzero_deltas = 0
    for i in range(1, len(phases)):
        delta = (phases[i] - phases[i-1]) & 0x3FF
        # Normalize to signed: if delta > 512, it wrapped backward
        if delta > 512:
            delta -= 1024
        abs_delta = abs(delta)
        if abs_delta > max_jump:
            max_jump = abs_delta
        if abs_delta > 0:
            nonzero_deltas += 1

    # At 80mm/s with 16 microsteps, step_dist=0.0125mm:
    # MSCNT rate = 80/0.0125 * 16 = 102400 MSCNT/sec
    # Per 10kHz interval: 10.24 MSCNT per interval
    # So phase should change by ~10 per interval -- definitely nonzero
    expected_delta = speed / step_dist * (256.0 / microsteps) / 10000.0

    print(f"  Total ISR events: {len(phases)}")
    print(f"  Phase changes (nonzero): {nonzero_deltas}/{len(phases)-1}")
    print(f"  Max phase jump per interval: {max_jump}")
    print(f"  Expected phase delta/interval: {expected_delta:.1f}")

    # Phase should change nearly every interval
    assert nonzero_deltas > 0.8 * (len(phases) - 1), \
        f"Phase barely changes: only {nonzero_deltas}/{len(phases)-1} intervals"

    # Max jump should be reasonable (not huge wraps from bad data)
    assert max_jump < expected_delta * 3, \
        f"Phase jump {max_jump} too large (expected ~{expected_delta:.0f})"

    # Verify total phase rotation matches expected distance
    # Total MSCNT change = move_mm / step_dist * (256/microsteps)
    total_phase_change = 0
    for i in range(1, len(phases)):
        delta = (phases[i] - phases[i-1]) & 0x3FF
        if delta > 512:
            delta -= 1024
        total_phase_change += delta

    expected_total = 5.0 / step_dist * (256.0 / microsteps)
    ratio = total_phase_change / expected_total if expected_total else 0

    print(f"  Total phase rotation: {total_phase_change:.0f} MSCNT")
    print(f"  Expected: {expected_total:.0f} MSCNT")
    print(f"  Ratio: {ratio:.4f}")

    assert abs(ratio - 1.0) < 0.02, \
        f"Total phase rotation off by {abs(ratio-1)*100:.1f}%" \
        f" (got {total_phase_change:.0f}, expected {expected_total:.0f})"

    print("  PASS")

def test_print_scurve_profile():
    """Print a detailed S-curve profile for visual inspection."""
    print("\nS-Curve Profile Visualization (text):")
    print("=" * 60)
    ffi_main, ffi_lib = get_ffi()
    tq = ffi_lib.trapq_alloc()

    start_v = 0.0
    cruise_v = 100.0
    accel_max = 500.0
    jerk_max = 5000.0

    tj = accel_max / jerk_max
    v_j = 0.5 * jerk_max * tj * tj
    ta = (cruise_v - start_v - 2.0 * v_j) / accel_max
    tc = 0.3
    tj2 = tj
    td = ta

    ffi_lib.trapq_append_scurve(tq, 0.0,
                                0.0, 0.0, 0.0,
                                1.0, 0.0, 0.0,
                                start_v, jerk_max,
                                accel_max, cruise_v,
                                tj, ta, tc, tj2, td)

    moves = extract_moves_sorted(ffi_main, ffi_lib, tq)

    # Sample position/velocity/acceleration at fine intervals
    print(f"{'Time':>8s} {'Pos':>10s} {'Vel':>10s} {'Accel':>10s} {'Phase':>8s}")
    print("-" * 50)

    abs_time = 0.0
    for i, mv in enumerate(moves):
        phase_t = mv['move_t']
        v0 = mv['start_v']
        a = mv['accel']
        j = mv['jerk']
        p0 = mv['start_x']

        steps = max(1, int(phase_t / 0.01))
        for s in range(steps + 1):
            t = s * phase_t / steps
            pos = p0 + (v0 + (a/2.0 + j/6.0 * t) * t) * t
            vel = v0 + a * t + 0.5 * j * t * t
            acc = a + j * t
            if s % max(1, steps // 4) == 0 or s == steps:
                print(f"{abs_time + t:8.4f} {pos:10.3f} {vel:10.3f} "
                      f"{acc:10.1f} {i+1:>8d}")
        abs_time += phase_t

    ffi_lib.trapq_free(tq)
    print("=" * 60)

if __name__ == '__main__':
    print("Building C helper library...")
    try:
        ffi_main, ffi_lib = get_ffi()
    except Exception as e:
        print(f"ERROR: Failed to build C library: {e}")
        print("Make sure gcc is installed.")
        sys.exit(1)
    print("C library built successfully.\n")

    tests = [
        test_trapezoidal_backward_compat,
        test_scurve_basic,
        test_scurve_position_continuity,
        test_scurve_triangular,
        test_phase_compressor,
        test_phase_compressor_sine,
        test_phase_generator_move_start_trimming,
        test_phase_generator_extract_rejects_nonfinite,
        test_phase_generator_holds_last_position_on_inactive_moves,
        test_phase_pipeline_scaling,
        test_phase_pipeline_direction,
        test_phase_pipeline_offset,
        test_phase_pipeline_mcu_phase_continuity,
    ]

    passed = 0
    failed = 0
    for test in tests:
        try:
            test()
            passed += 1
        except Exception as e:
            print(f"  FAIL: {e}")
            import traceback
            traceback.print_exc()
            failed += 1
        print()

    # Print profile visualization
    try:
        test_print_scurve_profile()
    except Exception as e:
        print(f"Profile visualization failed: {e}")

    print(f"\nResults: {passed} passed, {failed} failed out of {len(tests)}")
    sys.exit(1 if failed else 0)
