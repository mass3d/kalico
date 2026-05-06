"""Verify phase_compressor_compress_anchored produces continuous segment
chains: segment K+1's start_position equals MCU's last-emitted phase from
segment K, bit-exactly.

This is the construction-level guarantee that makes the resume-from-idle
click impossible: every emitted XDIRECT phase is continuous from the
previous one regardless of float drift in the input position stream.
"""

import math
import random
import sys

sys.path.insert(0, "/home/lpearl/kalico/klippy")
import chelper


# Mirror C int32 wrap.
INT32_MASK = 0xFFFFFFFF


def to_int32(x):
    x &= INT32_MASK
    if x & 0x80000000:
        x -= 0x100000000
    return x


def mirror_advance(start, vel, accel, count):
    """Closed-form position after `count` MCU tick-advances, matching C
    mirror_advance and the MCU loop.  This is what load_next overwrites
    ps->position with — i.e. the value the next segment's start_position
    must equal for a continuous chain."""
    if count == 0:
        return to_int32(start)
    n_choose_2 = (count * (count - 1)) // 2  # N*(N-1)/2
    raw = start + count * vel + n_choose_2 * accel
    return to_int32(raw)


def simulate_emit(segments):
    """Simulate the MCU's per-tick emission stream for a chain of segments.
    Returns the list of emitted int32 positions (one per tick)."""
    emitted = []
    for (start, vel, accel, count) in segments:
        position = to_int32(start)
        velocity = to_int32(vel)
        acceleration = to_int32(accel)
        for _ in range(count):
            emitted.append(position)
            position = to_int32(position + velocity)
            velocity = to_int32(velocity + acceleration)
    return emitted


def assert_chain_matches_positions(positions, segments, tol_microsteps):
    """Verify that the MCU's emission stream from `segments` matches the
    input `positions` (mod 1024 microsteps) within `tol_microsteps`.

    This catches the failure mode where chain-self-consistency holds but
    every segment is force-emit length=1 and the emission freezes — the
    chain looks valid but the MCU's phase output does not track the
    commanded motion."""
    emitted = simulate_emit(segments)
    assert len(emitted) == len(positions), (
        f"emit count {len(emitted)} != position count {len(positions)}")
    max_err = 0.0
    worst_idx = -1
    for i, (em_int, pos) in enumerate(zip(emitted, positions)):
        em_real = em_int / 65536.0
        diff = (em_real - pos + 512.0) % 1024.0 - 512.0
        if abs(diff) > abs(max_err):
            max_err = diff
            worst_idx = i
    assert abs(max_err) <= tol_microsteps, (
        f"emission diverges from input: max err {max_err:+.4f} microsteps at "
        f"index {worst_idx} (positions[{worst_idx}]={positions[worst_idx]:.4f}, "
        f"emitted={emitted[worst_idx] / 65536.0:.4f})")


def reduce_anchor(x):
    """Match C: anchor_reduced = anchor & 0x03FFFFFF (low 26 bits)."""
    return x & 0x03FFFFFF


def make_test_environment():
    ffi_main, ffi_lib = chelper.get_ffi()
    pc = ffi_main.gc(ffi_lib.phase_compressor_alloc(), ffi_lib.free)
    ffi_lib.phase_compressor_set_max_error(pc, 0.25)
    return ffi_main, ffi_lib, pc


def compress_anchored_raw(ffi_main, ffi_lib, pc, positions, anchor, max_out=256):
    """Wrap the C call and return (n_out, segment tuples).

    n_out is negative when the compressor filled max_out before consuming all
    samples.  That must be surfaced to the host so it can fault instead of
    silently queuing a partial prefix.
    """
    n = len(positions)
    pos_arr = ffi_main.new("double[]", n)
    for i, p in enumerate(positions):
        pos_arr[i] = p
    out_arr = ffi_main.new("struct phase_mcu_move[]", max_out)
    n_out = ffi_lib.phase_compressor_compress_anchored(
        pc, pos_arr, n, anchor, out_arr, max_out
    )
    if n_out < 0:
        return n_out, []
    return n_out, [
        (out_arr[i].start_position, out_arr[i].velocity,
         out_arr[i].acceleration, out_arr[i].count)
        for i in range(n_out)
    ]


def compress_anchored(ffi_main, ffi_lib, pc, positions, anchor):
    """Return segment tuples, asserting that compression completed."""
    n_out, segments = compress_anchored_raw(
        ffi_main, ffi_lib, pc, positions, anchor)
    assert n_out >= 0, f"compressor overflowed unexpectedly: n_out={n_out}"
    return segments


def verify_chain_continuity(segments, anchor):
    """Walk the segment list. First segment's start must == reduced anchor.
    Each subsequent segment's start must == mirror_advance of previous."""
    if not segments:
        return True, "empty segments OK"
    expected_anchor = reduce_anchor(anchor)
    for i, (start, vel, accel, count) in enumerate(segments):
        if start != expected_anchor:
            return False, (
                f"segment {i}: start={start:#010x} expected={expected_anchor:#010x} "
                f"diff={(start - expected_anchor) & INT32_MASK:#010x}"
            )
        next_anchor = mirror_advance(start, vel, accel, count)
        expected_anchor = reduce_anchor(next_anchor)
    return True, "continuous"


def main():
    ffi_main, ffi_lib, pc = make_test_environment()

    # Test 1: pure idle (all samples equal anchor)
    print("Test 1: pure idle hold")
    anchor_d = 100.5
    anchor = int(anchor_d * 65536)
    positions = [anchor_d] * 50
    segments = compress_anchored(ffi_main, ffi_lib, pc, positions, anchor)
    print(f"  -> {len(segments)} segments")
    for s in segments:
        print(f"    start={s[0]:#010x} vel={s[1]} accel={s[2]} count={s[3]}")
    ok, msg = verify_chain_continuity(segments, anchor)
    assert ok, f"FAIL: {msg}"
    # idle should be one segment with vel=0, accel=0
    assert len(segments) == 1, "idle should compress to one segment"
    assert segments[0][1] == 0, "idle vel must be 0"
    assert segments[0][2] == 0, "idle accel must be 0"
    assert segments[0][3] == 50, f"idle count must be 50, got {segments[0][3]}"
    print("  PASS")

    # Test 2: linear ramp (constant velocity)
    print("Test 2: linear ramp at 0.5 microsteps/tick")
    anchor_d = 200.0
    anchor = int(anchor_d * 65536)
    positions = [anchor_d + i * 0.5 for i in range(100)]
    segments = compress_anchored(ffi_main, ffi_lib, pc, positions, anchor)
    print(f"  -> {len(segments)} segments")
    ok, msg = verify_chain_continuity(segments, anchor)
    assert ok, f"FAIL: {msg}"
    print(f"    first seg: start={segments[0][0]:#010x} vel={segments[0][1]} "
          f"count={segments[0][3]}")
    print("  PASS")

    # Test 3: anchor INTENTIONALLY differs from positions[0] by 0.4 microsteps
    # (sub-max_error). The anchor should win — first segment starts at anchor.
    print("Test 3: anchor differs from positions[0] by 0.4 microsteps")
    anchor_d = 300.0
    anchor = int(anchor_d * 65536)
    positions = [anchor_d + 0.4]  # positions[0] differs from anchor by 0.4
    positions.extend([anchor_d + 0.4 + i * 0.5 for i in range(1, 50)])
    segments = compress_anchored(ffi_main, ffi_lib, pc, positions, anchor)
    ok, msg = verify_chain_continuity(segments, anchor)
    assert ok, f"FAIL: {msg}"
    assert segments[0][0] == reduce_anchor(anchor), (
        f"first segment must start at anchor, got {segments[0][0]:#010x}")
    print(f"  -> {len(segments)} segments, first start = {segments[0][0]:#010x} "
          f"= reduced anchor {reduce_anchor(anchor):#010x}")
    print("  PASS")

    # Test 4: anchor differs from positions[0] by MORE than max_error.
    # The fit should still produce a segment starting at anchor — continuity
    # beats fit error. The fit may use n=1 (force-emit single tick).
    print("Test 4: anchor differs from positions[0] by 5.0 microsteps")
    anchor_d = 400.0
    anchor = int(anchor_d * 65536)
    positions = [anchor_d + 5.0]  # way more than 0.25 max_error
    positions.extend([anchor_d + 5.0 + i * 0.5 for i in range(1, 50)])
    segments = compress_anchored(ffi_main, ffi_lib, pc, positions, anchor)
    ok, msg = verify_chain_continuity(segments, anchor)
    assert ok, f"FAIL: {msg}"
    assert segments[0][0] == reduce_anchor(anchor), (
        "first segment must start at anchor even when force-emitted")
    print(f"  -> {len(segments)} segments, first start = {segments[0][0]:#010x}, "
          f"first count = {segments[0][3]}")
    print("  PASS")

    # Test 5: random fuzz — produce arbitrary sample streams, verify chain
    # AND verify the simulated MCU emission tracks the input positions.
    # Without the emission check, force-emit-everything chains pass trivially.
    print("Test 5: 200 random fuzz cases (chain + emission)")
    random.seed(123)
    fail_count = 0
    for trial in range(200):
        n = random.randint(1, 200)
        anchor_d = random.uniform(-512.0, 512.0)
        anchor = int(anchor_d * 65536)
        # Generate samples with quadratic + noise
        b = random.uniform(-0.5, 0.5)
        c = random.uniform(-0.001, 0.001)
        positions = [anchor_d + b * i + c * i * i + random.uniform(-0.05, 0.05)
                     for i in range(n)]
        segments = compress_anchored(ffi_main, ffi_lib, pc, positions, anchor)
        ok, msg = verify_chain_continuity(segments, anchor)
        if not ok:
            fail_count += 1
            if fail_count <= 3:
                print(f"  trial {trial} FAIL: {msg}")
                print(f"    n={n} anchor={anchor:#010x} b={b:.4f} c={c:.6f}")
            continue
        # Sanity: total samples covered = n
        covered = sum(s[3] for s in segments)
        assert covered == n, f"trial {trial}: covered {covered} of {n}"
        # Tolerance: max_error (0.25) plus a small slack for force-emit
        # convergence at boundaries (~0.5 microstep across one tick).
        try:
            assert_chain_matches_positions(positions, segments, 1.0)
        except AssertionError as exc:
            fail_count += 1
            if fail_count <= 3:
                print(f"  trial {trial} EMISSION DIVERGENCE: {exc}")
                print(f"    n={n} anchor_d={anchor_d:.4f} b={b:.4f} c={c:.6f}")
    if fail_count:
        raise SystemExit(f"{fail_count} of 200 fuzz trials failed")
    print(f"  All 200 fuzz trials passed")
    print("  PASS")

    # Test 6: high-accel stress (verify int32 overflow chaining still bit-exact)
    print("Test 6: high-acceleration overflow chaining")
    anchor_d = 0.0
    anchor = int(anchor_d * 65536)
    # Quadratic position: x(t) = 0 + 1.0*t + 0.5*0.01*t^2 = t + 0.005*t^2
    positions = [1.0 * i + 0.005 * i * i for i in range(500)]
    segments = compress_anchored(ffi_main, ffi_lib, pc, positions, anchor)
    ok, msg = verify_chain_continuity(segments, anchor)
    assert ok, f"FAIL: {msg}"
    assert_chain_matches_positions(positions, segments, 1.0)
    print(f"  -> {len(segments)} segments, all chained continuously")
    print("  PASS")

    # Test 7: regression for the absolute-MSCNT scale bug.  Real trapq feeds
    # absolute MSCNT positions in the tens of thousands while the anchor is
    # reduced mod 1024.  Before the per-segment shift fix, this triggered
    # force-emit length=1 for every sample (200 samples -> 200 segments,
    # blowing the MCU's 1024-node move pool with 4 motors).
    print("Test 7: realistic absolute-MSCNT positions far from reduced anchor")
    anchor_phase = 36
    anchor = (anchor_phase << 16) & 0x03FFFFFF
    # Stepper near 50mm with step_dist=0.0125, microsteps=16 -> ~64000 MSCNT.
    # phase_offset is chosen at activate time so that positions[0] mod 1024
    # exactly equals anchor_phase; reproduce that here by choosing a base
    # value that satisfies (base mod 1024) == anchor_phase.
    base = 64 * 1024.0 + anchor_phase  # = 65572.0, base mod 1024 = 36
    positions = [base + i * 0.5 for i in range(200)]
    segments = compress_anchored(ffi_main, ffi_lib, pc, positions, anchor)
    ok, msg = verify_chain_continuity(segments, anchor)
    assert ok, f"FAIL: {msg}"
    assert_chain_matches_positions(positions, segments, 1.0)
    assert len(segments) <= 4, (
        f"steady-velocity 200-sample stream should compress to <=4 segments, "
        f"got {len(segments)} (regression: positions far from reduced anchor "
        f"used to force-emit length=1 per sample)")
    print(f"  -> {len(segments)} segments for 200 samples (was 200 pre-fix)")
    print("  PASS")

    # Test 8: regression for the chain off-by-one.  With tight max_error and
    # cubic motion (which won't fit a single quadratic), the compressor
    # produces multiple segments.  Pre-fix mirror_advance returned the LAST
    # emitted value of segment K, so segment K+1 anchored one tick behind
    # and force-emitted forever, freezing the emission stream.
    print("Test 8: multi-segment chain with cubic motion (off-by-one regression)")
    ffi_lib.phase_compressor_set_max_error(pc, 0.001)
    try:
        anchor_d = 100.0
        anchor = int(anchor_d * 65536)
        positions = [anchor_d + 0.1 * i + 0.0001 * i * i + 1e-6 * i * i * i
                     for i in range(500)]
        segments = compress_anchored(ffi_main, ffi_lib, pc, positions, anchor)
        ok, msg = verify_chain_continuity(segments, anchor)
        assert ok, f"FAIL: {msg}"
        # Tolerance includes max_error (0.001) plus a small chain slack.
        assert_chain_matches_positions(positions, segments, 0.5)
        # Pre-fix produced 475+ segments (one per sample after seg 0).
        # Post-fix should produce far fewer; bound generously.
        assert len(segments) < 200, (
            f"cubic motion compressed to {len(segments)} segments; expected "
            f"<200 (regression: chain off-by-one used to freeze emission and "
            f"emit length=1 per sample)")
        print(f"  -> {len(segments)} segments for 500 samples "
              f"(was 475+ pre-fix)")
        print("  PASS")
    finally:
        ffi_lib.phase_compressor_set_max_error(pc, 0.25)

    # Test 9: output-cap overflow must be visible to the host.  Before this
    # regression fix, the C helper returned max_out as if compression succeeded,
    # so the host advanced generator time and queued only a partial prefix.
    print("Test 9: segment-cap overflow returns a negative count")
    anchor_d = 100.0
    anchor = int(anchor_d * 65536)
    positions = [anchor_d + 5.0, anchor_d + 5.5, anchor_d + 6.0]
    n_out, segments = compress_anchored_raw(
        ffi_main, ffi_lib, pc, positions, anchor, max_out=1)
    assert n_out == -1, (
        f"overflow with one emitted segment should return -1, got {n_out}")
    assert segments == []
    print("  -> overflow reported as n_out=-1, not a silent partial success")
    print("  PASS")

    print()
    print("All tests passed.  phase_compressor_compress_anchored produces "
          "bit-exact continuous segment chains.")


if __name__ == "__main__":
    main()
