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
    """Closed-form last-emitted, matching C mirror_advance and MCU loop."""
    if count == 0:
        return to_int32(start)
    n_minus_1 = count - 1
    n_choose_2 = (n_minus_1 * (n_minus_1 - 1)) // 2  # (N-1)*(N-2)/2
    raw = start + n_minus_1 * vel + n_choose_2 * accel
    return to_int32(raw)


def reduce_anchor(x):
    """Match C: anchor_reduced = anchor & 0x03FFFFFF (low 26 bits)."""
    return x & 0x03FFFFFF


def make_test_environment():
    ffi_main, ffi_lib = chelper.get_ffi()
    pc = ffi_main.gc(ffi_lib.phase_compressor_alloc(), ffi_lib.free)
    ffi_lib.phase_compressor_set_max_error(pc, 0.25)
    return ffi_main, ffi_lib, pc


def compress_anchored(ffi_main, ffi_lib, pc, positions, anchor):
    """Wrap the C call and return a list of (start, vel, accel, count) tuples."""
    n = len(positions)
    pos_arr = ffi_main.new("double[]", n)
    for i, p in enumerate(positions):
        pos_arr[i] = p
    out_arr = ffi_main.new("struct phase_mcu_move[]", 256)
    n_out = ffi_lib.phase_compressor_compress_anchored(
        pc, pos_arr, n, anchor, out_arr, 256
    )
    return [
        (out_arr[i].start_position, out_arr[i].velocity,
         out_arr[i].acceleration, out_arr[i].count)
        for i in range(n_out)
    ]


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

    # Test 5: random fuzz — produce arbitrary sample streams, verify chain.
    print("Test 5: 200 random fuzz cases")
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
        else:
            # Sanity: total samples covered = n
            covered = sum(s[3] for s in segments)
            assert covered == n, f"trial {trial}: covered {covered} of {n}"
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
    print(f"  -> {len(segments)} segments, all chained continuously")
    print("  PASS")

    print()
    print("All tests passed.  phase_compressor_compress_anchored produces "
          "bit-exact continuous segment chains.")


if __name__ == "__main__":
    main()
