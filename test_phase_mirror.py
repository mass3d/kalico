"""Bit-exact verification of the host-side phase-position mirror.

The host emits compressed phase segments to the MCU. To prevent
resume-from-idle clicks, every new compressed segment must start at
EXACTLY the position the MCU will be holding when the segment
arrives. The host computes that anchor in closed form using the
same int32 arithmetic the MCU uses per-tick. This script verifies
the closed form against an explicit N-tick simulation of the MCU
loop.

If this test ever fails, the host mirror has drifted from the MCU
and segment boundaries will produce phase discontinuities.
"""

import random


INT32_MASK = 0xFFFFFFFF


def _to_int32(x):
    """Sign-extend a Python int to a 32-bit signed value (matches C int32)."""
    x &= INT32_MASK
    if x & 0x80000000:
        x -= 0x100000000
    return x


def simulate_advance(start, vel, accel, count):
    """N-tick simulation matching the MCU's phase_stepper_advance loop:
        emitted = position
        position += velocity
        velocity += acceleration
        if --count == 0: position = emitted; load_next.
    Returns the LAST emitted position (which is what the MCU restores to
    `position` and what the next segment must anchor at).
    All arithmetic wraps at int32 boundaries.
    """
    if count <= 0:
        return _to_int32(start)
    position = _to_int32(start)
    velocity = _to_int32(vel)
    acceleration = _to_int32(accel)
    last_emitted = position
    for _ in range(count):
        last_emitted = position
        position = _to_int32(position + velocity)
        velocity = _to_int32(velocity + acceleration)
    return last_emitted


def advance_mirror(start, vel, accel, count):
    """Closed-form last-emitted position after a (start, vel, accel, count)
    segment, matching the MCU loop. All arithmetic done as Python ints,
    wrapped to int32 at the end.

    Derivation (count=N, last emitted is at tick N-1, zero-indexed):
        emit_i = start + i*vel + accel * i*(i-1)/2
        last  = start + (N-1)*vel + accel * (N-1)*(N-2)/2
    """
    if count <= 0:
        return _to_int32(start)
    n_minus_1 = count - 1
    n_choose_2 = n_minus_1 * (count - 2) // 2  # exact even/odd handled by //2
    # Note: (N-1)*(N-2) is always even, so the //2 is exact integer division.
    raw = start + n_minus_1 * vel + n_choose_2 * accel
    return _to_int32(raw)


def random_int32():
    return random.randint(-0x80000000, 0x7FFFFFFF)


def run_tests(n_trials=10000, seed=0):
    random.seed(seed)
    failures = []
    for trial in range(n_trials):
        # Mix of small and large values to stress overflow paths.
        if trial % 3 == 0:
            start = random.randint(-1024 << 16, (1024 << 16) - 1)
            vel = random.randint(-1 << 20, 1 << 20)
            accel = random.randint(-1 << 12, 1 << 12)
        else:
            start = random_int32()
            vel = random_int32()
            accel = random_int32()
        count = random.randint(1, 4096)

        sim = simulate_advance(start, vel, accel, count)
        cf = advance_mirror(start, vel, accel, count)
        if sim != cf:
            failures.append((start, vel, accel, count, sim, cf))
            if len(failures) >= 5:
                break
    return failures


if __name__ == "__main__":
    # Boundary cases first
    boundary_cases = [
        # (start, vel, accel, count, expected_last_emitted)
        (0, 0, 0, 1, 0),
        (0, 0, 0, 100, 0),
        (12345, 0, 0, 50, 12345),
        (0, 65536, 0, 1, 0),
        (0, 65536, 0, 4, 3 * 65536),
        (0, 0, 65536, 4, 3),  # accel=1 (in 16.16); last emit = (3*2/2)*65536 = 3*65536? wait
        # accel=65536 means accel/tick = 1.0 microstep/tick^2
        # last emit at tick 3 = 0 + 3*0 + 3*2/2 * 65536 = 3 * 65536 = 196608
    ]
    boundary_cases[-1] = (0, 0, 65536, 4, 3 * 65536)

    for case in boundary_cases:
        start, vel, accel, count, expected = case
        sim = simulate_advance(start, vel, accel, count)
        cf = advance_mirror(start, vel, accel, count)
        ok = sim == cf == expected
        print(f"  case start={start} vel={vel} accel={accel} count={count}: "
              f"sim={sim} cf={cf} expected={expected} {'OK' if ok else 'FAIL'}")
        assert ok, f"Boundary case failed: {case}"

    # Random fuzz
    print(f"Running 10000 random fuzz trials...")
    failures = run_tests(n_trials=10000, seed=42)
    if failures:
        print(f"FAILURES ({len(failures)} of first 5):")
        for f in failures:
            start, vel, accel, count, sim, cf = f
            print(f"  start={start:#010x} vel={vel:#010x} accel={accel:#010x} "
                  f"count={count}: sim={sim:#010x} cf={cf:#010x} "
                  f"diff={(sim - cf) & INT32_MASK:#010x}")
        raise SystemExit(1)
    print("All 10000 trials passed. Closed form is bit-exact with simulation.")

    # Overflow-stress trials
    print("Running 1000 overflow-heavy trials...")
    random.seed(100)
    overflow_failures = []
    for _ in range(1000):
        # Large accel to force velocity wrap mid-segment
        start = random.randint(-1 << 25, 1 << 25)
        vel = random.randint(-1 << 28, 1 << 28)
        accel = random.randint(-1 << 28, 1 << 28)
        count = random.randint(1, 8192)
        sim = simulate_advance(start, vel, accel, count)
        cf = advance_mirror(start, vel, accel, count)
        if sim != cf:
            overflow_failures.append((start, vel, accel, count, sim, cf))
            if len(overflow_failures) >= 5:
                break
    if overflow_failures:
        print(f"OVERFLOW FAILURES ({len(overflow_failures)}):")
        for f in overflow_failures:
            print(f"  {f}")
        raise SystemExit(1)
    print("All overflow trials passed.")
