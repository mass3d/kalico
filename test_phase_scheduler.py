#!/usr/bin/env python3
"""
Host-side phase stepping protocol and scheduler tests.

These tests focus on the AWD/CoreXY failure modes that are hard to expose in
fileoutput mode:
  - reset boundaries after idle / diagnostics
  - holding the last phase across natural idle without replaying stale motion
  - independent due times on a shared MCU clock
  - stripping the first moving sample instead of preserving standstill
  - keeping homing / probe windows from resuming too early
"""

from dataclasses import dataclass
import math
from typing import Optional


@dataclass
class HostResetState:
    needs_clock_reset: bool
    expected_end_clock: int

    def boundary(self):
        self.needs_clock_reset = True
        self.expected_end_clock = 0

    def queue_batch(self, start_clock, total_count, interval):
        need_reset = self.needs_clock_reset
        if need_reset:
            self.needs_clock_reset = False
        self.expected_end_clock = start_clock + total_count * interval
        return need_reset


@dataclass
class EmulatedMCUMotor:
    need_reset: bool = True
    accepted_batches: int = 0

    def reset(self, _clock):
        self.need_reset = False

    def queue_batch(self):
        if self.need_reset:
            return False
        self.accepted_batches += 1
        return True

    def drain(self):
        self.need_reset = True


@dataclass
class ContinuousIdleMCUMotor:
    need_reset: bool = True
    accepted_batches: int = 0

    def reset(self, _clock):
        self.need_reset = False

    def queue_batch(self):
        if self.need_reset:
            return False
        self.accepted_batches += 1
        return True

    def natural_idle(self):
        self.need_reset = False


@dataclass
class ResetArmedMCUMotor:
    name: str
    need_reset: bool = True
    reset_armed: bool = False
    count: int = 0
    position: int = 0
    emitted: Optional[list] = None
    accepted_batches: int = 0

    def __post_init__(self):
        if self.emitted is None:
            self.emitted = []


@dataclass
class ScheduledMotor:
    name: str
    interval: int = 0
    remaining: int = 0
    next_clock: Optional[int] = None
    first_event_clock: Optional[int] = None


@dataclass
class DelayedResumeState:
    generation: int = 0
    scheduled_generation: int = 0
    resume_count: int = 0

    def cancel(self):
        self.generation += 1
        self.scheduled_generation = 0

    def schedule(self):
        self.generation += 1
        self.scheduled_generation = self.generation
        return self.scheduled_generation

    def fire(self, generation):
        if not self.scheduled_generation:
            return False
        if generation != self.scheduled_generation or generation != self.generation:
            return False
        self.scheduled_generation = 0
        self.resume_count += 1
        return True


@dataclass
class HomingWindowState:
    resume: DelayedResumeState
    home_rails_depth: int = 0
    homing_move_depth: int = 0

    def home_rails_begin(self):
        self.home_rails_depth += 1
        self.resume.cancel()

    def home_rails_end(self):
        if self.home_rails_depth:
            self.home_rails_depth -= 1
        if self.home_rails_depth or self.homing_move_depth:
            return None
        return self.resume.schedule()

    def homing_move_begin(self):
        self.homing_move_depth += 1
        self.resume.cancel()

    def homing_move_end(self):
        if self.homing_move_depth:
            self.homing_move_depth -= 1
        if self.home_rails_depth or self.homing_move_depth:
            return None
        return self.resume.schedule()


@dataclass
class ProbeAwareResumeState:
    resume: DelayedResumeState
    probe_pending: bool = False

    def fire(self, generation):
        if self.probe_pending:
            return False
        return self.resume.fire(generation)


@dataclass
class HomingDeferredResumeState:
    resume: DelayedResumeState
    home_rails_depth: int = 0
    homing_move_depth: int = 0

    def fire(self, generation):
        if self.home_rails_depth or self.homing_move_depth:
            return "retry"
        return self.resume.fire(generation)


@dataclass
class BufferedMotionResumeState:
    resume: DelayedResumeState
    buffered_motion: float = 0.0
    idle_slack: float = 0.05
    special_queuing_state: str = ""

    def fire(self, generation):
        if self.special_queuing_state:
            return self.resume.fire(generation)
        if self.buffered_motion > self.idle_slack:
            return False
        return self.resume.fire(generation)


@dataclass
class ExactOidResponseRouter:
    handlers: dict

    def __init__(self):
        self.handlers = {}

    def register(self, name, oid=None):
        self.handlers[name, oid] = True

    def deliver(self, name, oid=None):
        return self.handlers.get((name, oid), False)


@dataclass
class IdleEntryState:
    mcu_hold_active: bool = False
    needs_clock_reset: bool = True

    def enter_idle(self):
        self.needs_clock_reset = not self.mcu_hold_active


@dataclass
class TMCStandstillPolicy:
    ihold: int
    irun: int
    iholddelay: int
    tpowerdown: int
    faststandstill: bool


def phase_active_policy(policy, active_tpowerdown=255,
                        disable_faststandstill=True):
    return TMCStandstillPolicy(
        ihold=policy.irun,
        irun=policy.irun,
        iholddelay=policy.iholddelay,
        tpowerdown=active_tpowerdown,
        faststandstill=(False if disable_faststandstill
                        else policy.faststandstill),
    )


def activation_preload(mscnt, live_cur_a, live_cur_b):
    if live_cur_a or live_cur_b:
        return live_cur_a, live_cur_b, "mscuract"
    angle = mscnt * math.pi / 512.0
    return (int(round(248.0 * math.cos(angle))),
            int(round(248.0 * math.sin(angle))),
            "mscnt")


def phase_from_currents(cur_a, cur_b):
    if not cur_a and not cur_b:
        return None
    phase = int(round(math.atan2(cur_b, cur_a)
                      * 1024.0 / (2.0 * math.pi)))
    return phase & 0x3FF


def phase_delta(from_phase, to_phase):
    if from_phase is None or to_phase is None:
        return None
    delta = (to_phase - from_phase) & 0x3FF
    if delta >= 0x200:
        delta -= 0x400
    return delta


def phase_position_to_mcu_phase(position):
    reduced = math.fmod(position, 1024.0)
    fixed = int(reduced * 65536.0)
    return (fixed >> 16) & 0x3FF


def vector_peak_delta(cur_a, cur_b, next_a, next_b):
    return max(abs(cur_a - next_a), abs(cur_b - next_b))


def continuity_snapshot(origin, from_phase, to_phase):
    from_a = int(round(248.0 * math.cos(from_phase * 2.0 * math.pi / 1024.0)))
    from_b = int(round(248.0 * math.sin(from_phase * 2.0 * math.pi / 1024.0)))
    to_a = int(round(248.0 * math.cos(to_phase * 2.0 * math.pi / 1024.0)))
    to_b = int(round(248.0 * math.sin(to_phase * 2.0 * math.pi / 1024.0)))
    return {
        "origin": origin,
        "from_phase": from_phase,
        "to_phase": to_phase,
        "delta_phase": phase_delta(from_phase, to_phase),
        "delta_peak": vector_peak_delta(from_a, from_b, to_a, to_b),
        "from_a": from_a,
        "from_b": from_b,
        "to_a": to_a,
        "to_b": to_b,
    }


def resolve_phase_direction(invert_dir, override=0):
    if override:
        return 1 if override > 0 else -1
    return -1 if invert_dir else 1


def decode_mscuract_5160(raw):
    cur_b = raw & 0x1FF
    cur_a = (raw >> 16) & 0x1FF
    if cur_a & 0x100:
        cur_a -= 0x200
    if cur_b & 0x100:
        cur_b -= 0x200
    return cur_a, cur_b


class GlobalCadenceScheduler:
    """Emulates the current broken single-cadence MCU scheduler."""

    def __init__(self):
        self.motors = {}
        self.active = False
        self.interval = 0
        self.next_clock = 0

    def _motor(self, name):
        return self.motors.setdefault(name, ScheduledMotor(name=name))

    def start_motor(self, name, start_clock, interval, count):
        motor = self._motor(name)
        motor.interval = interval
        motor.remaining = count
        motor.next_clock = start_clock
        if not self.active:
            self.active = True
            self.interval = interval
            self.next_clock = start_clock

    def step_once(self):
        if not self.active:
            return False
        now = self.next_clock
        any_active = False
        for motor in self.motors.values():
            if not motor.remaining:
                continue
            if motor.first_event_clock is None:
                motor.first_event_clock = now
            motor.remaining -= 1
            any_active = any_active or bool(motor.remaining)
        if any_active:
            self.next_clock += self.interval
            return True
        self.active = False
        return False


class IndependentScheduler:
    """Emulates the fixed shared-clock, independent-due-time model."""

    def __init__(self):
        self.motors = {}

    def _motor(self, name):
        return self.motors.setdefault(name, ScheduledMotor(name=name))

    def start_motor(self, name, start_clock, interval, count):
        motor = self._motor(name)
        motor.interval = interval
        motor.remaining = count
        motor.next_clock = start_clock

    def step_once(self):
        active = [
            motor for motor in self.motors.values() if motor.remaining and
            motor.next_clock is not None
        ]
        if not active:
            return False
        now = min(motor.next_clock for motor in active)
        for motor in active:
            if motor.next_clock != now:
                continue
            if motor.first_event_clock is None:
                motor.first_event_clock = now
            motor.remaining -= 1
            if motor.remaining:
                motor.next_clock += motor.interval
            else:
                motor.next_clock = None
        return True


class ResetArmedPhaseGroup:
    """Small model of src/tmc_phase_stepper.c reset/arm semantics.

    reset_phase_clock must not unmask stale position output.  It should mark
    a motor as reset-armed, and the first queue_phase_move must atomically load
    the new segment before clearing NEED_RESET.
    """

    def __init__(self, motors):
        self.motors = {motor.name: motor for motor in motors}
        self.active = False
        self.interval = 0
        self.waketime = 0

    def has_armed_motors(self):
        return any(not motor.need_reset for motor in self.motors.values())

    def stop_if_unarmed(self):
        if self.has_armed_motors():
            return
        self.active = False
        self.interval = 0

    def reset(self, name, clock):
        motor = self.motors[name]
        motor.count = 0
        motor.need_reset = True
        motor.reset_armed = True
        self.stop_if_unarmed()
        if not self.active:
            self.waketime = clock

    def stop(self, name):
        motor = self.motors[name]
        motor.count = 0
        motor.need_reset = True
        motor.reset_armed = False
        self.stop_if_unarmed()

    def queue(self, name, interval, start_position, count):
        motor = self.motors[name]
        if motor.need_reset:
            if not motor.reset_armed:
                return False
            motor.position = start_position
            motor.count = count
            motor.need_reset = False
            motor.reset_armed = False
            motor.accepted_batches += 1
        elif motor.count:
            motor.accepted_batches += 1
        else:
            motor.position = start_position
            motor.count = count
            motor.accepted_batches += 1
        if self.interval == 0:
            self.interval = interval
        if not self.active:
            self.active = True
        return True

    def event(self):
        if not self.active:
            return False
        if not self.has_armed_motors():
            self.stop_if_unarmed()
            return False
        for motor in self.motors.values():
            if motor.need_reset:
                continue
            motor.emitted.append(motor.position)
            if motor.count:
                motor.count -= 1
        self.waketime += self.interval
        return True


def run_until_idle(scheduler):
    while scheduler.step_once():
        pass


def find_move_start(positions, threshold):
    if len(positions) <= 1:
        return 0
    base = positions[0]
    for index, position in enumerate(positions[1:], start=1):
        diff = position - base
        if diff > threshold or diff < -threshold:
            return index
    return len(positions)


def find_resume_window(positions, idle_position, threshold):
    last_match = None
    for index, position in enumerate(positions):
        diff = position - idle_position
        if diff <= threshold and diff >= -threshold:
            last_match = index
            continue
        if last_match is not None:
            return last_match, index
    if last_match is None:
        return None
    return last_match, len(positions)


def trim_idle_resume(positions, idle_position, threshold):
    trimmed = list(positions)
    resume_window = find_resume_window(trimmed, idle_position, threshold)
    if resume_window is not None:
        trim_start, move_start = resume_window
    else:
        if abs(trimmed[0] - idle_position) > threshold:
            trimmed = [idle_position] + trimmed
        move_start = find_move_start(trimmed, threshold)
        trim_start = max(0, move_start - 1)
    return trimmed[trim_start:], trim_start


def retry_resume_start(flush_time, resume_lookback):
    return max(0.0, flush_time - resume_lookback)


def recent_idle_tail_start(flush_time, idle_lookback, resume_lookback):
    return max(0.0, flush_time - max(idle_lookback, resume_lookback))


def sample_positions(last_flush_time, flush_time, interval, max_samples,
                     hold_position, move_start, velocity):
    positions = []
    t = last_flush_time
    while t < flush_time and len(positions) < max_samples:
        if t < move_start:
            pos = hold_position
        else:
            pos = hold_position + (t - move_start) * velocity
        positions.append(pos)
        t += interval
    return positions


def sample_piecewise_positions(last_flush_time, flush_time, interval,
                               max_samples, hold_position,
                               move_start, move_end, end_position):
    positions = []
    t = last_flush_time
    velocity = ((end_position - hold_position) / (move_end - move_start))
    while t < flush_time and len(positions) < max_samples:
        if t < move_start:
            pos = hold_position
        elif t < move_end:
            pos = hold_position + (t - move_start) * velocity
        else:
            pos = end_position
        positions.append(pos)
        t += interval
    return positions


def classify_flush_endpoints(first_pos, last_pos):
    if not math.isfinite(first_pos) or not math.isfinite(last_pos):
        return "boundary"
    if abs(last_pos - first_pos) < 0.5:
        return "standstill"
    return "moving"


def handoff_stepdir(sync_before_unmask):
    stepcompress_synced = False
    commanded_pos_synced = False
    phase_stepping = True

    if sync_before_unmask:
        stepcompress_synced = True
        commanded_pos_synced = True
    phase_stepping = False
    # Toolhead flush may immediately call generate_steps once step/dir resumes.
    if (not phase_stepping
            and (not stepcompress_synced or not commanded_pos_synced)):
        return False
    return True


@dataclass
class ExternalProbeHomingState:
    resume: DelayedResumeState
    external_probe_flow: bool = False
    home_rails_depth: int = 0
    homing_move_depth: int = 0

    def homing_move_end(self):
        if self.homing_move_depth:
            self.homing_move_depth -= 1
        if self.home_rails_depth or self.homing_move_depth:
            return None
        if self.external_probe_flow:
            return None
        return self.resume.schedule()

    def home_rails_end(self):
        if self.home_rails_depth:
            self.home_rails_depth -= 1
        if self.home_rails_depth or self.homing_move_depth:
            return None
        if self.external_probe_flow:
            return None
        return self.resume.schedule()


def test_find_move_start_returns_first_moving_sample():
    move_start = find_move_start([0.0, 0.0, 0.2, 0.8, 1.2, 2.0], 1.0)
    assert move_start == 4, move_start
    print("PASS: first moving sample is returned for mixed standstill flushes")


def test_resume_trim_ignores_replayed_pre_idle_motion():
    positions = [-544629.0, -500000.0, -460000.0, -416629.0, -416629.0,
                 -455621.0, -544168.0]
    trim_start, move_start = find_resume_window(
        positions, idle_position=-416629.0, threshold=1.0)
    assert trim_start == 4, trim_start
    assert move_start == 5, move_start
    print("PASS: resume trim anchors to the held idle phase, not stale motion")


def test_resume_keeps_one_held_phase_sample():
    positions = [0.0, 0.0, 0.2, 0.8, 1.2, 2.0]
    trim_start, move_start = find_resume_window(
        positions, idle_position=0.0, threshold=1.0)
    assert trim_start == 3, trim_start
    assert move_start == 4, move_start
    trimmed = positions[trim_start:]
    assert trimmed[0] == 0.8, trimmed
    assert trimmed[1] == 1.2, trimmed
    print("PASS: resume keeps one held-phase anchor sample before motion")


def test_natural_idle_flush_preserves_held_phase_on_resume():
    interval = 0.1
    max_samples = 8
    flush_time = 10.8
    hold_position = 98196.0
    move_start = 10.35
    velocity = -6400.0

    sampled = sample_positions(
        last_flush_time=10.0, flush_time=flush_time,
        interval=interval, max_samples=max_samples,
        hold_position=hold_position, move_start=move_start,
        velocity=velocity)

    assert sampled[0] == hold_position, sampled
    move_start_index = find_move_start(sampled, 1.0)
    assert 0 < move_start_index < len(sampled), move_start_index
    print("PASS: advancing the generator through natural idle still"
          " preserves the held phase before a resumed move")


def test_resume_retry_rewinds_to_recent_idle_history():
    interval = 0.1
    max_samples = 8
    flush_time = 10.0
    hold_position = -30212.0
    move_start = 9.35
    velocity = -6400.0

    current = sample_positions(
        last_flush_time=9.5, flush_time=flush_time, interval=interval,
        max_samples=max_samples, hold_position=hold_position,
        move_start=move_start, velocity=velocity)
    retry = sample_positions(
        last_flush_time=retry_resume_start(flush_time, 1.0),
        flush_time=flush_time, interval=interval, max_samples=max_samples,
        hold_position=hold_position, move_start=move_start,
        velocity=velocity)

    assert current[0] != hold_position, current
    assert retry[0] == hold_position, retry
    assert find_resume_window(retry, hold_position, 1.0) is not None, retry
    print("PASS: idle resumes that miss the held phase retry from recent"
          " idle history instead of synthesizing a bad anchor")


def test_long_idle_flush_rechecks_recent_tail_before_declaring_standstill():
    interval = 0.1
    max_samples = 32
    flush_time = 150.567
    old_start = 136.805
    hold_position = -209436.993
    end_position = 33762.993
    move_start = 149.000
    move_end = 149.500

    head_only = sample_piecewise_positions(
        last_flush_time=old_start, flush_time=flush_time,
        interval=interval, max_samples=max_samples,
        hold_position=hold_position, move_start=move_start,
        move_end=move_end, end_position=end_position)
    tail_start = recent_idle_tail_start(
        flush_time, idle_lookback=0.5, resume_lookback=2.0)
    recent_tail = sample_piecewise_positions(
        last_flush_time=tail_start, flush_time=flush_time,
        interval=interval, max_samples=max_samples,
        hold_position=hold_position, move_start=move_start,
        move_end=move_end, end_position=end_position)

    assert all(abs(pos - hold_position) < 0.5 for pos in head_only), head_only
    assert abs(recent_tail[-1] - end_position) < 0.5, recent_tail[-5:]
    assert abs(recent_tail[0] - hold_position) < 0.5, recent_tail[:5]
    print("PASS: long idle standstill flushes recheck the recent tail so a"
          " completed move is not skipped")


def test_natural_idle_does_not_need_host_hold_padding():
    host = HostResetState(needs_clock_reset=True, expected_end_clock=0)
    mcu = ContinuousIdleMCUMotor()
    interval = 100

    need_reset = host.queue_batch(start_clock=1000, total_count=100,
                                  interval=interval)
    assert need_reset
    mcu.reset(1000)
    assert mcu.queue_batch()

    # Natural idle now relies on the existing held phase on the MCU side, not
    # on host-generated standstill padding.
    mcu.natural_idle()

    need_reset = host.queue_batch(start_clock=15000, total_count=60,
                                  interval=interval)
    assert not need_reset
    assert mcu.queue_batch()
    print("PASS: natural idle continues from the held phase without"
          " host-side hold padding")


def test_pre_move_idle_keeps_clock_reset_armed():
    state = IdleEntryState(mcu_hold_active=False, needs_clock_reset=True)

    state.enter_idle()

    assert state.needs_clock_reset, \
        "idle entered before the first MCU batch must keep the reset armed"
    print("PASS: pre-move idle keeps the first reset_phase_clock armed")


def test_natural_idle_keeps_mcu_hold_active_without_reset():
    host = HostResetState(needs_clock_reset=True, expected_end_clock=0)
    mcu = ContinuousIdleMCUMotor()
    interval = 100

    need_reset = host.queue_batch(start_clock=1000, total_count=100,
                                  interval=interval)
    assert need_reset
    mcu.reset(1000)
    assert mcu.queue_batch()
    mcu.natural_idle()

    need_reset = host.queue_batch(start_clock=24000, total_count=60,
                                  interval=interval)
    assert not need_reset
    assert mcu.queue_batch()
    print("PASS: natural idle keeps the MCU hold active without a reset")


def test_query_response_router_requires_exact_oid_match():
    router = ExactOidResponseRouter()

    router.register("phase_stepper_status")
    assert not router.deliver("phase_stepper_status", oid=17), \
        "a query handler registered without oid will miss oid-tagged replies"

    router = ExactOidResponseRouter()
    router.register("phase_stepper_status", oid=17)
    assert router.deliver("phase_stepper_status", oid=17), \
        "registering the handler with the phase stepper oid matches the reply"
    print("PASS: MCU query handlers must register with the exact oid")


def test_explicit_idle_boundary_requires_new_clock_anchor():
    host = HostResetState(needs_clock_reset=True, expected_end_clock=0)
    mcu = EmulatedMCUMotor()
    interval = 100

    need_reset = host.queue_batch(start_clock=1000, total_count=100,
                                  interval=interval)
    assert need_reset
    mcu.reset(1000)
    assert mcu.queue_batch()
    mcu.drain()
    host.boundary()

    need_reset = host.queue_batch(start_clock=24000, total_count=60,
                                  interval=interval)
    assert need_reset
    mcu.reset(24000)
    assert mcu.queue_batch()
    print("PASS: explicit idle boundaries still require a new clock anchor")


def test_idle_boundary_forces_reset_before_resume():
    host = HostResetState(needs_clock_reset=False, expected_end_clock=25000)
    mcu = EmulatedMCUMotor(need_reset=False)
    mcu.drain()
    host.boundary()

    need_reset = host.queue_batch(start_clock=30000, total_count=25, interval=100)
    assert need_reset, "resume batch must send reset after an idle boundary"
    if need_reset:
        mcu.reset(30000)
    assert mcu.queue_batch(), "MCU should accept the first post-idle batch"
    print("PASS: idle boundary forces a reset before resume")


def test_diagnostic_boundary_forces_reset_before_resume():
    host = HostResetState(needs_clock_reset=False, expected_end_clock=42000)
    mcu = EmulatedMCUMotor(need_reset=False)

    # PHASE_STEPPER_STATUS stops the ISR and drains the queue.
    mcu.drain()
    host.boundary()

    need_reset = host.queue_batch(start_clock=46000, total_count=10, interval=100)
    assert need_reset, "diagnostics must force a reset before the next move"
    if need_reset:
        mcu.reset(46000)
    assert mcu.queue_batch(), "MCU should accept the first post-diagnostic batch"
    print("PASS: diagnostic stop forces a reset before resume")


def test_reset_clock_masks_motor_until_first_segment_is_loaded():
    x = ResetArmedMCUMotor(
        name="X", need_reset=False, position=1111, count=0)
    y = ResetArmedMCUMotor(
        name="Y", need_reset=False, position=2222, count=2)
    group = ResetArmedPhaseGroup([x, y])
    group.active = True
    group.interval = 100
    group.waketime = 1000

    group.reset("X", clock=5000)
    assert group.active, "the group remains active because Y is still armed"
    assert x.need_reset and x.reset_armed

    assert group.event()
    assert x.emitted == [], \
        "reset-armed motor must not emit stale position while waiting for queue"
    assert y.emitted == [2222]

    assert group.queue("X", interval=100, start_position=3333, count=1)
    assert not x.need_reset and not x.reset_armed
    assert group.event()
    assert x.emitted == [3333], \
        "first post-reset emission must be the newly queued segment start"
    print("PASS: reset clock masks a motor until its first segment is loaded")


def test_reset_clock_stops_idle_group_and_honors_new_waketime():
    x = ResetArmedMCUMotor(
        name="X", need_reset=False, position=1111, count=0)
    group = ResetArmedPhaseGroup([x])
    group.active = True
    group.interval = 100
    group.waketime = 1200

    group.reset("X", clock=5000)

    assert not group.active, \
        "when all motors are reset-masked the shared timer must stop"
    assert group.interval == 0
    assert group.waketime == 5000, \
        "reset while idle must install the requested reset_phase_clock waketime"
    assert not group.event()
    assert x.emitted == [], "no stale phase should emit between reset and queue"

    assert group.queue("X", interval=80, start_position=4444, count=1)
    assert group.active
    assert group.interval == 80
    assert group.waketime == 5000
    assert group.event()
    assert x.emitted == [4444]
    print("PASS: reset clock stops an idle group and preserves the new waketime")


def test_stop_requires_explicit_reset_before_queueing_again():
    x = ResetArmedMCUMotor(
        name="X", need_reset=False, position=1111, count=0)
    group = ResetArmedPhaseGroup([x])
    group.active = True
    group.interval = 100
    group.waketime = 1200

    group.stop("X")
    assert not group.active
    assert group.interval == 0
    assert x.need_reset and not x.reset_armed
    assert not group.queue("X", interval=100, start_position=2222, count=1), \
        "stop_phase_stepper should require reset_phase_clock before new motion"

    group.reset("X", clock=6000)
    assert group.queue("X", interval=100, start_position=2222, count=1)
    assert group.event()
    assert x.emitted == [2222]
    print("PASS: stopped phase steppers require an explicit reset before queue")


def test_global_cadence_starts_future_motor_too_early():
    broken = GlobalCadenceScheduler()
    fixed = IndependentScheduler()

    broken.start_motor("A", start_clock=100, interval=10, count=4)
    fixed.start_motor("A", start_clock=100, interval=10, count=4)

    broken.start_motor("B", start_clock=130, interval=10, count=2)
    fixed.start_motor("B", start_clock=130, interval=10, count=2)

    run_until_idle(broken)
    run_until_idle(fixed)

    assert broken.motors["B"].first_event_clock == 100
    assert fixed.motors["B"].first_event_clock == 130
    print("PASS: independent scheduler preserves a future motor start clock")


def test_global_cadence_shifts_resumed_motor_to_wrong_tick():
    broken = GlobalCadenceScheduler()
    fixed = IndependentScheduler()

    broken.start_motor("A", start_clock=100, interval=10, count=6)
    fixed.start_motor("A", start_clock=100, interval=10, count=6)

    broken.step_once()
    fixed.step_once()

    broken.start_motor("B", start_clock=125, interval=5, count=3)
    fixed.start_motor("B", start_clock=125, interval=5, count=3)

    run_until_idle(broken)
    run_until_idle(fixed)

    assert broken.motors["B"].first_event_clock == 110
    assert fixed.motors["B"].first_event_clock == 125
    print("PASS: resumed motor keeps its own cadence on the shared clock")


def test_homing_move_end_does_not_resume_inside_home_rails():
    state = HomingWindowState(resume=DelayedResumeState())

    state.home_rails_begin()
    state.homing_move_begin()
    generation = state.homing_move_end()

    assert generation is None, \
        "individual probe moves must not resume while the home_rails window is active"
    assert state.resume.scheduled_generation == 0

    generation = state.home_rails_end()
    assert generation is not None
    assert state.resume.fire(generation)
    print("PASS: homing_move_end waits for home_rails_end before resuming")


def test_nested_homing_cancels_early_resume():
    resume = DelayedResumeState()

    first = resume.schedule()
    resume.cancel()
    assert not resume.fire(first), \
        "an early home_rails_end resume must be canceled by a new begin"

    second = resume.schedule()
    assert resume.fire(second), \
        "the final homing end should be allowed to resume phase stepping"
    assert resume.resume_count == 1
    print("PASS: nested homing windows cancel stale resume timers")


def test_only_latest_manual_resume_can_fire():
    resume = DelayedResumeState()

    first = resume.schedule()
    second = resume.schedule()
    assert not resume.fire(first), \
        "a stale scheduled resume must not reactivate phase stepping"
    assert resume.fire(second), \
        "the most recent scheduled resume should activate"
    print("PASS: only the latest scheduled resume can fire")


def test_probe_window_defers_resume_until_calibration_finishes():
    state = ProbeAwareResumeState(resume=DelayedResumeState(), probe_pending=True)

    generation = state.resume.schedule()
    assert not state.fire(generation), \
        "phase stepping must stay suspended while probe multi-probe is active"
    assert state.resume.scheduled_generation == generation

    state.probe_pending = False
    assert state.fire(generation), \
        "resume should fire once probe multi-probe completes"
    print("PASS: probe multi-probe defers resume until calibration ends")


def test_manual_delayed_resume_retries_until_homing_clears():
    state = HomingDeferredResumeState(
        resume=DelayedResumeState(), home_rails_depth=1)

    generation = state.resume.schedule()
    assert state.fire(generation) == "retry", \
        "manual delayed resume should keep retrying while homing is active"
    assert state.resume.scheduled_generation == generation

    state.home_rails_depth = 0
    assert state.fire(generation), \
        "the same delayed resume should fire once homing clears"
    print("PASS: manual delayed resume retries until homing completes")


def test_buffered_motion_defers_resume_until_printer_is_idle():
    state = BufferedMotionResumeState(
        resume=DelayedResumeState(), buffered_motion=0.8)

    generation = state.resume.schedule()
    assert not state.fire(generation), \
        "delayed resume must not activate after motion is already buffered"

    state.buffered_motion = 0.0
    assert state.fire(generation), \
        "resume should fire once the motion queue drains"
    print("PASS: delayed resume waits for buffered motion to drain")


def test_priming_buffer_does_not_block_resume():
    state = BufferedMotionResumeState(
        resume=DelayedResumeState(), buffered_motion=0.25,
        special_queuing_state="NeedPrime")

    generation = state.resume.schedule()
    assert state.fire(generation), \
        "NeedPrime/Priming buffering must not block phase-stepper resume"
    print("PASS: priming buffer does not block delayed resume")


def test_suspend_resyncs_before_stepdir_unmasks():
    assert not handoff_stepdir(sync_before_unmask=False), \
        ("re-enabling step/dir before syncing the true commanded position"
         " reproduces the invalid-sequence handoff")
    assert handoff_stepdir(sync_before_unmask=True), \
        ("suspend must resync both stepcompress and commanded_pos before"
         " normal step/dir resumes")
    print("PASS: suspend resyncs stepcompress and commanded_pos before"
          " step/dir unmasks")


def test_nonfinite_endpoints_force_idle_boundary():
    assert classify_flush_endpoints(float("nan"), 0.0) == "boundary"
    assert classify_flush_endpoints(0.0, float("inf")) == "boundary"
    print("PASS: non-finite endpoints force a reset boundary instead of idle hold")


def test_external_beacon_flow_suppresses_homing_auto_resume():
    state = ExternalProbeHomingState(
        resume=DelayedResumeState(), external_probe_flow=True)

    assert state.homing_move_end() is None
    assert state.home_rails_end() is None
    assert state.resume.scheduled_generation == 0
    print("PASS: external Beacon/probe flows suppress homing auto-resume")


def test_phase_active_policy_keeps_run_current_and_delays_powerdown():
    cached = TMCStandstillPolicy(
        ihold=12, irun=28, iholddelay=6, tpowerdown=10, faststandstill=True)
    active = phase_active_policy(cached, active_tpowerdown=255,
                                 disable_faststandstill=True)

    assert active.ihold == cached.irun
    assert active.irun == cached.irun
    assert active.iholddelay == cached.iholddelay
    assert active.tpowerdown == 255
    assert not active.faststandstill
    print("PASS: active phase policy keeps run current and overrides"
          " standstill timeout")


def test_phase_active_policy_does_not_mutate_cached_driver_settings():
    cached = TMCStandstillPolicy(
        ihold=10, irun=24, iholddelay=4, tpowerdown=10, faststandstill=False)
    active = phase_active_policy(cached, active_tpowerdown=255,
                                 disable_faststandstill=True)

    assert cached == TMCStandstillPolicy(
        ihold=10, irun=24, iholddelay=4, tpowerdown=10,
        faststandstill=False)
    assert active != cached
    print("PASS: phase activation policy leaves cached TMC settings intact"
          " for restore")


def test_activation_preload_prefers_live_current_vector():
    preload_a, preload_b, source = activation_preload(508, -17, 245)
    assert (preload_a, preload_b, source) == (-17, 245, "mscuract")
    print("PASS: activation preload prefers the live MSCURACT vector")


def test_phase_from_currents_roundtrips_logged_activation_vectors():
    assert phase_from_currents(16, 246) == 245
    assert phase_from_currents(-180, 169) == 389
    assert phase_from_currents(-7, 247) == 261
    print("PASS: direct-mode phase diagnostics can recover phase from"
          " logged current vectors")


def test_phase_position_to_mcu_phase_matches_fixed_point_wrap():
    assert phase_position_to_mcu_phase(-30476.790) == 243
    assert phase_position_to_mcu_phase(-210955.993) == 1012
    assert phase_position_to_mcu_phase(32243.993) == 499
    print("PASS: continuity diagnostics use the same wrapped phase as the MCU")


def test_continuity_snapshot_sees_small_idle_restart_jump():
    snap = continuity_snapshot("idle_resume", 1012, 1013)
    assert snap["delta_phase"] == 1
    assert snap["delta_peak"] <= 2
    print("PASS: idle restarts report a one-phase continuity jump when"
          " the restart is electrically smooth")


def test_continuity_snapshot_sees_large_preload_readback_mismatch():
    snap = continuity_snapshot("activation", 244, 994)
    assert snap["delta_phase"] == -274
    assert snap["delta_peak"] > 200
    print("PASS: activation diagnostics detect a large preload/readback"
          " phase mismatch")


def test_tmc5160_mscuract_decode_matches_datasheet_bit_order():
    raw = ((-5 & 0x1FF) << 16) | (-248 & 0x1FF)
    cur_a, cur_b = decode_mscuract_5160(raw)
    assert (cur_a, cur_b) == (-5, -248)
    print("PASS: TMC5160 MSCURACT decode keeps CUR_A in bits 24..16"
          " and CUR_B in bits 8..0")


def test_activation_preload_falls_back_to_phase_table():
    preload_a, preload_b, source = activation_preload(512, 0, 0)
    assert source == "mscnt"
    assert preload_a == -248
    assert abs(preload_b) <= 2
    print("PASS: activation preload falls back to MSCNT when live current is unavailable")


def test_phase_direction_override_beats_invert_dir_auto():
    assert resolve_phase_direction(True) == -1
    assert resolve_phase_direction(True, override=1) == 1
    assert resolve_phase_direction(False, override=-1) == -1
    print("PASS: explicit phase direction overrides beat invert_dir auto-detection")


def main():
    tests = [
        test_find_move_start_returns_first_moving_sample,
        test_resume_trim_ignores_replayed_pre_idle_motion,
        test_resume_keeps_one_held_phase_sample,
        test_natural_idle_flush_preserves_held_phase_on_resume,
        test_resume_retry_rewinds_to_recent_idle_history,
        test_long_idle_flush_rechecks_recent_tail_before_declaring_standstill,
        test_natural_idle_does_not_need_host_hold_padding,
        test_pre_move_idle_keeps_clock_reset_armed,
        test_natural_idle_keeps_mcu_hold_active_without_reset,
        test_query_response_router_requires_exact_oid_match,
        test_explicit_idle_boundary_requires_new_clock_anchor,
        test_idle_boundary_forces_reset_before_resume,
        test_diagnostic_boundary_forces_reset_before_resume,
        test_reset_clock_masks_motor_until_first_segment_is_loaded,
        test_reset_clock_stops_idle_group_and_honors_new_waketime,
        test_stop_requires_explicit_reset_before_queueing_again,
        test_global_cadence_starts_future_motor_too_early,
        test_global_cadence_shifts_resumed_motor_to_wrong_tick,
        test_homing_move_end_does_not_resume_inside_home_rails,
        test_nested_homing_cancels_early_resume,
        test_only_latest_manual_resume_can_fire,
        test_probe_window_defers_resume_until_calibration_finishes,
        test_manual_delayed_resume_retries_until_homing_clears,
        test_buffered_motion_defers_resume_until_printer_is_idle,
        test_priming_buffer_does_not_block_resume,
        test_suspend_resyncs_before_stepdir_unmasks,
        test_nonfinite_endpoints_force_idle_boundary,
        test_external_beacon_flow_suppresses_homing_auto_resume,
        test_phase_active_policy_keeps_run_current_and_delays_powerdown,
        test_phase_active_policy_does_not_mutate_cached_driver_settings,
        test_activation_preload_prefers_live_current_vector,
        test_phase_from_currents_roundtrips_logged_activation_vectors,
        test_phase_position_to_mcu_phase_matches_fixed_point_wrap,
        test_continuity_snapshot_sees_small_idle_restart_jump,
        test_continuity_snapshot_sees_large_preload_readback_mismatch,
        test_tmc5160_mscuract_decode_matches_datasheet_bit_order,
        test_activation_preload_falls_back_to_phase_table,
        test_phase_direction_override_beats_invert_dir_auto,
    ]
    for test in tests:
        test()
    print("All phase scheduler tests passed")


if __name__ == "__main__":
    main()
