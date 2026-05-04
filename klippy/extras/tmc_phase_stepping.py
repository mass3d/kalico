# TMC5160 Phase Stepping support
#
# Copyright (C) 2026  klipper contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
from .. import chelper

FIXED_POINT_SCALE = 65536.  # 16.16 fixed-point
PHASE_ACTIVE_TPOWERDOWN = 255

# Host-side version marker — bumped on each tmc_phase_stepping.py change
# so deploys can be verified via klippy.log.  MCU firmware version is
# tracked separately via the PHASE_STEPPER_VER constant in
# src/tmc_phase_stepper.c (queryable through MCU_constants).
HOST_PHASE_STEPPER_VER = "v9-pair-generator-sync"

# Maximum number of position samples per flush cycle.
# At the default 10kHz update rate, 32768 samples cover ~3.28s. Real AWD logs
# showed the first resumed flush arriving ~1.25s after motion started; with the
# default 0.5s idle lookback, 16384 samples (~1.64s) was still too short and
# the host queued the move from mid-trajectory. 32768 leaves headroom for that
# long first-flush latency while keeping the whole resume on the host side.
MAX_PHASE_SAMPLES = 32768
# Maximum number of compressed segments emitted per motor per flush.
# Each emitted segment consumes one node in the MCU's shared move_alloc pool
# (size 1024, also used by stepcompress and other queued-move users). With
# four phase-stepped motors plus the rest of the printer, the per-motor cap
# must keep the worst-case in-flight total well below 1024. With healthy
# compression, normal motion produces only a handful of segments per flush;
# this cap is just a safety bound against pathological compressor behavior.
MAX_PHASE_SEGMENTS = 128


def _mask_shift(mask):
    return (mask & -mask).bit_length() - 1


def _compose_tmc_field(fields, field_name, field_value,
                       reg_value=None, reg_name=None):
    if reg_name is None:
        reg_name = fields.lookup_register(field_name)
    if reg_value is None:
        reg_value = fields.registers.get(reg_name, 0)
    mask = fields.all_fields[reg_name][field_name]
    return ((reg_value & ~mask)
            | ((field_value << _mask_shift(mask)) & mask))


def _sign9(val):
    val &= 0x1FF
    return val - 0x200 if val & 0x100 else val


def _decode_xdirect(raw):
    return _sign9(raw), _sign9(raw >> 16)


def _decode_mscuract(raw):
    # MSCURACT raw layout per datasheet: bits 0..8 = CUR_A, bits 16..24 = CUR_B.
    # We return them swapped so the (cur_a, cur_b) tuple matches what we write
    # to XDIRECT below — XDIRECT in direct_mode swaps coils relative to MSCURACT
    # (Prusa documents this as "TMC in Xdirect mode swaps coils"). The labels
    # here track XDIRECT's coil mapping, not MSCURACT's bit layout.
    return _sign9(raw >> 16), _sign9(raw)


def _phase_table_currents(mscnt):
    angle = mscnt * 2.0 * math.pi / 1024.0
    # TMC5160 MSCNT indexes phase B on the sine wave and phase A on the
    # cosine wave. XTARGET uses A in bits 8..0 and B in bits 24..16.
    return (int(round(248.0 * math.cos(angle))),
            int(round(248.0 * math.sin(angle))))


def _phase_from_currents(cur_a, cur_b):
    if not cur_a and not cur_b:
        return None
    phase = int(round(math.atan2(cur_b, cur_a)
                      * 1024.0 / (2.0 * math.pi)))
    return phase & 0x3FF


def _phase_delta(from_phase, to_phase):
    if from_phase is None or to_phase is None:
        return None
    delta = (to_phase - from_phase) & 0x3FF
    if delta >= 0x200:
        delta -= 0x400
    return delta


def _phase_position_to_mcu_phase(position):
    if position is None or not math.isfinite(position):
        return None
    reduced = math.fmod(position, 1024.0)
    fixed = int(reduced * FIXED_POINT_SCALE)
    return (fixed >> 16) & 0x3FF


def _phase_from_mcu_start_position(start_position):
    return (int(start_position) >> 16) & 0x3FF


def _vector_peak_delta(cur_a, cur_b, next_a, next_b):
    return max(abs(cur_a - next_a), abs(cur_b - next_b))


def _activation_preload_currents(mscnt, mscuract_raw):
    cur_a, cur_b = _decode_mscuract(mscuract_raw)
    # Prefer the live current vector when available so direct_mode takes over
    # from the actual held electrical state, not an inferred ideal sine phase.
    if cur_a or cur_b:
        return cur_a, cur_b, "mscuract"
    cur_a, cur_b = _phase_table_currents(mscnt)
    return cur_a, cur_b, "mscnt"

class MCU_phase_stepper:
    """Manages phase-stepping output for a single TMC5160 axis.
    Replaces step/dir with SPI-based XDIRECT writes."""
    def __init__(self, config, stepper, tmc_name):
        self.printer = config.get_printer()
        self.stepper = stepper
        self.tmc_obj = None  # Set later via set_tmc_obj()
        self.tmc_name = tmc_name
        self.name = config.get_name()
        self.mcu = stepper.get_mcu()
        self.oid = self.mcu.create_oid()
        self.update_rate = config.getfloat('phase_update_rate', 25000.,
                                           above=1000., maxval=50000.)
        self.update_interval = 1. / self.update_rate
        self.phase_bus = config.getint('phase_bus', 0, minval=0, maxval=3)
        self.active_tpowerdown = config.getint(
            'phase_active_tpowerdown', PHASE_ACTIVE_TPOWERDOWN,
            minval=0, maxval=255)
        self.disable_faststandstill = config.getboolean(
            'phase_disable_faststandstill', True)
        self.max_error = config.getfloat('phase_compress_error', 0.25,
                                         above=0.)
        self._phase_direction_override = config.getint(
            'phase_direction_override', 0, minval=-1, maxval=1)
        self._phase_stepping_active = False
        self._needs_clock_reset = True
        # Bit-exact mirror of the MCU's last-emitted phase position (16.16
        # int32).  Every compressed segment's start_position is FORCED to
        # equal this anchor — that's what makes the resume-from-idle click
        # impossible by construction.  Updated locally using mirror_advance
        # math identical to the MCU loop, so flush-to-flush continuity is
        # bit-exact.
        self._mcu_anchor_pos = 0
        self._mcu_anchor_clock = 0
        # AWD pair partnership (set by TMCPhaseStepping._resolve_pairs).
        self._pair_partner = None
        self._pair_reset_consumed_clock = None
        self._invalid_flush_count = 0
        self._emit_log_count = 0
        self._last_emit_motion = False
        self._faulted = False
        self._fault_handler = None
        self._phase_step_dist = None
        self._phase_direction = 1.0
        self._phase_direction_source = 'invert_dir'
        self._dir_inverted = False
        self._phase_offset = 0.0
        self._last_commanded_position = None
        self._activation_pending = False
        self._trace_events = []
        self._activation_vector = {}
        self._last_continuity = {}
        # Saved TMCCommandHelper methods replaced while phase stepping is
        # active. The cmdhelper isn't stored on the TMC5160 instance — we
        # find it via the stepper_enable callback list during activate().
        # Restored in deactivate().
        self._tmc_cmd_helper = None
        self._saved_tmc_do_disable = None
        self._saved_tmc_do_enable = None
        # FFI handles
        ffi_main, ffi_lib = chelper.get_ffi()
        self._ffi_main = ffi_main
        self._ffi_lib = ffi_lib
        self._phase_generator = ffi_main.gc(
            ffi_lib.phase_generator_alloc(), ffi_lib.free)
        self._phase_compressor = ffi_main.gc(
            ffi_lib.phase_compressor_alloc(), ffi_lib.free)
        ffi_lib.phase_compressor_set_max_error(
            self._phase_compressor, self.max_error)
        # Allocate sample and segment buffers
        self._samples = ffi_main.new('struct phase_sample[]', MAX_PHASE_SAMPLES)
        self._positions = ffi_main.new('double[]', MAX_PHASE_SAMPLES)
        self._segments = ffi_main.new('struct phase_compressed_move[]',
                                      MAX_PHASE_SEGMENTS)
        self._mcu_moves = ffi_main.new('struct phase_mcu_move[]',
                                       MAX_PHASE_SEGMENTS)
        # MCU command handles (filled during _build_config)
        self._queue_cmd = self._reset_cmd = self._set_current_cmd = None
        self._reset_group_cmd = None
        self._stop_cmd = None
        self.mcu.register_config_callback(self._build_config)
        self.printer.register_event_handler('klippy:connect',
                                            self._handle_connect)
    def _build_config(self):
        # SPI oid is resolved at build time from the TMC driver
        # The TMC module must already have its SPI configured by now
        tmc_obj = self.printer.lookup_object(self.tmc_name, None)
        if tmc_obj is None:
            logging.warning("Phase stepping: TMC object %s not found"
                            " during config", self.tmc_name)
            return
        tmc_spi = tmc_obj.mcu_tmc.tmc_spi
        spi_oid = tmc_spi.spi.get_oid()
        # Configure the MCU-side phase stepper
        self.mcu.add_config_cmd(
            "config_tmc_phase_stepper oid=%d spi_oid=%d mode=%d"
            % (self.oid, spi_oid, 0))  # mode=0 = DIRECT_CURRENT
        self._queue_cmd = self.mcu.lookup_command(
            "queue_phase_move oid=%c interval=%u start_pos=%i"
            " velocity=%i accel=%i count=%hu")
        self._reset_cmd = self.mcu.lookup_command(
            "reset_phase_clock oid=%c clock=%u")
        self._reset_group_cmd = self.mcu.lookup_command(
            "reset_phase_clock_group oids=%*s clock=%u")
        self._set_current_cmd = self.mcu.lookup_command(
            "set_phase_stepper_current oid=%c scale=%hu")
        self._stop_cmd = self.mcu.lookup_command(
            "stop_phase_stepper oid=%c")
        self._status_cmd = self.mcu.lookup_query_command(
            "get_phase_stepper_status oid=%c",
            "phase_stepper_status oid=%c event_count=%u write_count=%u"
            " skip_count=%u last_phase=%hu position=%i",
            oid=self.oid)
        # Tell the MCU which bus_group this motor belongs to.  Default is
        # buses[0]; multi-bus setups (future custom board) override per
        # stepper.
        if self.phase_bus != 0:
            self.mcu.add_config_cmd(
                "config_phase_bus oid=%d bus_index=%d"
                % (self.oid, self.phase_bus))
    def get_mcu_status(self):
        """Query MCU-side ISR diagnostic counters."""
        if self._status_cmd is None:
            return None
        return self._status_cmd.send([self.oid])
    def _resolve_phase_direction(self, invert_dir):
        if self._phase_direction_override:
            direction = 1.0 if self._phase_direction_override > 0 else -1.0
            return direction, 'override'
        direction = 1.0 if invert_dir else -1.0
        return direction, 'invert_dir'
    def _phase_vector_from_position(self, phase_position):
        phase = _phase_position_to_mcu_phase(phase_position)
        if phase is None:
            return None, None, None
        cur_a, cur_b = _phase_table_currents(phase)
        return phase, cur_a, cur_b
    def _record_continuity(self, origin, from_phase, from_a, from_b, to_phase):
        if from_phase is None or to_phase is None:
            self._last_continuity = {}
            return
        if from_a is None or from_b is None:
            from_a, from_b = _phase_table_currents(from_phase)
        to_a, to_b = _phase_table_currents(to_phase)
        delta_phase = _phase_delta(from_phase, to_phase)
        delta_peak = _vector_peak_delta(from_a, from_b, to_a, to_b)
        entry = {
            'origin': origin,
            'from_phase': from_phase,
            'from_a': from_a,
            'from_b': from_b,
            'to_phase': to_phase,
            'to_a': to_a,
            'to_b': to_b,
            'delta_phase': delta_phase,
            'delta_peak': delta_peak,
        }
        self._last_continuity = entry
        self._trace_event('continuity', **entry)
        logging.info(
            "Phase stepping %s: %s continuity from_phase=%d to_phase=%d"
            " delta_phase=%+d delta_peak=%d"
            " from_a=%d from_b=%d to_a=%d to_b=%d",
            self.name, origin, from_phase, to_phase, delta_phase,
            delta_peak, from_a, from_b, to_a, to_b)
    def set_tmc_obj(self, tmc_obj):
        self.tmc_obj = tmc_obj
    def _handle_connect(self):
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.register_step_generator(self.generate_and_queue)
        # Log MCU firmware version so it's obvious whether a fresh flash
        # carries the latest phase-stepping fixes.  If "missing" appears,
        # the MCU was NOT reflashed with v7+ firmware.
        try:
            ver = self.mcu.get_constants().get(
                "PHASE_STEPPER_VER", "missing")
            logging.info(
                "Phase stepping %s: HOST_PHASE_STEPPER_VER=%s"
                " MCU firmware PHASE_STEPPER_VER=%s",
                self.name, HOST_PHASE_STEPPER_VER, ver)
        except Exception:
            pass
    def set_fault_handler(self, cb):
        self._fault_handler = cb
    def _trace_event(self, kind, **fields):
        entry = {'kind': kind}
        entry.update(fields)
        self._trace_events.append(entry)
        if len(self._trace_events) > 32:
            del self._trace_events[0]
    def _report_fault(self, reason):
        if self._faulted:
            return
        self._faulted = True
        self._trace_event('fault', reason=reason)
        logging.warning("Phase stepping fault on %s: %s", self.name, reason)
        if self._fault_handler is not None:
            self._fault_handler(self.name, reason)
    def _apply_phase_stepping_registers(self):
        """Write the TMC register block needed for phase stepping mode.
        Used both at initial activate() and from the patched _do_enable
        path so a post-M84 motor_enable re-establishes direct_mode.
        Returns (mscnt, mscuract_raw, cur_a, cur_b, preload_source) so the
        caller can log/trace the preload state."""
        mcu_tmc = self.tmc_obj.mcu_tmc
        fields = mcu_tmc.fields
        cached_ihr = fields.registers.get("IHOLD_IRUN", 0)
        for reg_name in ["GLOBALSCALER", "CHOPCONF"]:
            val = fields.registers.get(reg_name, None)
            if val is not None:
                mcu_tmc.set_register(reg_name, val)
        # Set IHOLD = IRUN (direct_mode scales current by IHOLD per datasheet)
        irun = fields.get_field("irun", cached_ihr, "IHOLD_IRUN")
        ihr_val = _compose_tmc_field(fields, "ihold", irun, cached_ihr,
                                     "IHOLD_IRUN")
        mcu_tmc.set_register("IHOLD_IRUN", ihr_val)
        # Keep the driver in its run-current policy while phase stepping is
        # active. The TMC5160 datasheet's direct_mode note says current scaling
        # remains active and depends on STEP impulses; phase stepping suppresses
        # STEP/DIR, so we extend the standstill timeout and disable fast
        # standstill detection while active instead of relying on defaults.
        mcu_tmc.set_register("TPOWERDOWN", self.active_tpowerdown)
        # Read the live phase/current state before enabling direct_mode so the
        # handoff preserves the actual electrical hold, not an inferred one.
        mscnt = mcu_tmc.get_register("MSCNT") & 0x3FF
        mscuract_raw = mcu_tmc.get_register("MSCURACT")
        cur_a, cur_b, preload_source = _activation_preload_currents(
            mscnt, mscuract_raw)
        # GCONF: direct_mode=1, SpreadCycle (en_pwm_mode=0).  StealthChop's
        # auto-regulation can't track at 10kHz and falls back to MSLUT.
        gconf_val = self._saved_gconf
        gconf_val |= (1 << 16)   # direct_mode
        gconf_val &= ~(1 << 2)   # SpreadCycle (clear en_pwm_mode)
        if self.disable_faststandstill:
            gconf_val &= ~(1 << 1)
        mcu_tmc.set_register("GCONF", gconf_val)
        # Immediately pre-load XDIRECT with correct currents so the motor
        # doesn't lose torque between direct_mode enable and first ISR write.
        # Register 0x2D = XDIRECT in direct_mode: coil_B[31:16] | coil_A[15:0]
        cur_b_u16 = cur_b & 0xFFFF
        cur_a_u16 = cur_a & 0xFFFF
        mcu_tmc.set_register("XTARGET", (cur_b_u16 << 16) | cur_a_u16)
        # Set current scale (256 = no scaling, table peak 248 passes through)
        self._set_current_cmd.send([self.oid, 256])
        return mscnt, mscuract_raw, cur_a, cur_b, preload_source
    def _ramp_xdirect_to_mscnt(self, print_time):
        """Ramp XDIRECT one MSCNT microstep at a time toward the TMC's
        current MSCNT phase. When deactivate() then clears GCONF.direct_mode,
        the TMC reverts to MSLUT-driven currents based on MSCNT (which has
        been frozen at activation time). Without alignment, the held XDIRECT
        phase and MSCNT phase differ -> motor jumps electrically -> click.
        Ramp uses fire-and-forget spi_send with minclock spacing so the
        host returns in O(host scheduling) instead of waiting for n_steps
        SPI verifications."""
        mcu_tmc = self.tmc_obj.mcu_tmc
        try:
            mscnt = mcu_tmc.get_register("MSCNT") & 0x3FF
            xdirect_raw = mcu_tmc.get_register("XTARGET")
        except Exception:
            logging.exception(
                "Phase stepping %s: deactivate ramp register read failed",
                self.name)
            return print_time
        xd_a, xd_b = _decode_xdirect(xdirect_raw)
        cur_phase = _phase_from_currents(xd_a, xd_b)
        if cur_phase is None:
            self._trace_event('deactivate_ramp_skip', mscnt=mscnt,
                              reason='no_xdirect_phase')
            return print_time
        delta = _phase_delta(cur_phase, mscnt)
        if delta is None or delta == 0:
            self._trace_event('deactivate_ramp_skip', mscnt=mscnt,
                              cur_phase=cur_phase, reason='aligned')
            return print_time
        tmc_spi = mcu_tmc.tmc_spi
        spi = tmc_spi.spi
        chain_pos = mcu_tmc.chain_pos
        reg = mcu_tmc.name_to_reg["XTARGET"]
        step = 1 if delta > 0 else -1
        n_steps = abs(delta)
        ramp_dt = self.update_interval
        phase = cur_phase
        ramp_pt = print_time
        for _ in range(n_steps):
            phase = (phase + step) & 0x3FF
            cur_a, cur_b = _phase_table_currents(phase)
            val = ((cur_b & 0xFFFF) << 16) | (cur_a & 0xFFFF)
            data = [
                (reg | 0x80) & 0xFF,
                (val >> 24) & 0xFF,
                (val >> 16) & 0xFF,
                (val >> 8) & 0xFF,
                val & 0xFF,
            ]
            cmd = tmc_spi._build_cmd(data, chain_pos)
            ramp_pt += ramp_dt
            minclock = spi.get_mcu().print_time_to_clock(ramp_pt)
            spi.spi_send(cmd, minclock=minclock)
        self._trace_event('deactivate_ramp', mscnt=mscnt,
                          start_phase=cur_phase, end_phase=mscnt,
                          n_steps=n_steps, ramp_end=ramp_pt)
        logging.info(
            "Phase stepping %s: deactivate ramp from phase=%d to mscnt=%d"
            " in %d steps (%.3fms), ramp_end_print_time=%.6f",
            self.name, cur_phase, mscnt, n_steps,
            n_steps * ramp_dt * 1000.0, ramp_pt)
        return ramp_pt
    def _find_tmc_cmd_helper(self):
        """The TMC5160 class instantiates TMCCommandHelper as a local
        variable and discards the reference, so we can't reach it
        directly via self.tmc_obj. The cmdhelper does register itself
        with the stepper's enable_line via register_state_callback, so
        we walk that callback list looking for a bound method whose
        owner has both _do_disable and _do_enable (the TMC enable hooks)."""
        try:
            stepper_enable = self.printer.lookup_object('stepper_enable')
            enable_line = stepper_enable.lookup_enable(self.stepper.get_name())
        except Exception:
            return None
        for cb in getattr(enable_line, 'callbacks', []):
            owner = getattr(cb, '__self__', None)
            if owner is None:
                continue
            if (callable(getattr(owner, '_do_disable', None))
                    and callable(getattr(owner, '_do_enable', None))):
                return owner
        return None
    def _phase_aware_do_disable(self, print_time):
        # Suppress chopper-off (toff=0) while phase stepping is active so M84
        # / idle_timeout doesn't drop coil current and click. The original
        # _do_disable is restored in deactivate().
        if self._phase_stepping_active:
            return
        if self._saved_tmc_do_disable is not None:
            self._saved_tmc_do_disable(print_time)
    def _phase_aware_do_enable(self, print_time):
        # If something invokes _do_enable while phase stepping is still
        # active (rare — active_callbacks are bypassed), re-establish our
        # GCONF/CHOPCONF/XDIRECT state instead of letting _init_registers()
        # clobber direct_mode.
        if self._phase_stepping_active:
            try:
                self._apply_phase_stepping_registers()
            except Exception:
                logging.exception(
                    "Phase stepping %s: failed to re-apply registers in"
                    " patched _do_enable", self.name)
            return
        if self._saved_tmc_do_enable is not None:
            self._saved_tmc_do_enable(print_time)
    def activate(self):
        """Enable XDIRECT mode on TMC5160 and start phase stepping."""
        if self._phase_stepping_active:
            return
        if self._queue_cmd is None or self.tmc_obj is None:
            logging.warning("Phase stepping not available for %s"
                            " (MCU commands not configured)", self.name)
            return
        mcu_tmc = self.tmc_obj.mcu_tmc
        # Re-initialize key TMC registers from config cache.
        # TMC5160 may have reset during homing (undervoltage from back-EMF).
        fields = mcu_tmc.fields
        # Get configured microstep resolution for position scaling.
        # Klipper step_dist maps to configured microsteps (e.g. 16/full step),
        # but MSCNT uses 256 native microsteps per full step.  We need the
        # ratio to convert positions to MSCNT units.
        mres = fields.get_field("mres")
        microsteps = 256 >> mres  # mres=4 -> 16, mres=0 -> 256
        # Save original GCONF so we can restore it on deactivate and so the
        # helper has a stable base to OR direct_mode onto.
        self._saved_gconf = mcu_tmc.get_register("GCONF")
        # Apply our register block (CHOPCONF/IHOLD_IRUN/TPOWERDOWN/GCONF/XTARGET)
        mscnt, mscuract_raw, cur_a, cur_b, preload_source = (
            self._apply_phase_stepping_registers())
        live_cur_a, live_cur_b = _decode_mscuract(mscuract_raw)
        chop_rb = mcu_tmc.get_register("CHOPCONF")
        logging.info("Phase stepping %s: reinit CHOPCONF=0x%08x",
                     self.name, chop_rb)
        cur_b_u16 = cur_b & 0xFFFF
        cur_a_u16 = cur_a & 0xFFFF
        self._needs_clock_reset = True
        self._invalid_flush_count = 0
        self._emit_log_count = 0
        self._last_emit_motion = False
        self._faulted = False
        self._pair_reset_consumed_clock = None
        # Configure the phase generator with the stepper's kinematics.
        # Scale step_dist so positions are in MSCNT units (256 per full step)
        # rather than klipper microstep units (e.g. 16 per full step).
        # pos / phase_step_dist = pos / step_dist * (256 / microsteps)
        sk = self.stepper.get_stepper_kinematics()
        step_dist = self.stepper.get_step_dist()
        phase_step_dist = step_dist * microsteps / 256.0
        self._phase_step_dist = phase_step_dist
        self._ffi_lib.phase_generator_config(
            self._phase_generator, sk, phase_step_dist, self.update_interval)
        # Scale compression error to MSCNT units.  The config value is in
        # klipper microstep units; multiply by 256/microsteps to maintain
        # the same physical error tolerance in MSCNT space.
        mscnt_max_error = self.max_error * (256.0 / microsteps)
        self._ffi_lib.phase_compressor_set_max_error(
            self._phase_compressor, mscnt_max_error)
        # Detect direction: inverted dir_pin compensates for swapped coil
        # pairs. In direct_mode we must reverse the phase direction to match,
        # since there's no DIR signal to invert.
        invert_dir = self.stepper.get_dir_inverted()[0]
        direction, dir_source = self._resolve_phase_direction(invert_dir)
        self._dir_inverted = invert_dir
        self._phase_direction = direction
        self._phase_direction_source = dir_source
        self._ffi_lib.phase_generator_set_direction(
            self._phase_generator, direction)
        # Calibrate phase offset: difference between TMC's internal phase
        # (MSCNT) and klipper's stepper position converted to MSCNT units.
        stepper_pos_mscnt = (self.stepper.get_commanded_position()
                             / step_dist * (256.0 / microsteps))
        klipper_phase = int(stepper_pos_mscnt * direction) & 0x3FF
        # Anchor phase = the phase the preloaded XDIRECT actually represents.
        # Prefer the live MSCURACT-derived current vector; fall back to MSCNT
        # if the live read yielded no information.
        anchor_phase = _phase_from_currents(cur_a, cur_b)
        if anchor_phase is None:
            anchor_phase = mscnt
        # phase_offset aligns the trapq's first sample (in microsteps,
        # post-direction) so that sample[0] mod 1024 == anchor_phase. With
        # this alignment the compressor's first segment fits cleanly at the
        # anchor — no force-emit-single-tick fallback, no phase glitch.
        phase_offset = float((anchor_phase - klipper_phase) & 0x3FF)
        self._phase_offset = phase_offset
        self._ffi_lib.phase_generator_set_offset(
            self._phase_generator, phase_offset)
        self._last_commanded_position = self.stepper.get_commanded_position()
        # 16.16 fixed-point representation of the held phase — the host-mirror
        # of the MCU's expected next-tick phase position.  Every subsequent
        # compressed segment's start_position will be FORCED to equal this
        # value, eliminating the phase discontinuity that produced the
        # resume-from-idle click.
        self._mcu_anchor_pos = (int(anchor_phase) << 16) & 0x03FFFFFF
        self._mcu_anchor_clock = 0  # set on first generate_and_queue
        self._trace_event('activate_begin',
                          mscnt=mscnt,
                          mscuract_a=live_cur_a,
                          mscuract_b=live_cur_b,
                          cmd_pos=self.stepper.get_commanded_position(),
                          klipper_phase=klipper_phase)
        # Verify direct_mode was actually set
        gconf_readback = mcu_tmc.get_register("GCONF")
        direct_mode_set = bool(gconf_readback & (1 << 16))
        gconf_shaft = fields.get_field("shaft", gconf_readback, "GCONF")
        # Read back XDIRECT register to verify our preload write took effect
        xdirect_rb = mcu_tmc.get_register("XTARGET")
        xd_a, xd_b = _decode_xdirect(xdirect_rb)
        preload_phase = _phase_from_currents(cur_a, cur_b)
        xdirect_phase = _phase_from_currents(xd_a, xd_b)
        xdirect_delta = _phase_delta(preload_phase, xdirect_phase)
        self._activation_vector = {
            'phase': preload_phase,
            'a': cur_a,
            'b': cur_b,
            'source': preload_source,
            'readback_phase': xdirect_phase,
            'readback_a': xd_a,
            'readback_b': xd_b,
            'readback_delta': xdirect_delta,
            'gconf_shaft': gconf_shaft,
        }
        self._last_continuity = {}
        self._trace_event('activate_preload',
                          preload_a=cur_a, preload_b=cur_b,
                          preload_phase=preload_phase,
                          readback_a=xd_a, readback_b=xd_b,
                          readback_phase=xdirect_phase,
                          readback_delta=xdirect_delta,
                          source=preload_source, offset=phase_offset,
                          phase_dir=direction,
                          phase_dir_source=dir_source,
                          invert_dir=invert_dir,
                          gconf_shaft=gconf_shaft)
        logging.info("Phase stepping %s: mscnt=%d klipper_phase=%d offset=%d"
                     " dir=%.0f dir_source=%s invert=%s gconf_shaft=%d"
                     " microsteps=%d"
                     " phase_step_dist=%.6f GCONF=0x%08x direct_mode=%s"
                     " active_tpowerdown=%d"
                     " mscuract_a=%d mscuract_b=%d preload_source=%s"
                     " preload_a=%d preload_b=%d preload_phase=%s"
                     " xdirect_a=%d xdirect_b=%d xdirect_phase=%s"
                     " xdirect_delta=%s"
                     " preload_raw=0x%08x xdirect_raw=0x%08x",
                     self.name, mscnt, klipper_phase, int(phase_offset),
                     direction, dir_source, invert_dir, gconf_shaft,
                     microsteps,
                     phase_step_dist, gconf_readback, direct_mode_set,
                     self.active_tpowerdown,
                     live_cur_a, live_cur_b, preload_source,
                     cur_a, cur_b, preload_phase,
                     xd_a, xd_b, xdirect_phase, xdirect_delta,
                     (cur_b_u16 << 16) | cur_a_u16, xdirect_rb)
        # Initialize last_flush_time to current print time so generator
        # doesn't try to sample from t=0 (boot time)
        current_print_time = self.mcu.estimated_print_time(
            self.printer.get_reactor().monotonic())
        self._ffi_lib.phase_generator_set_time(
            self._phase_generator, current_print_time)
        # Seed the generator's "held position" to the activate-time phase so
        # samples taken in pre-move windows (e.g. between activate and the
        # first G1 after an idle gap) return the held phase via the
        # last_position fallback in phase_generator_generate, instead of
        # extrapolating an unrelated trapq move backwards in time.
        activate_phase_pos = (self.stepper.get_commanded_position()
                              / step_dist * (256.0 / microsteps)
                              * direction + phase_offset)
        self._ffi_lib.phase_generator_set_last_position(
            self._phase_generator, activate_phase_pos)
        logging.info(
            "Phase stepping %s: v3 activate seed_last_position=%.6f"
            " current_print_time=%.6f",
            self.name, activate_phase_pos, current_print_time)
        # Intercept M84/idle_timeout: replace TMCCommandHelper's
        # _do_disable / _do_enable so the chopper stays on while phase
        # stepping is active. The TMC5160 class doesn't store its
        # cmdhelper, so we find it via the stepper_enable callback list
        # (cmdhelper.register_state_callback() registered itself there).
        # Restored in deactivate().
        if self._saved_tmc_do_disable is None:
            cmdhelper = self._find_tmc_cmd_helper()
            if cmdhelper is not None:
                self._tmc_cmd_helper = cmdhelper
                self._saved_tmc_do_disable = cmdhelper._do_disable
                self._saved_tmc_do_enable = cmdhelper._do_enable
                cmdhelper._do_disable = self._phase_aware_do_disable
                cmdhelper._do_enable = self._phase_aware_do_enable
            else:
                logging.warning(
                    "Phase stepping %s: could not find TMCCommandHelper"
                    " for %s; M84 / idle_timeout will turn off the chopper"
                    " and click.", self.name, self.stepper.get_name())
        try:
            stepper_enable = self.printer.lookup_object('stepper_enable')
            enable_line = stepper_enable.lookup_enable(self.stepper.get_name())
            if enable_line.has_dedicated_enable():
                logging.warning(
                    "Phase stepping %s: stepper has a dedicated enable_pin."
                    " M84 / idle_timeout will still de-energize the motor"
                    " via the EN pin and click. Remove enable_pin from the"
                    " [%s] config to use TMC virtual enable instead.",
                    self.name, self.stepper.get_name())
        except Exception:
            pass
        # Suppress regular step/dir generation (TMC5160 ignores it in
        # direct_mode, and the ISR load would crash the MCU)
        self.stepper.set_phase_stepping(True)
        self._phase_stepping_active = True
        self._activation_pending = True
        self._trace_event('activate')
        logging.info("Phase stepping activated for %s", self.name)
    def stop_timer(self):
        """Stop MCU-side phase stepper timer and flush its move queue."""
        if not self._phase_stepping_active:
            return
        self._phase_stepping_active = False
        if self._stop_cmd is not None:
            self._stop_cmd.send([self.oid])
    def _phase_to_commanded(self, phase_pos):
        if self._phase_step_dist is None or not self._phase_direction:
            return None
        return ((phase_pos - self._phase_offset)
                * self._phase_step_dist / self._phase_direction)
    def _advance_mirror(self, start, vel, accel, count):
        """Closed-form position the MCU holds *after* `count` tick-advances.
        Bit-exact match for the MCU's per-tick advance (position += velocity;
        velocity += acceleration); after `count` advances ps->position equals
        start + count*vel + count*(count-1)/2 * accel.  This is the value
        load_next would read as the next segment's start, so chaining via
        this formula keeps the host mirror aligned with the MCU.  Result is
        reduced mod 1024<<16 (= 0x04000000) to fit the compressor's
        reduced-anchor convention."""
        if count <= 0:
            return start & 0x03FFFFFF
        n_choose_2 = (count * (count - 1)) // 2
        raw = start + count * vel + n_choose_2 * accel
        raw &= 0xFFFFFFFF
        if raw & 0x80000000:
            raw -= 0x100000000
        return raw & 0x03FFFFFF
    def _emit_reset_clock(self, clock):
        """Pair-aware reset_phase_clock emit.  When this motor has a partner
        (AWD), bundle both OIDs into a single MCU command so the partners'
        waketimes are anchored atomically inside one irq_disable() window —
        no host-ordering race can desync them."""
        partner = self._pair_partner
        if partner is not None and self._reset_group_cmd is not None:
            if partner._pair_reset_consumed_clock != clock:
                oids_bytes = bytearray([self.oid, partner.oid])
                self._reset_group_cmd.send(
                    [bytes(oids_bytes), clock])
                self._pair_reset_consumed_clock = clock
                partner._pair_reset_consumed_clock = clock
                partner._needs_clock_reset = False
                partner._mcu_anchor_clock = clock
            else:
                # Partner already covered this clock — skip.
                pass
        else:
            self._reset_cmd.send([self.oid, clock])
    def deactivate(self, print_time=None):
        """Disable XDIRECT mode and return to step/dir."""
        # Stop MCU timer first (if not already stopped)
        self.stop_timer()
        if print_time is None:
            toolhead = self.printer.lookup_object("toolhead")
            print_time = toolhead.get_last_move_time()
        cmd_pos = self._last_commanded_position
        if cmd_pos is None:
            cmd_pos = self.stepper.get_commanded_position()
        self._trace_event('deactivate_begin', print_time=print_time,
                          cmd_pos=cmd_pos)
        self.stepper.sync_commanded_position(print_time, cmd_pos)
        self._activation_pending = False
        self._pair_reset_consumed_clock = None
        # Re-enable regular step/dir generation
        self.stepper.set_phase_stepping(False)
        # Ramp XDIRECT toward the TMC's frozen MSCNT phase before clearing
        # direct_mode. Without alignment, clearing direct_mode reverts the
        # driver to MSLUT-driven currents (CUR_A=sin(MSCNT), CUR_B=cos(MSCNT))
        # and the held electrical phase snaps to MSCNT -> click.
        ramp_end_pt = self._ramp_xdirect_to_mscnt(print_time)
        # Restore original GCONF (clearing direct_mode) and IHOLD_IRUN.
        # The GCONF write must happen AFTER the ramp commands; pass print_time
        # so it's scheduled at the end of the ramp.
        mcu_tmc = self.tmc_obj.mcu_tmc
        if hasattr(self, '_saved_gconf'):
            mcu_tmc.set_register("GCONF", self._saved_gconf, ramp_end_pt)
        else:
            gconf_val = mcu_tmc.get_register("GCONF")
            gconf_val &= ~(1 << 16)
            mcu_tmc.set_register("GCONF", gconf_val, ramp_end_pt)
        # Restore IHOLD_IRUN from config cache (not from TMC readback,
        # which may be 0 if the TMC reset during phase stepping)
        fields = mcu_tmc.fields
        ihr_cached = fields.registers.get("IHOLD_IRUN", None)
        if ihr_cached is not None:
            mcu_tmc.set_register("IHOLD_IRUN", ihr_cached)
        tpowerdown_cached = fields.registers.get("TPOWERDOWN", None)
        if tpowerdown_cached is not None:
            mcu_tmc.set_register("TPOWERDOWN", tpowerdown_cached)
        # Restore TMCCommandHelper methods replaced during activate().
        if (self._tmc_cmd_helper is not None
                and self._saved_tmc_do_disable is not None):
            self._tmc_cmd_helper._do_disable = self._saved_tmc_do_disable
            self._saved_tmc_do_disable = None
        if (self._tmc_cmd_helper is not None
                and self._saved_tmc_do_enable is not None):
            self._tmc_cmd_helper._do_enable = self._saved_tmc_do_enable
            self._saved_tmc_do_enable = None
        self._tmc_cmd_helper = None
        self._trace_event('deactivate_restore',
                          restored_gconf=hasattr(self, '_saved_gconf'),
                          restored_tpowerdown=tpowerdown_cached is not None,
                          restored_ihold=ihr_cached is not None)
        logging.info("Phase stepping deactivated for %s", self.name)
    def generate_and_queue(self, flush_time):
        """Called from the flush callback.  Mirror-anchored continuous
        emission: every flush produces a contiguous segment chain whose
        first segment starts EXACTLY at the host-mirror of the MCU's last
        emitted phase.  No idle/resume special case — when the kinematic
        position doesn't change, the compressor produces a single segment
        with vel=0, accel=0, count=N which the MCU writes as the same
        XDIRECT bytes for N ticks.  No phase discontinuity at any segment
        boundary, ever."""
        if not self._phase_stepping_active:
            return
        ffi_lib = self._ffi_lib
        if self._needs_clock_reset:
            # Re-anchor the generator just before the first emission so
            # start_clock lands in the future.  Without this, an idle gap
            # between activate and the first move leaves last_flush_time
            # stale and sched_add_timer on the MCU trips "Timer too close".
            #
            # Compute everything in MCU clock units so the math doesn't
            # depend on the per-MCU print_time / adj_offset relationship —
            # then convert the chosen clock back to print_time for the
            # generator's last_flush_time.  The host->MCU buffer is the
            # raw "this far ahead of MCU's current clock" interval; we
            # don't have to reason about secondary-MCU offsets at all.
            reactor = self.printer.get_reactor()
            now_pt = self.mcu.estimated_print_time(reactor.monotonic())
            now_clock = self.mcu.print_time_to_clock(now_pt)
            flush_clock = self.mcu.print_time_to_clock(flush_time)
            # 0.050s matches STEPCOMPRESS_FLUSH_TIME in toolhead.py — the
            # standard "send commands this far ahead of wall clock" buffer
            # that absorbs host->MCU command latency.
            min_buffer = 0.050
            buffer_ticks = self.mcu.seconds_to_clock(min_buffer)
            safe_clock = now_clock + buffer_ticks
            safe_start = self.mcu.clock_to_print_time(safe_clock)
            logging.info(
                "Phase stepping %s: first_emit_anchor now_pt=%.6f"
                " now_clock=%d flush_time=%.6f flush_clock=%d"
                " buffer_ticks=%d safe_clock=%d safe_start=%.6f",
                self.name, now_pt, now_clock, flush_time, flush_clock,
                buffer_ticks, safe_clock, safe_start)
            self._trace_event(
                'first_emit_anchor', now_pt=now_pt, now_clock=now_clock,
                flush_time=flush_time, flush_clock=flush_clock,
                buffer_ticks=buffer_ticks, safe_clock=safe_clock,
                safe_start=safe_start)
            if safe_start >= flush_time:
                # Toolhead's flush_time is not yet far enough ahead of now;
                # wait for the next call.
                logging.info(
                    "Phase stepping %s: first_emit_defer"
                    " safe_start=%.6f flush_time=%.6f",
                    self.name, safe_start, flush_time)
                self._trace_event('first_emit_defer',
                                  safe_start=safe_start, flush_time=flush_time)
                return
            ffi_lib.phase_generator_set_time(
                self._phase_generator, safe_start)
            # Pair atomicity: when this motor emits its reset_phase_clock,
            # `_emit_reset_clock` will clear the partner's _needs_clock_reset
            # so the partner's generate_and_queue skips this first-emit
            # block.  Advance the partner's generator last_flush_time to
            # the SAME safe_start so both motors sample from the same point
            # in print_time space.  Otherwise the partner samples from its
            # activate-time seed (way behind), produces only held-position
            # samples, and emits hold segments while the primary emits
            # real motion — the partner motor "stays put" while the primary
            # moves.
            if self._pair_partner is not None:
                ffi_lib.phase_generator_set_time(
                    self._pair_partner._phase_generator, safe_start)
        # Sample positions for [last_flush_time .. flush_time].
        n = ffi_lib.phase_generator_generate(
            self._phase_generator, flush_time, self._samples,
            MAX_PHASE_SAMPLES)
        if n <= 0:
            if self._emit_log_count < 6:
                logging.info(
                    "Phase stepping %s: emit n=0 flush_time=%.6f"
                    " (no new samples since last_flush_time)",
                    self.name, flush_time)
                self._emit_log_count += 1
            return
        # Extract into the float array; checks for NaN/inf.
        ret = ffi_lib.phase_generator_extract_positions(
            self._samples, n, self._positions)
        if ret < 0:
            # Non-finite samples from trapq.  Don't queue this flush; the
            # next one should recover.  Fault if it persists.
            self._invalid_flush_count += 1
            self._trace_event('invalid_flush', flush=flush_time,
                              count=self._invalid_flush_count)
            if self._invalid_flush_count >= 4:
                self._report_fault("repeated non-finite phase samples")
            return
        self._invalid_flush_count = 0
        # On the very first flush after activate, the mirror anchor was set
        # to the preloaded XDIRECT phase but the anchor clock is 0 — set it
        # now to the first sample's clock so reset_phase_clock points at the
        # right tick.
        start_time = self._samples[0].time
        start_clock = self.mcu.print_time_to_clock(start_time)
        if self._mcu_anchor_clock == 0:
            self._mcu_anchor_clock = start_clock
        # Compress with the mirror anchor.  The compressor forces the first
        # segment's start_position to equal _mcu_anchor_pos and chains all
        # subsequent segments via the same int32 mirror_advance arithmetic
        # the MCU uses — guaranteeing bit-exact continuous phase emission.
        num_mcu = ffi_lib.phase_compressor_compress_anchored(
            self._phase_compressor, self._positions, n,
            self._mcu_anchor_pos, self._mcu_moves, MAX_PHASE_SEGMENTS)
        if num_mcu <= 0:
            logging.info(
                "Phase stepping %s: emit num_mcu=0 n=%d flush_time=%.6f"
                " positions[0]=%.4f positions[-1]=%.4f anchor=0x%08x",
                self.name, n, flush_time, self._positions[0],
                self._positions[n - 1], self._mcu_anchor_pos)
            return
        # Issue reset_phase_clock on the very first emission post-activate.
        # Continuous-emission means we never need to reset mid-print — the
        # MCU ISR runs from activate to deactivate without stopping.
        if self._needs_clock_reset:
            self._emit_reset_clock(self._mcu_anchor_clock)
            self._needs_clock_reset = False
        # Emit segments and advance the host mirror.  After this loop the
        # mirror equals the MCU's last-emitted phase, ready to anchor the
        # next flush.
        interval_ticks = self.mcu.seconds_to_clock(self.update_interval)
        first_seg = self._mcu_moves[0]
        last_seg = self._mcu_moves[num_mcu - 1]
        max_vel = max(abs(self._mcu_moves[i].velocity) for i in range(num_mcu))
        is_motion = max_vel != 0
        # Log first 4 emits, plus the moment motion first appears or stops.
        if (self._emit_log_count < 4
                or is_motion != self._last_emit_motion):
            logging.info(
                "Phase stepping %s: emit n=%d num_mcu=%d flush_time=%.6f"
                " first(start=0x%08x vel=%d accel=%d count=%d)"
                " last(start=0x%08x vel=%d accel=%d count=%d)"
                " max_abs_vel=%d positions[0]=%.4f positions[-1]=%.4f",
                self.name, n, num_mcu, flush_time,
                first_seg.start_position, first_seg.velocity,
                first_seg.acceleration, first_seg.count,
                last_seg.start_position, last_seg.velocity,
                last_seg.acceleration, last_seg.count,
                max_vel, self._positions[0], self._positions[n - 1])
            self._emit_log_count += 1
        self._last_emit_motion = is_motion
        for i in range(num_mcu):
            m = self._mcu_moves[i]
            self._queue_cmd.send([self.oid, interval_ticks,
                                  m.start_position, m.velocity,
                                  m.acceleration, m.count])
            self._mcu_anchor_pos = self._advance_mirror(
                m.start_position, m.velocity, m.acceleration, m.count)
            self._mcu_anchor_clock += m.count * interval_ticks
        # Track commanded position for the deactivation handoff.
        last_phase_pos = self._positions[n - 1]
        self._last_commanded_position = self._phase_to_commanded(
            last_phase_pos)
        # Continuity logging for activation only (no idle resume in
        # continuous-emission model).
        if self._activation_pending and self._activation_vector:
            start_phase = _phase_from_mcu_start_position(
                self._mcu_moves[0].start_position)
            self._record_continuity(
                'activation',
                self._activation_vector.get('phase'),
                self._activation_vector.get('a'),
                self._activation_vector.get('b'),
                start_phase)
            self._activation_pending = False
            self._trace_event('activate_first_emit',
                              flush=flush_time, anchor=self._mcu_anchor_pos,
                              start_phase=start_phase, num_mcu=num_mcu)
        if num_mcu > 0:
            self._trace_event('queue', flush=flush_time, samples=n,
                              mcu=num_mcu,
                              anchor_after=self._mcu_anchor_pos)

class TMCPhaseStepping:
    """Klipper module for configuring phase stepping on TMC5160 drivers."""
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name()
        self.phase_steppers = {}
        self._phase_stepping_enabled = False
        self._fault_pending = False
        # AWD pair config: each line is "stepper_a, stepper_b".  Multiple
        # phase_pair entries (one per pair) are supported via Klipper's
        # repeat-key syntax (phase_pair, phase_pair2, ...).
        self._pair_specs = []
        for opt_name in config.get_prefix_options('phase_pair'):
            opt_val = config.get(opt_name)
            members = [m.strip() for m in opt_val.split(',') if m.strip()]
            if len(members) != 2:
                raise config.error(
                    "%s '%s': expected exactly two stepper names, got '%s'"
                    % (self.name, opt_name, opt_val))
            self._pair_specs.append(tuple(members))
        self.printer.register_event_handler('klippy:ready',
                                            self._handle_ready)
        self.printer.register_event_handler(
            'homing:homing_move_begin', self._handle_homing_move_begin)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('PHASE_STEPPER_DEBUG', self.cmd_DEBUG,
                               desc="Query non-disruptive phase stepper state")
        gcode.register_command('PHASE_STEPPER_SUSPEND', self.cmd_SUSPEND,
                               desc="Suspend phase stepping immediately")
        gcode.register_command('PHASE_STEPPER_RESUME', self.cmd_RESUME,
                               desc="Resume phase stepping now or after delay")
        gcode.register_command('PHASE_STEPPER_SET_DIRECTION',
                               self.cmd_SET_DIRECTION,
                               desc="Override per-motor phase direction")
        gcode.register_command('PHASE_STEPPER_TRACE', self.cmd_TRACE,
                               desc="Dump recent host-side phase-stepper trace")
        gcode.register_command('PHASE_STEPPER_STATUS', self.cmd_STATUS,
                               desc="Query phase stepper diagnostics")
        gcode.register_command('TEST_XDIRECT', self.cmd_TEST_XDIRECT,
                               desc="Manual XDIRECT test on first stepper")
    def _resolve_pairs(self):
        """Wire each MCU_phase_stepper's _pair_partner pointer based on the
        phase_pair config.  Called once before first activate."""
        # Build name -> ps lookup that's indifferent to the leading
        # 'tmc5160 ' prefix in self.phase_steppers keys.
        name_to_ps = {}
        for full_name, ps in self.phase_steppers.items():
            stepper_name = ps.stepper.get_name()
            name_to_ps[stepper_name] = ps
            name_to_ps[full_name] = ps
        for a, b in self._pair_specs:
            ps_a = name_to_ps.get(a)
            ps_b = name_to_ps.get(b)
            if ps_a is None or ps_b is None:
                logging.warning(
                    "Phase stepping: phase_pair '%s, %s' references missing"
                    " stepper(s); pair binding skipped.", a, b)
                continue
            ps_a._pair_partner = ps_b
            ps_b._pair_partner = ps_a
            logging.info("Phase stepping pair: %s <-> %s", a, b)
    def _activate_all(self):
        if self._phase_stepping_enabled:
            return
        self._resolve_pairs()
        for name, ps in self.phase_steppers.items():
            ps.activate()
        self._phase_stepping_enabled = True
    def _handle_ready(self):
        pass  # Don't activate at boot — wait for first homing cycle
    def _deactivate_all(self, reason, flush_toolhead=False):
        if not self._phase_stepping_enabled:
            return
        toolhead = self.printer.lookup_object("toolhead")
        if flush_toolhead:
            toolhead.flush_step_generation()
        print_time = toolhead.get_last_move_time()
        # Stop all MCU-side timers first to kill ISR SPI traffic.
        for name, ps in self.phase_steppers.items():
            ps.stop_timer()
        # Now safe to do TMC register writes (no ISR contention).
        for name, ps in self.phase_steppers.items():
            ps.deactivate(print_time)
        self._phase_stepping_enabled = False
    def _handle_phase_fault(self, name, reason):
        if self._fault_pending:
            return
        self._fault_pending = True
        def do_fault(_eventtime):
            self._fault_pending = False
            self._deactivate_all("phase fault: %s (%s)" % (name, reason),
                                 flush_toolhead=True)
        self.reactor.register_callback(do_fault)
    def _handle_homing_move_begin(self, hmove):
        if self._phase_stepping_enabled:
            raise self.printer.command_error(
                "Cannot home while phase stepping is active."
                " Run PHASE_STEPPER_SUSPEND first.")
    def _tmc_debug_state(self, ps, live=False):
        if ps.tmc_obj is None:
            return {}
        mcu_tmc = ps.tmc_obj.mcu_tmc
        fields = mcu_tmc.fields
        ihr_cached = fields.registers.get("IHOLD_IRUN", 0)
        gconf_cached = fields.registers.get("GCONF", 0)
        tpowerdown_cached = fields.registers.get("TPOWERDOWN", 0)
        state = {
            'ihold': fields.get_field("ihold", ihr_cached, "IHOLD_IRUN"),
            'irun': fields.get_field("irun", ihr_cached, "IHOLD_IRUN"),
            'iholddelay': fields.get_field("iholddelay", ihr_cached,
                                           "IHOLD_IRUN"),
            'tpowerdown': fields.get_field("tpowerdown", tpowerdown_cached,
                                           "TPOWERDOWN"),
            'faststandstill': fields.get_field("faststandstill", gconf_cached,
                                               "GCONF"),
            'shaft': fields.get_field("shaft", gconf_cached, "GCONF"),
            'active_tpowerdown': ps.active_tpowerdown,
        }
        if not live:
            return state
        mscnt = mcu_tmc.get_register("MSCNT") & 0x3FF
        xdirect = mcu_tmc.get_register("XTARGET")
        mscuract = mcu_tmc.get_register("MSCURACT")
        drv_status = mcu_tmc.get_register("DRV_STATUS")
        gstat = mcu_tmc.get_register("GSTAT")
        live_gconf = mcu_tmc.get_register("GCONF")
        xdirect_a, xdirect_b = _decode_xdirect(xdirect)
        cur_a, cur_b = _decode_mscuract(mscuract)
        state.update({
            'mscnt': mscnt,
            'xdirect_raw': xdirect,
            'mscuract_raw': mscuract,
            'xdirect_a': xdirect_a,
            'xdirect_b': xdirect_b,
            'xdirect_phase': _phase_from_currents(xdirect_a, xdirect_b),
            'mscuract_a': cur_a,
            'mscuract_b': cur_b,
            'mscuract_phase': _phase_from_currents(cur_a, cur_b),
            'drv_status': drv_status,
            'gstat': gstat,
            'live_faststandstill': fields.get_field(
                "faststandstill", live_gconf, "GCONF"),
            'live_shaft': fields.get_field("shaft", live_gconf, "GCONF"),
            'live_direct_mode': bool(live_gconf & (1 << 16)),
            'live_en_pwm_mode': bool(live_gconf & (1 << 2)),
            'live_gconf_raw': live_gconf,
            'cs_actual': fields.get_field("cs_actual", drv_status,
                                          "DRV_STATUS"),
            'fsactive': fields.get_field("fsactive", drv_status,
                                         "DRV_STATUS"),
            'stst': fields.get_field("stst", drv_status, "DRV_STATUS"),
            'otpw': fields.get_field("otpw", drv_status, "DRV_STATUS"),
            's2ga': fields.get_field("s2ga", drv_status, "DRV_STATUS"),
            's2gb': fields.get_field("s2gb", drv_status, "DRV_STATUS"),
            'ola': fields.get_field("ola", drv_status, "DRV_STATUS"),
            'olb': fields.get_field("olb", drv_status, "DRV_STATUS"),
            'gstat_reset': fields.get_field("reset", gstat, "GSTAT"),
            'gstat_drv_err': fields.get_field("drv_err", gstat, "GSTAT"),
            'gstat_uv_cp': fields.get_field("uv_cp", gstat, "GSTAT"),
        })
        return state
    def cmd_STATUS(self, gcmd):
        """PHASE_STEPPER_STATUS - read TMC register state for diagnostics.
        Reads XDIRECT first (while ISR is running) to capture ISR-written
        values, then stops ISR and reads remaining registers."""
        # Phase 1: Read XDIRECT while ISR is still running.
        # The bus_hold mechanism makes the ISR skip during our SPI reads.
        xdirect_live = {}
        for name, ps in sorted(self.phase_steppers.items()):
            if ps.tmc_obj is not None and ps._phase_stepping_active:
                raw = ps.tmc_obj.mcu_tmc.get_register("XTARGET")
                xdirect_live[name] = raw
        # Phase 2: Stop ISR timers
        was_active = {}
        for name, ps in self.phase_steppers.items():
            was_active[name] = ps._phase_stepping_active
            if ps._phase_stepping_active and ps._stop_cmd is not None:
                ps._stop_cmd.send([ps.oid])
        # Phase 3: Read all registers with ISR stopped
        for name, ps in sorted(self.phase_steppers.items()):
            if ps.tmc_obj is None:
                gcmd.respond_info("%s: TMC not available" % name)
                continue
            mcu_tmc = ps.tmc_obj.mcu_tmc
            gconf = mcu_tmc.get_register("GCONF")
            mscuract = mcu_tmc.get_register("MSCURACT")
            xd_after = mcu_tmc.get_register("XTARGET")  # after stop (zeroed)
            pwmconf = mcu_tmc.get_register("PWMCONF")
            drv_status = mcu_tmc.get_register("DRV_STATUS")
            chopconf = mcu_tmc.get_register("CHOPCONF")
            cs_actual = mcu_tmc.fields.get_field("cs_actual", drv_status,
                                                 "DRV_STATUS")
            fsactive = mcu_tmc.fields.get_field("fsactive", drv_status,
                                                "DRV_STATUS")
            stst = mcu_tmc.fields.get_field("stst", drv_status, "DRV_STATUS")
            # Decode XDIRECT captured while ISR was running
            live = xdirect_live.get(name, 0)
            live_a, live_b = _decode_xdirect(live)
            # Decode values after stop
            after_a, after_b = _decode_xdirect(xd_after)
            cur_a, cur_b = _decode_mscuract(mscuract)
            gcmd.respond_info(
                "%s: GCONF=0x%08x direct_mode=%s en_pwm=%s"
                " XDIRECT_live_a=%d XDIRECT_live_b=%d"
                " XDIRECT_after_a=%d XDIRECT_after_b=%d"
                " MSCURACT_a=%d MSCURACT_b=%d"
                " PWMCONF=0x%08x DRV_STATUS=0x%08x"
                " cs_actual=%d fsactive=%s stst=%s"
                " CHOPCONF=0x%08x" % (
                    name, gconf,
                    bool(gconf & (1 << 16)), bool(gconf & (1 << 2)),
                    live_a, live_b, after_a, after_b,
                    cur_a, cur_b,
                    pwmconf, drv_status, cs_actual, bool(fsactive),
                    bool(stst), chopconf))
        # Flag clock reset since we stopped the timer for status read.
        for name, ps in self.phase_steppers.items():
            if was_active[name]:
                ps._needs_clock_reset = True
    def cmd_DEBUG(self, gcmd):
        """PHASE_STEPPER_DEBUG - non-disruptive host/MCU state dump.
        MOTOR=stepper_x optionally limits output to one motor.
        MCU=1 also queries MCU-side counters. TMC=1 also reads live TMC
        standstill/current state.
        PROBE=1 reads XDIRECT four times in a row to detect whether the
        MCU's per-tick writes are actually landing in the TMC.  If the
        readbacks differ across the four reads, the MCU is updating
        XDIRECT on the wire (motor SHOULD be moving, problem is mechanical
        or driver-config).  If the readbacks are identical, the MCU's SPI
        bursts aren't reaching the TMC even though `write_count` climbs."""
        if not self.phase_steppers:
            gcmd.respond_info("No phase steppers registered")
            return
        query_mcu = gcmd.get_int("MCU", 0, minval=0, maxval=1)
        query_tmc = gcmd.get_int("TMC", 0, minval=0, maxval=1)
        probe_writes = gcmd.get_int("PROBE", 0, minval=0, maxval=1)
        motor_filter = gcmd.get("MOTOR", None)
        if motor_filter and motor_filter not in self.phase_steppers:
            gcmd.respond_info("Unknown motor '%s'. Available: %s"
                              % (motor_filter,
                                 ", ".join(sorted(self.phase_steppers))))
            return
        names = ([motor_filter] if motor_filter
                 else sorted(self.phase_steppers.keys()))
        gcmd.respond_info("phase_stepping_enabled=%s"
                          " mcu_query=%s tmc_query=%s"
                          % (self._phase_stepping_enabled,
                             bool(query_mcu),
                             bool(query_tmc)))
        for name in names:
            ps = self.phase_steppers[name]
            mstat = {}
            mcu_status = "skipped"
            tmc_state = {}
            tmc_status = "skipped"
            if query_mcu:
                try:
                    mstat = ps.get_mcu_status() or {}
                    mcu_status = "ok" if mstat else "empty"
                except Exception as e:
                    mcu_status = "query_error"
                    logging.info("Phase stepping MCU status query failed for %s:"
                                 " %s", name, e)
            try:
                tmc_state = self._tmc_debug_state(ps, live=bool(query_tmc))
                tmc_status = "ok" if tmc_state else "missing"
            except Exception as e:
                tmc_status = "query_error" if query_tmc else "cache_error"
                logging.info("Phase stepping TMC debug query failed for %s: %s",
                             name, e)
            if probe_writes and ps.tmc_obj is not None:
                try:
                    mcu_tmc = ps.tmc_obj.mcu_tmc
                    # Stamp XDIRECT with a recognizable signature via the
                    # standard SPI command path (this we KNOW works — same
                    # path used at activate).  Then read XDIRECT four times.
                    # If MCU's per-tick writes land at the TMC, the readbacks
                    # differ from the signature and from each other (because
                    # ~25 group-ISR ticks fire between consecutive reads).
                    # If they all equal the signature, no MCU write reached
                    # the TMC — diagnoses an SPI / DMA path issue on the
                    # phase-stepping bus.
                    sig = 0x00ABCDEF
                    mcu_tmc.set_register("XTARGET", sig)
                    rb1 = mcu_tmc.get_register("XTARGET")
                    rb2 = mcu_tmc.get_register("XTARGET")
                    rb3 = mcu_tmc.get_register("XTARGET")
                    rb4 = mcu_tmc.get_register("XTARGET")
                    overwritten = (rb1 != sig or rb2 != sig
                                   or rb3 != sig or rb4 != sig)
                    gcmd.respond_info(
                        "%s probe: signature=0x%08x rb1=0x%08x rb2=0x%08x"
                        " rb3=0x%08x rb4=0x%08x mcu_writes_landing=%s" % (
                            name, sig, rb1, rb2, rb3, rb4, overwritten))
                except Exception as e:
                    gcmd.respond_info("%s probe failed: %s" % (name, e))
            gcmd.respond_info(
                "%s: active=%s needs_reset=%s"
                " update_hz=%.0f phase_bus=%d"
                " anchor_pos=0x%08x anchor_clock=%d"
                " sample_window=%.3f"
                " max_error=%.3f"
                " mcu_status=%s tmc_status=%s"
                " cached_ihold=%s cached_irun=%s cached_iholddelay=%s"
                " cached_tpowerdown=%s active_tpowerdown=%s"
                " cached_faststandstill=%s"
                " event_count=%s write_count=%s skip_count=%s"
                " last_phase=%s position=%s"
                " mscnt=%s xdirect_a=%s xdirect_b=%s"
                " mscuract_a=%s mscuract_b=%s"
                " drv_stst=%s drv_cs_actual=%s drv_fsactive=%s"
                " drv_otpw=%s drv_s2ga=%s drv_s2gb=%s"
                " drv_ola=%s drv_olb=%s"
                " gstat_reset=%s gstat_drv_err=%s gstat_uv_cp=%s"
                " live_faststandstill=%s"
                " live_direct_mode=%s live_en_pwm_mode=%s"
                " live_gconf_raw=0x%08x"
                " pair_partner=%s" % (
                    name, ps._phase_stepping_active,
                    ps._needs_clock_reset,
                    ps.update_rate, ps.phase_bus,
                    ps._mcu_anchor_pos & 0xFFFFFFFF,
                    ps._mcu_anchor_clock,
                    (MAX_PHASE_SAMPLES - 1) * ps.update_interval,
                    ps.max_error, mcu_status, tmc_status,
                    tmc_state.get('ihold', 'n/a'),
                    tmc_state.get('irun', 'n/a'),
                    tmc_state.get('iholddelay', 'n/a'),
                    tmc_state.get('tpowerdown', 'n/a'),
                    tmc_state.get('active_tpowerdown', 'n/a'),
                    tmc_state.get('faststandstill', 'n/a'),
                    mstat.get('event_count', 'n/a'),
                    mstat.get('write_count', 'n/a'),
                    mstat.get('skip_count', 'n/a'),
                    mstat.get('last_phase', 'n/a'),
                    mstat.get('position', 'n/a'),
                    tmc_state.get('mscnt', 'n/a'),
                    tmc_state.get('xdirect_a', 'n/a'),
                    tmc_state.get('xdirect_b', 'n/a'),
                    tmc_state.get('mscuract_a', 'n/a'),
                    tmc_state.get('mscuract_b', 'n/a'),
                    tmc_state.get('stst', 'n/a'),
                    tmc_state.get('cs_actual', 'n/a'),
                    tmc_state.get('fsactive', 'n/a'),
                    tmc_state.get('otpw', 'n/a'),
                    tmc_state.get('s2ga', 'n/a'),
                    tmc_state.get('s2gb', 'n/a'),
                    tmc_state.get('ola', 'n/a'),
                    tmc_state.get('olb', 'n/a'),
                    tmc_state.get('gstat_reset', 'n/a'),
                    tmc_state.get('gstat_drv_err', 'n/a'),
                    tmc_state.get('gstat_uv_cp', 'n/a'),
                    tmc_state.get('live_faststandstill', 'n/a'),
                    tmc_state.get('live_direct_mode', 'n/a'),
                    tmc_state.get('live_en_pwm_mode', 'n/a'),
                    tmc_state.get('live_gconf_raw', 0),
                    (ps._pair_partner.stepper.get_name()
                     if ps._pair_partner is not None else 'none')))
            preload = ps._activation_vector
            continuity = ps._last_continuity
            mcu_last_phase = mstat.get('last_phase', None)
            if isinstance(mcu_last_phase, int):
                mcu_last_a, mcu_last_b = _phase_table_currents(mcu_last_phase)
            else:
                mcu_last_a = mcu_last_b = 'n/a'
            xtarget_to_mcu = ('n/a' if not isinstance(mcu_last_phase, int)
                              else _phase_delta(
                                  mcu_last_phase,
                                  tmc_state.get('xdirect_phase')))
            gcmd.respond_info(
                "%s direct: phase_dir=%+.0f phase_dir_source=%s"
                " dir_inverted=%s phase_offset=%.3f"
                " cached_shaft=%s live_shaft=%s"
                " preload_source=%s preload_phase=%s preload_a=%s preload_b=%s"
                " preload_rb_phase=%s preload_rb_delta=%s"
                " mscuract_phase=%s xtarget_phase=%s" % (
                    name, ps._phase_direction, ps._phase_direction_source,
                    ps._dir_inverted, ps._phase_offset,
                    tmc_state.get('shaft', 'n/a'),
                    tmc_state.get('live_shaft', 'n/a'),
                    preload.get('source', 'n/a'),
                    preload.get('phase', 'n/a'),
                    preload.get('a', 'n/a'),
                    preload.get('b', 'n/a'),
                    preload.get('readback_phase', 'n/a'),
                    preload.get('readback_delta', 'n/a'),
                    tmc_state.get('mscuract_phase', 'n/a'),
                    tmc_state.get('xdirect_phase', 'n/a')))
            gcmd.respond_info(
                "%s continuity: origin=%s from_phase=%s to_phase=%s"
                " delta_phase=%s delta_peak=%s"
                " mcu_last_phase=%s mcu_last_a=%s mcu_last_b=%s"
                " xtarget_to_mcu_delta=%s" % (
                    name, continuity.get('origin', 'n/a'),
                    continuity.get('from_phase', 'n/a'),
                    continuity.get('to_phase', 'n/a'),
                    continuity.get('delta_phase', 'n/a'),
                    continuity.get('delta_peak', 'n/a'),
                    mcu_last_phase if mcu_last_phase is not None else 'n/a',
                    mcu_last_a, mcu_last_b, xtarget_to_mcu))
    def cmd_SUSPEND(self, gcmd):
        """PHASE_STEPPER_SUSPEND - disable phase stepping immediately."""
        self._deactivate_all("gcode suspend", flush_toolhead=True)
        gcmd.respond_info("Phase stepping suspended")
    def cmd_TRACE(self, gcmd):
        """PHASE_STEPPER_TRACE - dump recent host-side trace events.
        MOTOR=stepper_x optionally limits output to one motor.
        COUNT=<n> limits the number of entries per motor.
        CLEAR=1 clears the trace buffer after printing."""
        if not self.phase_steppers:
            gcmd.respond_info("No phase steppers registered")
            return
        motor_filter = gcmd.get("MOTOR", None)
        if motor_filter and motor_filter not in self.phase_steppers:
            gcmd.respond_info("Unknown motor '%s'. Available: %s"
                              % (motor_filter,
                                 ", ".join(sorted(self.phase_steppers))))
            return
        count = gcmd.get_int("COUNT", 16, minval=1, maxval=32)
        clear = gcmd.get_int("CLEAR", 0, minval=0, maxval=1)
        names = ([motor_filter] if motor_filter
                 else sorted(self.phase_steppers.keys()))
        for name in names:
            ps = self.phase_steppers[name]
            events = ps._trace_events[-count:]
            if not events:
                gcmd.respond_info("%s: no trace events" % (name,))
                continue
            start_index = len(ps._trace_events) - len(events)
            for offset, entry in enumerate(events):
                fields = ["kind=%s" % (entry.get('kind', 'unknown'),)]
                for key in sorted(k for k in entry.keys() if k != 'kind'):
                    val = entry[key]
                    if isinstance(val, float):
                        fields.append("%s=%.3f" % (key, val))
                    else:
                        fields.append("%s=%s" % (key, val))
                gcmd.respond_info("%s[%d]: %s"
                                  % (name, start_index + offset,
                                     " ".join(fields)))
            if clear:
                del ps._trace_events[:]
    def cmd_RESUME(self, gcmd):
        """PHASE_STEPPER_RESUME - re-enable phase stepping."""
        self._activate_all()
        gcmd.respond_info("Phase stepping resumed")
    def cmd_SET_DIRECTION(self, gcmd):
        """Override the resolved phase direction for one or more motors.
        SIGN=-1 or SIGN=+1 forces the electrical phase sign.
        SIGN=0 clears the override and returns to invert_dir auto mode.
        MOTOR=stepper_y optionally limits the change to one motor.
        REACTIVATE=1 restarts active phase stepping immediately so the
        new direction takes effect without a printer restart."""
        if not self.phase_steppers:
            gcmd.respond_info("No phase steppers registered")
            return
        sign = gcmd.get_int("SIGN", minval=-1, maxval=1)
        motor_filter = gcmd.get("MOTOR", None)
        if motor_filter and motor_filter not in self.phase_steppers:
            gcmd.respond_info("Unknown motor '%s'. Available: %s"
                              % (motor_filter,
                                 ", ".join(sorted(self.phase_steppers))))
            return
        names = ([motor_filter] if motor_filter
                 else sorted(self.phase_steppers.keys()))
        active_before = self._phase_stepping_enabled
        reactivate = gcmd.get_int(
            "REACTIVATE", 1 if active_before else 0, minval=0, maxval=1)
        if reactivate and active_before:
            self._deactivate_all("direction override change",
                                 flush_toolhead=True)
        for name in names:
            ps = self.phase_steppers[name]
            ps._phase_direction_override = sign
            effective, source = ps._resolve_phase_direction(
                ps.stepper.get_dir_inverted()[0])
            gcmd.respond_info(
                "%s: phase_direction_override=%+d"
                " effective_phase_dir=%+.0f source=%s"
                % (name, sign, effective, source))
        if reactivate and active_before:
            self._activate_all()
            gcmd.respond_info("Phase stepping reactivated with updated"
                              " direction override")
    def cmd_TEST_XDIRECT(self, gcmd):
        """Drive motors through microsteps via direct_mode.
        STEPS=N (default 10000, negative for reverse direction)
        MOTOR=stepper_x (optional, test single motor only)
        SIGN={-1,0,+1} forces electrical phase direction for audit
        DWELL=<seconds> pauses between writes for visible probes
        RESTORE=1 rewrites the starting hold vector before exit
        Run AFTER homing so TMC registers are initialized (IRUN>0)."""
        if not self.phase_steppers:
            gcmd.respond_info("No phase steppers registered")
            return
        # Determine which motors to test
        motor_filter = gcmd.get("MOTOR", None)
        if motor_filter and motor_filter not in self.phase_steppers:
            gcmd.respond_info("Unknown motor '%s'. Available: %s"
                              % (motor_filter,
                                 ", ".join(sorted(self.phase_steppers))))
            return
        test_names = ([motor_filter] if motor_filter
                      else sorted(self.phase_steppers.keys()))
        forced_sign = gcmd.get_int("SIGN", 0, minval=-1, maxval=1)
        dwell = gcmd.get_float("DWELL", 0.0, minval=0.0)
        restore_vector = gcmd.get_int("RESTORE", 1, minval=0, maxval=1)
        # Stop phase stepping ISR if active (we'll do our own writes)
        was_active = {}
        for name, ps in self.phase_steppers.items():
            was_active[name] = ps._phase_stepping_active
            if ps._phase_stepping_active:
                ps.stop_timer()
        toolhead = self.printer.lookup_object("toolhead")
        # Dump TMC register state for diagnostics
        # (IHOLD_IRUN is write-only on TMC5160, skip it)
        for name in test_names:
            ps = self.phase_steppers[name]
            mcu_tmc = ps.tmc_obj.mcu_tmc
            gconf = mcu_tmc.get_register("GCONF")
            chopconf = mcu_tmc.get_register("CHOPCONF")
            pwm_conf = mcu_tmc.get_register("PWMCONF")
            toff = chopconf & 0xF
            gcmd.respond_info(
                "%s: GCONF=0x%08x CHOPCONF=0x%08x TOFF=%d"
                " PWMCONF=0x%08x" % (name, gconf, chopconf, toff, pwm_conf))
        # Set IHOLD=IRUN for each motor (direct_mode scales by IHOLD)
        saved_gconf = {}
        saved_vectors = {}
        start_phases = {}
        for name in test_names:
            ps = self.phase_steppers[name]
            mcu_tmc = ps.tmc_obj.mcu_tmc
            # Reinit key registers from config cache
            fields = mcu_tmc.fields
            cached_ihr = fields.registers.get("IHOLD_IRUN", 0)
            for reg_name in ["GLOBALSCALER", "CHOPCONF", "TPOWERDOWN"]:
                val = fields.registers.get(reg_name, None)
                if val is not None:
                    mcu_tmc.set_register(reg_name, val)
            # Set IHOLD = IRUN
            irun = fields.get_field("irun", cached_ihr, "IHOLD_IRUN")
            ihr_val = _compose_tmc_field(fields, "ihold", irun,
                                         cached_ihr, "IHOLD_IRUN")
            mcu_tmc.set_register("IHOLD_IRUN", ihr_val)
            # Read MSCNT
            mscnt = mcu_tmc.get_register("MSCNT") & 0x3FF
            mscuract_raw = mcu_tmc.get_register("MSCURACT")
            start_phases[name] = mscnt
            # Enable direct_mode with SpreadCycle (not StealthChop)
            saved_gconf[name] = mcu_tmc.get_register("GCONF")
            gconf = (saved_gconf[name] | (1 << 16)) & ~(1 << 2)
            mcu_tmc.set_register("GCONF", gconf)
            # Pre-load the live electrical hold so direct_mode takes over
            # without a synthetic phase jump.
            a, b, source = _activation_preload_currents(mscnt, mscuract_raw)
            val = ((b & 0x1FF) << 16) | (a & 0x1FF)
            saved_vectors[name] = val
            mcu_tmc.set_register("XTARGET", val)
            gcmd.respond_info(
                "%s: MSCNT=%d IRUN=%d preload_phase=%s"
                " preload A=%d B=%d source=%s"
                % (name, mscnt, irun, _phase_from_currents(a, b),
                   a, b, source))
        # Determine per-motor MSCNT direction from dir_pin inversion.
        # The ordinary motion path may override the invert_dir-based default
        # when direct-mode hardware proves the electrical sign is different.
        motor_dirs = {}
        for name in test_names:
            ps = self.phase_steppers[name]
            invert = ps.stepper.get_dir_inverted()[0]
            auto_direction = -1 if invert else 1
            if forced_sign:
                effective_direction = forced_sign
                direction_source = "gcode"
            else:
                resolved, direction_source = ps._resolve_phase_direction(
                    invert)
                effective_direction = int(resolved)
            motor_dirs[name] = effective_direction
            gcmd.respond_info(
                "%s: invert_dir=%s auto_phase_dir=%+d"
                " effective_phase_dir=%+d dir_source=%s"
                % (name, invert, auto_direction,
                   motor_dirs[name], direction_source))
        steps = gcmd.get_int("STEPS", 10000)
        step_dir = 1 if steps >= 0 else -1
        abs_steps = abs(steps)
        gcmd.respond_info(
            "Stepping %s %d microsteps (~%.1fmm)..."
            % (", ".join(test_names),
               steps, abs(steps) * 0.00078))
        for step in range(abs_steps):
            for name in test_names:
                ps = self.phase_steppers[name]
                mcu_tmc = ps.tmc_obj.mcu_tmc
                phase = (start_phases[name]
                         + motor_dirs[name] * step_dir
                         * (step + 1)) & 0x3FF
                angle = phase * 2.0 * math.pi / 1024.0
                a = int(round(248.0 * math.cos(angle)))
                b = int(round(248.0 * math.sin(angle)))
                val = ((b & 0x1FF) << 16) | (a & 0x1FF)
                mcu_tmc.set_register("XTARGET", val)
            if dwell > 0.0:
                toolhead.dwell(dwell)
        gcmd.respond_info("Done. Restoring registers...")
        # Restore GCONF and IHOLD_IRUN from config cache
        for name in test_names:
            ps = self.phase_steppers[name]
            mcu_tmc = ps.tmc_obj.mcu_tmc
            if restore_vector:
                mcu_tmc.set_register("XTARGET", saved_vectors[name])
                if dwell > 0.0:
                    toolhead.dwell(dwell)
            final_xdirect = mcu_tmc.get_register("XTARGET")
            final_mscuract = mcu_tmc.get_register("MSCURACT")
            final_a, final_b = _decode_xdirect(final_xdirect)
            cur_a, cur_b = _decode_mscuract(final_mscuract)
            gcmd.respond_info(
                "%s: final_xtarget_phase=%s final_xtarget_a=%d"
                " final_xtarget_b=%d final_mscuract_phase=%s"
                " final_mscuract_a=%d final_mscuract_b=%d"
                % (name, _phase_from_currents(final_a, final_b),
                   final_a, final_b,
                   _phase_from_currents(cur_a, cur_b),
                   cur_a, cur_b))
            mcu_tmc.set_register("GCONF", saved_gconf[name])
            fields = mcu_tmc.fields
            ihr_cached = fields.registers.get("IHOLD_IRUN", None)
            if ihr_cached is not None:
                mcu_tmc.set_register("IHOLD_IRUN", ihr_cached)
        for name, was_running in was_active.items():
            if was_running:
                self.phase_steppers[name].activate()
        gcmd.respond_info("TEST_XDIRECT done.")
    def register_phase_stepper(self, name, phase_stepper):
        phase_stepper.set_fault_handler(self._handle_phase_fault)
        self.phase_steppers[name] = phase_stepper

def load_config(config):
    return TMCPhaseStepping(config)
