# TMC5160 Phase Stepping support
#
# Copyright (C) 2026  klipper contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
from .. import chelper

FIXED_POINT_SCALE = 65536.  # 16.16 fixed-point
PHASE_ACTIVE_TPOWERDOWN = 255

# Maximum number of position samples per flush cycle.
# At the default 10kHz update rate, 32768 samples cover ~3.28s. Real AWD logs
# showed the first resumed flush arriving ~1.25s after motion started; with the
# default 0.5s idle lookback, 16384 samples (~1.64s) was still too short and
# the host queued the move from mid-trajectory. 32768 leaves headroom for that
# long first-flush latency while keeping the whole resume on the host side.
MAX_PHASE_SAMPLES = 32768
# Maximum number of compressed segments per flush cycle
MAX_PHASE_SEGMENTS = 1024


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
    # TMC5160 datasheet: MSCURACT bit 8..0 is CUR_B, bit 24..16 is CUR_A.
    return _sign9(raw >> 16), _sign9(raw)


def _phase_table_currents(mscnt):
    angle = mscnt * 2.0 * math.pi / 1024.0
    # TMC5160 MSCNT indexes phase B on the sine wave and phase A on the
    # cosine wave. XTARGET uses A in bits 8..0 and B in bits 24..16.
    return (int(round(248.0 * math.cos(angle))),
            int(round(248.0 * math.sin(angle))))


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
        self.update_rate = config.getfloat('phase_update_rate', 10000.,
                                           above=1000., maxval=50000.)
        self.update_interval = 1. / self.update_rate
        max_idle_lookback = (MAX_PHASE_SAMPLES - 1) * self.update_interval
        self.idle_lookback = min(
            config.getfloat('phase_idle_lookback', 0.5, minval=0.0),
            max_idle_lookback)
        self.resume_lookback = min(
            config.getfloat('phase_resume_lookback', 2.0, minval=0.0),
            max_idle_lookback)
        self.idle_hold_time = config.getfloat(
            'phase_idle_hold_time', 1.0, minval=0.0)
        self.active_tpowerdown = config.getint(
            'phase_active_tpowerdown', PHASE_ACTIVE_TPOWERDOWN,
            minval=0, maxval=255)
        self.disable_faststandstill = config.getboolean(
            'phase_disable_faststandstill', True)
        self.max_error = config.getfloat('phase_compress_error', 0.25,
                                         above=0.)
        self._phase_stepping_active = False
        self._mcu_hold_active = False
        self._needs_clock_reset = True
        self._expected_end_clock = 0
        self._idle_position = None
        self._idle_start_time = None
        self._invalid_flush_count = 0
        self._faulted = False
        self._fault_handler = None
        self._phase_step_dist = None
        self._phase_direction = 1.0
        self._phase_offset = 0.0
        self._last_commanded_position = None
        self._activation_pending = False
        self._stagger_ticks = 0
        self._trace_events = []
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
        self._set_current_cmd = self.mcu.lookup_command(
            "set_phase_stepper_current oid=%c scale=%hu")
        self._stop_cmd = self.mcu.lookup_command(
            "stop_phase_stepper oid=%c")
        self._status_cmd = self.mcu.lookup_query_command(
            "get_phase_stepper_status oid=%c",
            "phase_stepper_status oid=%c event_count=%u write_count=%u"
            " skip_count=%u last_phase=%hu position=%i",
            oid=self.oid)
    def get_mcu_status(self):
        """Query MCU-side ISR diagnostic counters."""
        if self._status_cmd is None:
            return None
        return self._status_cmd.send([self.oid])
    def set_tmc_obj(self, tmc_obj):
        self.tmc_obj = tmc_obj
    def _handle_connect(self):
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.register_step_generator(self.generate_and_queue)
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
    def activate(self, stagger_ticks=0):
        """Enable XDIRECT mode on TMC5160 and start phase stepping."""
        self._stagger_ticks = stagger_ticks
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
        cached_ihr = fields.registers.get("IHOLD_IRUN", 0)
        # Get configured microstep resolution for position scaling.
        # Klipper step_dist maps to configured microsteps (e.g. 16/full step),
        # but MSCNT uses 256 native microsteps per full step.  We need the
        # ratio to convert positions to MSCNT units.
        mres = fields.get_field("mres")
        microsteps = 256 >> mres  # mres=4 -> 16, mres=0 -> 256
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
        chop_rb = mcu_tmc.get_register("CHOPCONF")
        logging.info("Phase stepping %s: reinit CHOPCONF=0x%08x",
                     self.name, chop_rb)
        # Read the live phase/current state before enabling direct_mode so the
        # handoff preserves the actual electrical hold, not an inferred one.
        mscnt = mcu_tmc.get_register("MSCNT") & 0x3FF
        mscuract_raw = mcu_tmc.get_register("MSCURACT")
        live_cur_a, live_cur_b = _decode_mscuract(mscuract_raw)
        cur_a, cur_b, preload_source = _activation_preload_currents(
            mscnt, mscuract_raw)
        # Save original GCONF, then enable direct_mode with SpreadCycle.
        # TMC5160 datasheet: SpreadCycle gives direct chopper control in
        # direct_mode.  StealthChop's auto-regulation can't track at 10kHz
        # and falls back to MSLUT — must use SpreadCycle (en_pwm_mode=0).
        self._saved_gconf = mcu_tmc.get_register("GCONF")
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
        self._needs_clock_reset = True
        self._mcu_hold_active = False
        self._invalid_flush_count = 0
        self._faulted = False
        self._idle_position = None
        self._idle_start_time = None
        # Reset per-movement logging counters so each new move gets logged
        self._had_zero_samples = True  # will trigger fresh log on first data
        self._move_log_count = 0
        if not hasattr(self, '_total_log_count'):
            self._total_log_count = 0
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
        direction = -1.0 if invert_dir else 1.0
        self._phase_direction = direction
        self._ffi_lib.phase_generator_set_direction(
            self._phase_generator, direction)
        # Calibrate phase offset: difference between TMC's internal phase
        # (MSCNT) and klipper's stepper position converted to MSCNT units.
        stepper_pos_mscnt = (self.stepper.get_commanded_position()
                             / step_dist * (256.0 / microsteps))
        klipper_phase = int(stepper_pos_mscnt * direction) & 0x3FF
        phase_offset = float((mscnt - klipper_phase) & 0x3FF)
        self._phase_offset = phase_offset
        self._ffi_lib.phase_generator_set_offset(
            self._phase_generator, phase_offset)
        self._last_commanded_position = self.stepper.get_commanded_position()
        self._ffi_lib.phase_generator_set_last_position(
            self._phase_generator, stepper_pos_mscnt * direction
            + phase_offset)
        self._trace_event('activate_begin',
                          mscnt=mscnt,
                          mscuract_a=live_cur_a,
                          mscuract_b=live_cur_b,
                          cmd_pos=self.stepper.get_commanded_position(),
                          klipper_phase=klipper_phase)
        self._trace_event('activate_preload',
                          preload_a=cur_a, preload_b=cur_b,
                          source=preload_source, offset=phase_offset)
        # Verify direct_mode was actually set
        gconf_readback = mcu_tmc.get_register("GCONF")
        direct_mode_set = bool(gconf_readback & (1 << 16))
        # Read back XDIRECT register to verify our preload write took effect
        xdirect_rb = mcu_tmc.get_register("XTARGET")
        xd_a, xd_b = _decode_xdirect(xdirect_rb)
        logging.info("Phase stepping %s: mscnt=%d klipper_phase=%d offset=%d"
                     " dir=%.0f invert=%s microsteps=%d"
                     " phase_step_dist=%.6f GCONF=0x%08x direct_mode=%s"
                     " active_tpowerdown=%d"
                     " mscuract_a=%d mscuract_b=%d preload_source=%s"
                     " preload_a=%d preload_b=%d xdirect_a=%d xdirect_b=%d"
                     " preload_raw=0x%08x xdirect_raw=0x%08x",
                     self.name, mscnt, klipper_phase, int(phase_offset),
                     direction, invert_dir, microsteps,
                     phase_step_dist, gconf_readback, direct_mode_set,
                     self.active_tpowerdown,
                     live_cur_a, live_cur_b, preload_source,
                     cur_a, cur_b, xd_a, xd_b,
                     (cur_b_u16 << 16) | cur_a_u16, xdirect_rb)
        # Initialize last_flush_time to current print time so generator
        # doesn't try to sample from t=0 (boot time)
        current_print_time = self.mcu.estimated_print_time(
            self.printer.get_reactor().monotonic())
        self._ffi_lib.phase_generator_set_time(
            self._phase_generator, current_print_time)
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
        self._mcu_hold_active = False
        if self._stop_cmd is not None:
            self._stop_cmd.send([self.oid])
    def _clear_idle_state(self):
        self._idle_position = None
        self._idle_start_time = None
    def _recent_history_start(self, flush_time):
        history = max(self.idle_lookback, self.resume_lookback)
        if history <= 0.0:
            return flush_time
        return max(0.0, flush_time - history)
    def _sampled_until(self, num_samples):
        if num_samples <= 0:
            return 0.0
        return self._samples[num_samples - 1].time + self.update_interval
    def _set_idle_boundary(self, flush_time, label='boundary'):
        lookback = min(self.idle_lookback, flush_time)
        boundary_time = flush_time - lookback
        self._clear_idle_state()
        self._expected_end_clock = 0
        self._mcu_hold_active = False
        self._needs_clock_reset = True
        self._ffi_lib.phase_generator_set_time(
            self._phase_generator, boundary_time)
        self._trace_event('boundary', flush=flush_time,
                          boundary=boundary_time, label=label)
        return boundary_time
    def _enter_idle_hold(self, flush_time, phase_position, label):
        self._idle_position = phase_position
        self._idle_start_time = flush_time
        self._last_commanded_position = self._phase_to_commanded(phase_position)
        self._expected_end_clock = 0
        # Natural idle is continuous only after at least one MCU phase batch has
        # actually run. Immediately after activation we may have only preloaded
        # XDIRECT with no timer running yet, so the first real move still needs
        # a reset_phase_clock anchor.
        self._needs_clock_reset = not self._mcu_hold_active
        self._ffi_lib.phase_generator_set_time(
            self._phase_generator, flush_time)
        self._ffi_lib.phase_generator_set_last_position(
            self._phase_generator, phase_position)
        self._trace_event('idle', flush=flush_time, pos=phase_position,
                          label=label, reset=self._needs_clock_reset,
                          mcu_hold=self._mcu_hold_active)
        if not self._had_zero_samples:
            self._had_zero_samples = True
            logging.info("phase_gen %s: %s at flush=%.3f pos=%.1f"
                         " (%s)",
                         self.name, label, flush_time, phase_position,
                         ("MCU idle hold active" if self._mcu_hold_active
                          else "preloaded direct-mode idle"))
    def _find_resume_window(self, num_samples, threshold):
        idle_pos = self._idle_position
        if idle_pos is None:
            return None
        last_match = None
        for i in range(num_samples):
            diff = self._positions[i] - idle_pos
            if diff <= threshold and diff >= -threshold:
                last_match = i
                continue
            if last_match is not None:
                return (last_match, i)
        if last_match is None:
            return None
        return (last_match, num_samples)
    def _phase_to_commanded(self, phase_pos):
        if self._phase_step_dist is None or not self._phase_direction:
            return None
        return ((phase_pos - self._phase_offset)
                * self._phase_step_dist / self._phase_direction)
    def _generate_samples(self, ffi_lib, flush_time):
        num_samples = ffi_lib.phase_generator_generate(
            self._phase_generator, flush_time,
            self._samples, MAX_PHASE_SAMPLES)
        if num_samples <= 0:
            if not self._had_zero_samples:
                self._had_zero_samples = True
                logging.info("phase_gen %s: 0 samples at flush=%.3f"
                             " (idle after %d move flushes)",
                             self.name, flush_time, self._move_log_count)
            ffi_lib.phase_generator_set_time(
                self._phase_generator, flush_time)
            return 0
        num_samples = self._trim_nonfinite_edges(num_samples, flush_time)
        if num_samples <= 0:
            self._clear_idle_state()
            return 0
        return num_samples
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
        self._clear_idle_state()
        self._activation_pending = False
        self._mcu_hold_active = False
        # Re-enable regular step/dir generation
        self.stepper.set_phase_stepping(False)
        # Restore original GCONF and IHOLD_IRUN
        mcu_tmc = self.tmc_obj.mcu_tmc
        if hasattr(self, '_saved_gconf'):
            mcu_tmc.set_register("GCONF", self._saved_gconf)
        else:
            gconf_val = mcu_tmc.get_register("GCONF")
            gconf_val &= ~(1 << 16)
            mcu_tmc.set_register("GCONF", gconf_val)
        # Restore IHOLD_IRUN from config cache (not from TMC readback,
        # which may be 0 if the TMC reset during phase stepping)
        fields = mcu_tmc.fields
        ihr_cached = fields.registers.get("IHOLD_IRUN", None)
        if ihr_cached is not None:
            mcu_tmc.set_register("IHOLD_IRUN", ihr_cached)
        tpowerdown_cached = fields.registers.get("TPOWERDOWN", None)
        if tpowerdown_cached is not None:
            mcu_tmc.set_register("TPOWERDOWN", tpowerdown_cached)
        self._trace_event('deactivate_restore',
                          restored_gconf=hasattr(self, '_saved_gconf'),
                          restored_tpowerdown=tpowerdown_cached is not None,
                          restored_ihold=ihr_cached is not None)
        logging.info("Phase stepping deactivated for %s", self.name)
    def _trim_nonfinite_edges(self, num_samples, flush_time):
        start_idx = 0
        end_idx = num_samples - 1
        while (start_idx < num_samples
               and not math.isfinite(self._samples[start_idx].position)):
            start_idx += 1
        while (end_idx >= start_idx
               and not math.isfinite(self._samples[end_idx].position)):
            end_idx -= 1
        if start_idx > end_idx:
            self._invalid_flush_count += 1
            boundary_time = self._set_idle_boundary(
                flush_time, label='all-nonfinite')
            logging.info("phase_gen %s: all-nonfinite flush at %.3f"
                         " (holding %.3fs lookback to %.3f invalid_flushes=%d)",
                         self.name, flush_time,
                         flush_time - boundary_time, boundary_time,
                         self._invalid_flush_count)
            if self._invalid_flush_count >= 4:
                self._report_fault("repeated non-finite phase samples")
            return 0
        trimmed_prefix = start_idx
        trimmed_suffix = num_samples - end_idx - 1
        if not trimmed_prefix and not trimmed_suffix:
            self._invalid_flush_count = 0
            return num_samples
        num_samples = end_idx - start_idx + 1
        for j in range(num_samples):
            self._samples[j] = self._samples[j + start_idx]
        self._invalid_flush_count = 0
        logging.info("phase_gen %s: trimmed non-finite samples"
                     " prefix=%d suffix=%d flush=%.3f",
                     self.name, trimmed_prefix, trimmed_suffix, flush_time)
        return num_samples
    def generate_and_queue(self, flush_time):
        """Called from the flush callback to generate phase moves."""
        if not self._phase_stepping_active:
            return
        ffi_lib = self._ffi_lib
        # Generate position samples
        num_samples = self._generate_samples(ffi_lib, flush_time)
        if num_samples <= 0:
            return
        ret = ffi_lib.phase_generator_extract_positions(
            self._samples, num_samples, self._positions)
        first_pos = self._samples[0].position
        last_pos = self._samples[num_samples - 1].position
        if ret < 0:
            # NaN or inf in position data — trapq returned invalid values.
            self._invalid_flush_count += 1
            boundary_time = self._set_idle_boundary(
                flush_time, label='invalid-samples')
            logging.info("phase_gen %s: inf/NaN at flush=%.3f"
                         " first_pos=%.3f last_pos=%.3f"
                         " (holding %.3fs lookback to %.3f"
                         " invalid_flushes=%d)",
                         self.name, flush_time, first_pos, last_pos,
                         flush_time - boundary_time, boundary_time,
                         self._invalid_flush_count)
            if self._invalid_flush_count >= 4:
                self._report_fault("repeated invalid phase samples")
            return
        self._invalid_flush_count = 0
        if abs(last_pos - first_pos) < 0.5:
            sampled_until = self._sampled_until(num_samples)
            # A long idle gap can overflow the sample buffer. If we only
            # inspect the oldest idle window and then jump straight to
            # flush_time, we can skip an entire move that happened in the
            # recent tail of that gap. Re-sample the most recent idle history
            # window before declaring this flush pure standstill.
            tail_start = self._recent_history_start(flush_time)
            if (sampled_until + self.update_interval * 0.5 < flush_time
                    and tail_start > self._samples[0].time
                    + self.update_interval * 0.5):
                self._trace_event('retry_idle_tail', flush=flush_time,
                                  old_start=self._samples[0].time,
                                  new_start=tail_start)
                logging.info(
                    "phase_gen %s: retrying recent idle tail"
                    " old_start=%.3f new_start=%.3f flush=%.3f",
                    self.name, self._samples[0].time,
                    tail_start, flush_time)
                ffi_lib.phase_generator_set_time(
                    self._phase_generator, tail_start)
                num_samples = self._generate_samples(ffi_lib, flush_time)
                if num_samples <= 0:
                    return
                ret = ffi_lib.phase_generator_extract_positions(
                    self._samples, num_samples, self._positions)
                first_pos = self._samples[0].position
                last_pos = self._samples[num_samples - 1].position
                if ret < 0:
                    boundary_time = self._set_idle_boundary(
                        flush_time, label='retry-idle-invalid-samples')
                    logging.info(
                        "phase_gen %s: invalid samples after idle-tail retry"
                        " at flush=%.3f (holding %.3fs lookback to %.3f)",
                        self.name, flush_time,
                        flush_time - boundary_time, boundary_time)
                    return
            if abs(last_pos - first_pos) < 0.5:
                self._enter_idle_hold(flush_time, first_pos, 'standstill')
                return
        resuming_from_idle = self._idle_position is not None
        move_start = None
        trim_start = None
        retried_resume = False
        while True:
            if resuming_from_idle:
                resume_window = self._find_resume_window(num_samples, 1.0)
                if resume_window is not None:
                    trim_start, move_start = resume_window
                    break
                if not retried_resume and self.resume_lookback > 0.0:
                    retry_start = max(0.0, flush_time - self.resume_lookback)
                    if (self._samples[0].time
                            > retry_start + self.update_interval * 0.5):
                        self._trace_event('retry_resume', flush=flush_time,
                                          old_start=self._samples[0].time,
                                          new_start=retry_start)
                        logging.info(
                            "phase_gen %s: retrying idle resume lookback"
                            " old_start=%.3f new_start=%.3f flush=%.3f",
                            self.name, self._samples[0].time,
                            retry_start, flush_time)
                        ffi_lib.phase_generator_set_time(
                            self._phase_generator, retry_start)
                        num_samples = self._generate_samples(
                            ffi_lib, flush_time)
                        if num_samples <= 0:
                            return
                        ret = ffi_lib.phase_generator_extract_positions(
                            self._samples, num_samples, self._positions)
                        first_pos = self._samples[0].position
                        last_pos = self._samples[num_samples - 1].position
                        if ret < 0:
                            boundary_time = self._set_idle_boundary(
                                flush_time, label='retry-invalid-samples')
                            logging.info(
                                "phase_gen %s: invalid samples after retry"
                                " at flush=%.3f (holding %.3fs lookback to %.3f)",
                                self.name, flush_time,
                                flush_time - boundary_time, boundary_time)
                            return
                        retried_resume = True
                        continue
            move_start = ffi_lib.phase_generator_find_move_start(
                self._positions, num_samples, 1.0)
            trim_start = max(0, move_start - 1)
            break
        if move_start >= num_samples:
            idle_pos = (self._idle_position if self._idle_position is not None
                        else first_pos)
            self._enter_idle_hold(flush_time, idle_pos, 'all-standstill flush')
            return
        if resuming_from_idle:
            self._had_zero_samples = False
            self._move_log_count = 0
            self._trace_event('resume', flush=flush_time, trim=trim_start,
                              move_start=move_start, first=first_pos,
                              last=last_pos, retried=retried_resume)
        self._clear_idle_state()
        if trim_start > 0:
            # Preserve one held-phase anchor sample on resume so the first MCU
            # write matches the idle phase before motion begins.
            num_samples -= trim_start
            # Shift position data (C array, manual copy)
            for j in range(num_samples):
                self._positions[j] = self._positions[j + trim_start]
            # Also update the samples array for correct start_time later
            for j in range(num_samples):
                self._samples[j] = self._samples[j + trim_start]
        self._move_log_count += 1
        self._total_log_count += 1
        self._last_commanded_position = self._phase_to_commanded(
            self._positions[num_samples - 1])
        if self._move_log_count <= 5 or self._total_log_count % 100 == 0:
            logging.info("phase_gen %s: samples=%d flush=%.3f first_pos=%.3f"
                         " last_pos=%.3f clock_reset_flag=%s move_flush=%d",
                         self.name, num_samples, flush_time,
                         self._samples[0].position,
                         self._samples[num_samples-1].position,
                         self._needs_clock_reset, self._move_log_count)
        # Compress into quadratic segments
        num_segments = ffi_lib.phase_compressor_compress(
            self._phase_compressor,
            self._positions, num_samples,
            self._segments, MAX_PHASE_SEGMENTS)
        # Convert to fixed-point MCU format (C loop, replaces slow Python loop)
        num_mcu = ffi_lib.phase_compressor_to_fixed(
            self._segments, num_segments,
            self._mcu_moves, MAX_PHASE_SEGMENTS)
        if num_mcu <= 0:
            boundary_time = self._set_idle_boundary(
                flush_time, label='compress-empty')
            logging.info("phase_gen %s: %d segments -> 0 MCU moves"
                         " (holding %.3fs lookback to %.3f)",
                         self.name, num_segments,
                         flush_time - boundary_time, boundary_time)
            return
        # Log first few segment details for diagnostics
        if self._move_log_count <= 3:
            for i in range(min(num_mcu, 3)):
                m = self._mcu_moves[i]
                logging.info("phase_seg %s: seg=%d/%d start=%d vel=%d"
                             " accel=%d count=%d",
                             self.name, i, num_mcu,
                             m.start_position, m.velocity,
                             m.acceleration, m.count)
        # After a drained idle hold, the MCU timer still needs a fresh clock
        # anchor even though the TMC driver may keep holding the last phase.
        interval_ticks = self.mcu.seconds_to_clock(self.update_interval)
        start_time = self._samples[0].time
        start_clock = self.mcu.print_time_to_clock(start_time)
        need_reset = self._needs_clock_reset
        if need_reset:
            clock = start_clock
            self._reset_cmd.send([self.oid, clock])
            self._needs_clock_reset = False
        # Send MCU commands and track total interval count for drain detection
        total_count = 0
        for i in range(num_mcu):
            m = self._mcu_moves[i]
            self._queue_cmd.send([self.oid, interval_ticks,
                                  m.start_position, m.velocity,
                                  m.acceleration, m.count])
            total_count += m.count
        self._expected_end_clock = start_clock + total_count * interval_ticks
        self._mcu_hold_active = True
        if self._activation_pending:
            self._trace_event('activate_first_emit',
                              start=start_time, flush=flush_time,
                              first=self._positions[0],
                              last=self._positions[num_samples - 1],
                              reset=need_reset)
            self._activation_pending = False
        self._trace_event('queue_move', start=start_time, flush=flush_time,
                          samples=num_samples, mcu=num_mcu,
                          first=self._positions[0],
                          last=self._positions[num_samples - 1],
                          reset=need_reset)

class TMCPhaseStepping:
    """Klipper module for configuring phase stepping on TMC5160 drivers."""
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name()
        self.phase_steppers = {}
        self.resume_delay = config.getfloat('phase_resume_delay', 1.0,
                                            minval=0.0)
        self._resume_generation = 0
        self._resume_scheduled_generation = 0
        self._home_rails_depth = 0
        self._homing_move_depth = 0
        self._phase_stepping_enabled = False
        self._probe_resume_deferred = False
        self._fault_pending = False
        self._resume_timer = self.reactor.register_timer(
            self._resume_event, self.reactor.NEVER)
        self.printer.register_event_handler('klippy:ready',
                                            self._handle_ready)
        self.printer.register_event_handler(
            'homing:homing_move_begin', self._handle_homing_move_begin)
        self.printer.register_event_handler(
            'homing:homing_move_end', self._handle_homing_move_end)
        self.printer.register_event_handler(
            'homing:home_rails_begin', self._handle_home_rails_begin)
        self.printer.register_event_handler(
            'homing:home_rails_end', self._handle_home_rails_end)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('PHASE_STEPPER_DEBUG', self.cmd_DEBUG,
                               desc="Query non-disruptive phase stepper state")
        gcode.register_command('PHASE_STEPPER_SUSPEND', self.cmd_SUSPEND,
                               desc="Suspend phase stepping immediately")
        gcode.register_command('PHASE_STEPPER_RESUME', self.cmd_RESUME,
                               desc="Resume phase stepping now or after delay")
        gcode.register_command('PHASE_STEPPER_TRACE', self.cmd_TRACE,
                               desc="Dump recent host-side phase-stepper trace")
        gcode.register_command('PHASE_STEPPER_STATUS', self.cmd_STATUS,
                               desc="Query phase stepper diagnostics")
        gcode.register_command('TEST_XDIRECT', self.cmd_TEST_XDIRECT,
                               desc="Manual XDIRECT test on first stepper")
    def _activate_all(self):
        # Activate all phase steppers without forcing a cross-motor stagger.
        # The MCU scheduler keeps independent due times on one shared clock.
        if self._phase_stepping_enabled:
            return
        for name, ps in self.phase_steppers.items():
            ps.activate(stagger_ticks=0)
        self._phase_stepping_enabled = True
    def _handle_ready(self):
        pass  # Don't activate at boot — wait for first homing cycle
    def _deactivate_all(self, reason, flush_toolhead=False):
        self._cancel_resume(reason)
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
    def _has_external_probe_flow(self):
        for obj_name, _obj in self.printer.lookup_objects():
            if obj_name == "beacon" or obj_name.startswith("beacon "):
                return True
            if obj_name == "beacons" or obj_name.startswith("beacons "):
                return True
        return False
    def _handle_phase_fault(self, name, reason):
        if self._fault_pending:
            return
        self._fault_pending = True
        def do_fault(_eventtime):
            self._fault_pending = False
            self._deactivate_all("phase fault: %s (%s)" % (name, reason),
                                 flush_toolhead=True)
        self.reactor.register_callback(do_fault)
    def _cancel_resume(self, reason):
        self._resume_generation += 1
        self._resume_scheduled_generation = 0
        self.reactor.update_timer(self._resume_timer, self.reactor.NEVER)
        logging.info("Phase stepping resume canceled: %s", reason)
    def _schedule_resume(self, reason, delay=None):
        if delay is None:
            delay = self.resume_delay
        self._resume_generation += 1
        self._resume_scheduled_generation = self._resume_generation
        self._probe_resume_deferred = False
        waketime = self.reactor.monotonic() + delay
        self.reactor.update_timer(self._resume_timer, waketime)
        logging.info("Phase stepping resume scheduled in %.3fs: %s",
                     delay, reason)
    def _is_probe_busy(self):
        probe = self.printer.lookup_object("probe", None)
        if probe is None:
            return False
        return bool(getattr(probe, "multi_probe_pending", False))
    def _buffered_motion_time(self):
        toolhead = self.printer.lookup_object("toolhead")
        if getattr(toolhead, "special_queuing_state", ""):
            return 0.0
        eventtime = self.reactor.monotonic()
        print_time, est_print_time, _lookahead_empty = toolhead.check_busy(
            eventtime)
        return print_time - est_print_time
    def _resume_event(self, eventtime):
        scheduled_generation = self._resume_scheduled_generation
        if not scheduled_generation:
            return self.reactor.NEVER
        if scheduled_generation != self._resume_generation:
            self._resume_scheduled_generation = 0
            return self.reactor.NEVER
        if self._home_rails_depth or self._homing_move_depth:
            logging.info("Phase stepping resume skipped:"
                         " home_rails_depth=%d homing_move_depth=%d",
                         self._home_rails_depth, self._homing_move_depth)
            return self.reactor.NEVER
        if self._is_probe_busy():
            if not self._probe_resume_deferred:
                logging.info("Phase stepping resume deferred:"
                             " probe multi_probe still active")
                self._probe_resume_deferred = True
            return eventtime + 0.250
        buffered_motion = self._buffered_motion_time()
        if buffered_motion > 0.050:
            logging.info("Phase stepping resume deferred:"
                         " %.3fs of motion still buffered",
                         buffered_motion)
            return eventtime + min(0.250, buffered_motion)
        self._resume_scheduled_generation = 0
        self._probe_resume_deferred = False
        logging.info("Phase stepping resume fired")
        self._activate_all()
        return self.reactor.NEVER
    def _handle_homing_move_begin(self, hmove):
        self._homing_move_depth += 1
        self._deactivate_all("homing_move_begin")
    def _handle_homing_move_end(self, hmove):
        if self._homing_move_depth:
            self._homing_move_depth -= 1
        if self._home_rails_depth or self._homing_move_depth:
            return
        if self._has_external_probe_flow():
            logging.info("Phase stepping auto-resume suppressed:"
                         " external Beacon/probe flow present")
            return
        self._schedule_resume("homing_move_end")
    def _handle_home_rails_begin(self, homing_state, rails):
        self._home_rails_depth += 1
        self._deactivate_all("home_rails_begin")
    def _handle_home_rails_end(self, homing_state, rails):
        if self._home_rails_depth:
            self._home_rails_depth -= 1
        if self._home_rails_depth or self._homing_move_depth:
            return
        if self._has_external_probe_flow():
            logging.info("Phase stepping auto-resume suppressed:"
                         " external Beacon/probe flow present")
            return
        # Beacon and other probe flows can continue after home_rails_end.
        # Delay reactivation and allow a new homing/probe begin to cancel it.
        self._schedule_resume("home_rails_end")
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
            'mscuract_a': cur_a,
            'mscuract_b': cur_b,
            'drv_status': drv_status,
            'gstat': gstat,
            'live_faststandstill': fields.get_field(
                "faststandstill", live_gconf, "GCONF"),
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
        # Flag clock reset and fast-forward generator time since we stopped
        toolhead = self.printer.lookup_object("toolhead")
        cur_time = toolhead.get_last_move_time()
        for name, ps in self.phase_steppers.items():
            if was_active[name]:
                ps._had_zero_samples = True
                ps._set_idle_boundary(cur_time, label='status-stop')
    def cmd_DEBUG(self, gcmd):
        """PHASE_STEPPER_DEBUG - non-disruptive host/MCU state dump.
        MOTOR=stepper_x optionally limits output to one motor.
        MCU=1 also queries MCU-side counters. TMC=1 also reads live TMC
        standstill/current state."""
        if not self.phase_steppers:
            gcmd.respond_info("No phase steppers registered")
            return
        query_mcu = gcmd.get_int("MCU", 0, minval=0, maxval=1)
        query_tmc = gcmd.get_int("TMC", 0, minval=0, maxval=1)
        motor_filter = gcmd.get("MOTOR", None)
        if motor_filter and motor_filter not in self.phase_steppers:
            gcmd.respond_info("Unknown motor '%s'. Available: %s"
                              % (motor_filter,
                                 ", ".join(sorted(self.phase_steppers))))
            return
        names = ([motor_filter] if motor_filter
                 else sorted(self.phase_steppers.keys()))
        gcmd.respond_info("phase_resume_pending=%s phase_resume_delay=%.3f"
                          " mcu_query=%s tmc_query=%s home_rails_depth=%d"
                          " homing_move_depth=%d probe_pending=%s"
                          " buffered_motion=%.3f"
                          " beacon_present=%s"
                          % (bool(self._resume_scheduled_generation),
                             self.resume_delay, bool(query_mcu),
                             bool(query_tmc),
                             self._home_rails_depth,
                             self._homing_move_depth,
                             self._is_probe_busy(),
                             max(0.0, self._buffered_motion_time()),
                             self._has_external_probe_flow()))
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
            gcmd.respond_info(
                "%s: active=%s needs_reset=%s expected_end_clock=%d"
                " update_hz=%.0f idle_lookback=%.3f resume_lookback=%.3f"
                " idle_hold_cfg=%.3f"
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
                " live_faststandstill=%s" % (
                    name, ps._phase_stepping_active,
                    ps._needs_clock_reset, ps._expected_end_clock,
                    ps.update_rate, ps.idle_lookback,
                    ps.resume_lookback,
                    ps.idle_hold_time,
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
                    tmc_state.get('live_faststandstill', 'n/a')))
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
        """PHASE_STEPPER_RESUME - re-enable phase stepping.
        DELAY=<seconds> optionally defers activation."""
        delay = gcmd.get_float("DELAY", 0.0, minval=0.0)
        if delay > 0.0:
            self._schedule_resume("gcode resume", delay)
            gcmd.respond_info("Phase stepping resume scheduled in %.3fs"
                              % (delay,))
            return
        buffered_motion = self._buffered_motion_time()
        if buffered_motion > 0.050:
            self._schedule_resume("gcode resume waiting for idle", 0.0)
            gcmd.respond_info("Phase stepping resume deferred until motion is"
                              " idle")
            return
        self._cancel_resume("gcode resume immediate")
        self._activate_all()
        gcmd.respond_info("Phase stepping resumed")
    def cmd_TEST_XDIRECT(self, gcmd):
        """Drive motors through microsteps via direct_mode.
        STEPS=N (default 10000, negative for reverse direction)
        MOTOR=stepper_x (optional, test single motor only)
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
        # Stop phase stepping ISR if active (we'll do our own writes)
        for ps in self.phase_steppers.values():
            if ps._phase_stepping_active:
                ps.stop_timer()
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
            mcu_tmc.set_register("XTARGET", val)
            gcmd.respond_info(
                "%s: MSCNT=%d IRUN=%d preload A=%d B=%d source=%s"
                % (name, mscnt, irun, a, b, source))
        # Determine per-motor MSCNT direction from dir_pin inversion.
        # Motors with inverted dir_pin need -MSCNT to match their belt
        # partner's +MSCNT (they're physically mounted opposite).
        motor_dirs = {}
        for name in test_names:
            ps = self.phase_steppers[name]
            invert = ps.stepper.get_dir_inverted()[0]
            motor_dirs[name] = -1 if invert else 1
            gcmd.respond_info(
                "%s: invert_dir=%s motor_dir=%+d"
                % (name, invert, motor_dirs[name]))
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
        gcmd.respond_info("Done. Restoring registers...")
        # Restore GCONF and IHOLD_IRUN from config cache
        for name in test_names:
            ps = self.phase_steppers[name]
            mcu_tmc = ps.tmc_obj.mcu_tmc
            mcu_tmc.set_register("GCONF", saved_gconf[name])
            fields = mcu_tmc.fields
            ihr_cached = fields.registers.get("IHOLD_IRUN", None)
            if ihr_cached is not None:
                mcu_tmc.set_register("IHOLD_IRUN", ihr_cached)
        gcmd.respond_info("TEST_XDIRECT done.")
    def register_phase_stepper(self, name, phase_stepper):
        phase_stepper.set_fault_handler(self._handle_phase_fault)
        self.phase_steppers[name] = phase_stepper

def load_config(config):
    return TMCPhaseStepping(config)
