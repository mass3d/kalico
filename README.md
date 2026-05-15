<p align="center"><a href="https://docs.kalico.gg"><img align="center" src="docs/logo/kalico-big.png" alt="Kalico Logo"></a></p>

# Kalico — Phase Stepping & S-Curve Motion Fork

A development fork of [Kalico](https://github.com/KalicoCrew/kalico) that adds:

1. **TMC phase stepping** — streams microsteps directly into TMC2130 / TMC5160 drivers over SPI every tick, instead of pulsing step/dir pins.
2. **S-curve (jerk-limited) motion planning** — 7-phase acceleration profiles with continuous third-order extrusion for cleaner pressure-advance behaviour.

---

## Requirements

- **TMC2130 or TMC5160 driver.** These have driver hooks in this fork for the `XDIRECT` register that phase stepping writes to. TMC2240 has an equivalent direct register over its SPI variant but no driver hook in this fork yet — adding it would parallel the TMC5160 implementation. TMC2208 / 2209 (UART-only, no `XDIRECT`) are not supported.
- **SPI between MCU and driver, hardware SPI for the DMA path.** Software SPI (bit-banged GPIO) works only with the polled fallback — DMA cannot drive a software SPI peripheral. Polled on software SPI runs but sits at the lowest update-rate tier.
- **STM32H7 MCU for the DMA path.** The H7 has a DMA-coherent SRAM region (D2 SRAM at `0x30000000`) used by the DMA SPI path. Other MCUs (STM32F4 / G4 / RP2040) fall back to polled SPI with a markedly lower update-rate ceiling.

S-curve motion has no extra hardware requirements.

Tested on an Octopus Pro running STM32H7 with TMC5160 drivers on hardware SPI.

---

## Installation

Same flow as any Kalico install.

```bash
mv ~/klipper ~/klipper_old
git clone https://github.com/<your-fork>/kalico ~/klipper
~/klippy-env/bin/pip install -r ~/klipper/scripts/klippy-requirements.txt
sudo systemctl restart klipper
```

Rebuild and flash MCU firmware (phase stepping adds new MCU commands, so a re-flash is required the first time):

```bash
cd ~/klipper
make menuconfig          # confirm board, exit
make clean && make -j4
strings -a out/klipper.elf | grep "queue_phase_move oid"   # must print
make flash FLASH_DEVICE=...   # or katapult for CAN
```

If `strings` prints nothing, the new MCU source did not compile — verify `src/Makefile` includes `tmc_phase_stepper.c`.

After Klipper restarts, `klippy.log` should show a line like:

```
Phase stepping ...: HOST_PHASE_STEPPER_VER=v15-chain-prepare-skip MCU firmware PHASE_STEPPER_VER=v15-chain-prepare-skip
```

If the MCU version is `missing` or older than the host version, the MCU was not reflashed.

---

## Configuration

Phase stepping and s-curve motion are both off by default. Each is enabled independently.

### Phase stepping

Enable per stepper, under each TMC driver section:

```ini
[tmc5160 stepper_x]
phase_stepping: true

[tmc5160 stepper_x1]
phase_stepping: true

[tmc5160 stepper_y]
phase_stepping: true

[tmc5160 stepper_y1]
phase_stepping: true
```

For AWD configurations where two motors share a belt (dual-X, dual-Y on CoreXY), add a `[tmc_phase_stepping]` section so paired motors receive an atomic clock reset:

```ini
[tmc_phase_stepping]
phase_pair: stepper_x, stepper_x1
phase_pair2: stepper_y, stepper_y1
# phase_pair3, phase_pair4, … for more pairs
```

The `[tmc_phase_stepping]` section is otherwise optional — without it the host still loads the module and registers the diagnostic G-code commands as soon as any stepper has `phase_stepping: true`.

### Per-stepper tuning (optional)

These keys live under each individual TMC section. All optional.

| Key | Default | Range / Notes |
|---|---|---|
| `phase_stepping` | `false` | Master switch for this motor. |
| `phase_update_rate` | `5000` Hz | 1000-50000. Tick rate for XDIRECT writes. The host may reduce this if it exceeds chain-time or bus-load caps. |
| `phase_spi_max_bus_load` | `0` (auto) | 0.05-0.80. Fraction of SPI bus time the group ISR may consume. `0` = auto (0.50 on DMA build, 0.20 on polled). |
| `phase_chain_per_motor_overhead` | `0` (auto) | seconds. Per-motor DMA chain overhead. `0` = auto (1.2 µs DMA, 1.4 µs polled). Tune downward only if your hardware measures faster. |
| `phase_bus` | `0` | 0-3. Logical bus index for multi-bus setups. |
| `phase_active_tpowerdown` | `255` | TPOWERDOWN held during phase stepping to keep the chopper from dropping standstill current mid-print. |
| `phase_disable_faststandstill` | `true` | Disable TMC fast-standstill while phase stepping (avoids spurious triggers on the held XDIRECT vector). |
| `phase_compress_error` | `0.25` | Max position error (microsteps) the anchored compressor allows when fitting motion segments. |
| `phase_direction_override` | `0` | `-1`/`+1` to force electrical phase sign; `0` = auto from `invert_dir`. |

### S-curve (jerk-limited) motion

```ini
[printer]
# 0 = legacy trapezoidal (upstream behaviour, fully backward-compatible).
# Non-zero = 7-phase s-curve with cubic extrusion. Reasonable starting
# point: max_accel × 10. Units: mm/s^3.
max_jerk: 0

[extruder]
# Optional separate cap for extrude-only moves (no XY, no retraction).
# 0 = no separate cap.
max_extrude_only_jerk: 0
```

Z-only moves stay on the trapezoidal path regardless of `max_jerk`. The hard-stop rule between extruding and non-extruding moves (matching RepRapFirmware behaviour) is enforced under `max_jerk > 0` to preserve PA continuity across print/Z-hop/print sequences.

### Build-time options (Kconfig)

Auto-resolve sensibly on STM32H7; verify after `make menuconfig` with `grep PHASE_STEPPER ~/klipper/.config`.

| Symbol | Default on STM32H7 | Default elsewhere | Purpose |
|---|---|---|---|
| `WANT_SPI_DMA` | `y` | `n` | Builds the DMA SPI primitive. |
| `PHASE_STEPPER_EXPERIMENTAL_DMA` | `y` | `n` | Selects DMA over polled SPI for phase-stepping writes. |
| `HAVE_DMA_BUF_REGION` | `y` (auto) | `n` (auto) | Provides D2 SRAM for DMA-coherent buffers. |

---

## G-code commands

| Command | Purpose |
|---|---|
| `PHASE_STEPPER_SUSPEND` | Hand control back to step/dir mode. Required before `G28` (see homing workflow). |
| `PHASE_STEPPER_RESUME` | Re-enter phase stepping. |
| `PHASE_STEPPER_DEBUG MCU=1 TMC=1` | Per-motor state dump: anchor, write/skip counts, MCU clocks, TMC driver registers. The main diagnostic. |
| `PHASE_STEPPER_STATUS` | Concise summary of update rate, bus load, chain-time fill, and which guard (if any) is capping the rate. |
| `PHASE_STEPPER_TRACE` | Dump the host-side trace ring buffer (verbose). |
| `PHASE_STEPPER_SET_DIRECTION STEPPER=<name> SIGN=<-1\|+1>` | Override phase direction at runtime without restarting. |
| `TEST_XDIRECT` | Bench test: write specific XDIRECT values. Args: `STEPS=<n>`, `MOTOR=<name>`, `SIGN={-1,0,+1}`, `DWELL=<sec>`, `RESTORE=1`. Run after homing. |

### Homing workflow

Phase stepping does not auto-suspend around homing. The intentional workflow:

```gcode
PHASE_STEPPER_SUSPEND
G28
PHASE_STEPPER_RESUME
```

Typically wrapped in a `START_PRINT` macro.

---

## How the update rate is limited

Smoothness, top speed, and SPI bandwidth are coupled through one equation. Units throughout are TMC phase positions (pp): 1024 pp per electrical revolution = 256 pp per fullstep on a standard 1.8° motor.

### Per-write phase jump

Each tick the firmware writes one `XDIRECT` vector per motor. Between writes the motor holds the last commanded phase — the TMC does not interpolate during phase stepping.

```
pp_per_write = motor_velocity_in_pp_per_sec / update_rate
```

Rough thresholds (motor-dependent — calibrate by sweeping velocity on your printer):

- below ~4 pp/write — smooth
- 5-8 pp/write — tolerable, audible texture
- above ~8 pp/write — motor can't follow electromagnetically; skips and audible roughness

The exact threshold depends on motor inertia, holding current, and load.

### Update rate is bounded by SPI bandwidth

Every tick each motor receives a 5-byte SPI burst. Motors on a shared SPI bus transfer sequentially — one motor's burst finishes before the next begins, because each motor has its own chip-select line.

```
chain_time_per_tick = N_motors × (byte_time + per_motor_overhead)
byte_time           = 5 × 8 / SPI_clock
per_motor_overhead  ≈ 1.2 µs (DMA) / 1.4 µs (polled)   on H7 @ 520 MHz
```

The chain must fit inside one tick period or the firmware drops writes:

```
update_rate_max ≤ 1 / chain_time_per_tick
```

If the configured rate exceeds this, the firmware enters a 50%-effective-rate skip pattern (chain spills into next tick, next tick is dropped). The host's chain-time guard refuses such configurations.

### SPI clock is bounded by the TMC's internal oscillator

TMC2130 / 5160 typically run with `CLK` grounded on retail boards, which forces the internal oscillator (datasheet 10-14 MHz, typical 12 MHz). SPI clock spec:

- `SCK ≤ fCLK / 2` for writes (the only direction phase stepping uses)
- `SCK ≤ fCLK / 4` for reads (TMC register queries from the host)

That gives ~6 MHz SPI at typical `fCLK = 12 MHz`. 8 MHz is over spec but often works (margin in the IC, lot variation). 10+ MHz reliably needs an external oscillator on the TMC's `CLK` pin, which almost no retail boards expose.

### Worked example: from configuration to top speed

For a 4-motor AWD setup on one shared SPI bus at 6 MHz, DMA path:

```
byte_time           = 5 × 8 / 6e6      = 6.67 µs
chain_time_per_tick = 4 × (6.67 + 1.2) = 31.5 µs
update_rate_max     = 1 / 31.5e-6      = 31.7 kHz   →  round to 30 kHz
```

To turn that into a mm/s ceiling you need your printer's `pp_per_mm`. Measure it:

1. Run a single-axis move at a known velocity, e.g. `G1 X100 F12000` (200 mm/s on X).
2. Read `max_abs_vel` from `PHASE_STEPPER_DEBUG` (16.16 fixed-point, units of pp per tick).
3. Compute:

```
pp_per_sec = (max_abs_vel / 65536) × update_hz
pp_per_mm  = pp_per_sec / velocity_mm_per_sec
```

Then the smooth/tolerable velocity ceilings:

```
smooth_mm_per_sec    = update_rate_max × 4 / pp_per_mm
tolerable_mm_per_sec = update_rate_max × 8 / pp_per_mm
```

Worked example using a value the dev machine's `PHASE_STEPPER_DEBUG` log implied (`max_abs_vel = 704643` at 25 kHz during a TEST_SPEED 800 mm/s run, treated as 800 mm/s motor speed → `pp_per_mm ≈ 336`). **This number is illustrative only** — it was inferred from a log snapshot rather than calibrated, and may not represent the printer's actual `pp_per_mm` if the snapshot landed during a slower-than-peak segment. Run the measurement procedure above on your printer for the real value.

```
smooth_mm_per_sec    = 30000 × 4 / 336 ≈ 357 mm/s
tolerable_mm_per_sec = 30000 × 8 / 336 ≈ 714 mm/s
```

A printer with a higher `pp_per_mm` (smaller pulley, larger gearing) has a lower mm/s ceiling in proportion; lower `pp_per_mm` raises it.

### Why per-driver hardware SPI is the architectural way out

The bandwidth equation has three free variables: `N_motors`, `SPI_clock`, and `update_rate`. Two are practically pinned — `SPI_clock` is capped by the TMC's internal oscillator and `N_motors` is determined by the printer. The remaining freedom is to reduce `N_motors_per_bus` by giving each driver its own SPI peripheral. With one motor per bus, the chains run in parallel and the bandwidth equation becomes:

```
chain_time_per_tick = 1 × (6.67 + 1.2) ≈ 7.9 µs
update_rate_max     ≈ 1 / 7.9e-6        ≈ 127 kHz
```

Using the same illustrative `pp_per_mm = 336`:

```
smooth_mm_per_sec    = 127000 × 4 / 336 ≈ 1500 mm/s
```

The firmware already supports parallel buses via the per-stepper `phase_bus` config; the bottleneck is hardware — a board (or rewired one) that exposes one MCU SPI peripheral per TMC driver.

---

## Architecture

### Phase stepping pipeline

```
toolhead trapq ──► host segment generator ──► anchored compressor (C) ──┐
                                                                         │
            ┌────────────────────────────────────────────────────────────┘
            ▼
  host mirror (_mcu_anchor_pos, int 16.16)
            │   tracks the MCU's ps->position bit-exactly via the
            │   closed form  start + count*vel + count*(count-1)/2 * accel
            ▼
      queue_phase_move ──► MCU per-motor segment queue
                                       │
                                       ▼
                       group_phase_stepper_event (timer ISR, every tick)
                                       │
                                       ▼
                spi_dma_kick_tx (H7) or spidev_transfer (polled fallback)
                                       │
                                       ▼
                              TMC XDIRECT register
```

Each motion segment supplies `start_position`, `velocity`, `acceleration`, and `count` (ticks). The MCU advances per tick as `position += velocity; velocity += acceleration`. The host keeps a bit-exact mirror of the same arithmetic so that segment K+1's `start_position` can be forced equal to where segment K leaves off — no float drift, no boundary discontinuity.

### Continuous emission

The group ISR runs every tick from activation to deactivation. When a motor's segment queue empties, `phase_stepper_load_next` zeros `vel` and `accel` and leaves `count` at 0. The polynomial holds; the same XDIRECT bytes are re-written every tick. "Idle" is just a segment with `vel=0, accel=0, count=N`, emitted naturally by the compressor when the kinematic position doesn't change. The next motion segment after idle starts at exactly the held anchor by construction — no resume discontinuity.

### S-curve motion

Enabled when `[printer] max_jerk > 0` and the move is kinematic. `_set_junction_scurve` builds the 7-phase profile (accel-jerk / accel-const / accel-jerk-out / cruise / decel-jerk / decel-const / decel-jerk-in). `trapq_append_scurve` lays the same profile onto the toolhead trapq and the extruder trapq, so XYZE move as a single coordinated third-order curve.

Rules under `max_jerk > 0`:

- Z-only moves stay on the trapezoidal path (single-axis Z motion gains nothing from s-curve).
- Print↔travel boundaries are hard stops. `Move.calc_junction` forces `max_start_v2 = 0` when crossing extrude/non-extrude, matching RepRapFirmware. Preserves PA continuity across print/Z-hop/print.
- `Move.max_jerk` can be lowered per-move via `move.limit_jerk(j)`. Extrude-only moves are capped by `max_extrude_only_jerk` when set.

### Pressure advance with third-order motion

`kin_extruder.c::extruder_integrate` and `extruder_integrate_time` include a `sixth_jerk` term (antiderivatives gain `t^4/4 × sixth_jerk` and `t^5/5 × sixth_jerk` respectively). `pa_move_integrate` adds `half_accel += PA × 3 × sixth_jerk` alongside the existing `start_v += PA × 2 × half_accel`. This is what makes "third-order extrusion with PA applied" produce a second-order velocity profile instead of a first-order discontinuity at junctions — the difference between visible PA ringing on every corner and clean walls.

### AWD pair atomic clock reset

The MCU exposes `reset_phase_clock_group oids=%*s clock=%u`, which resets multiple motors' position counters atomically inside one `irq_disable()` window. `_resolve_pairs` reads the `phase_pair` specs at first activation and binds each motor to its partner. With continuous emission, resets are rare — only at activate and after `PHASE_STEPPER_SUSPEND`/`RESUME`.

### Two update-rate guards

The host enforces two independent caps; whichever is tighter wins.

1. **Bus-load cap** (`phase_spi_max_bus_load`): fraction of bus time the group ISR may consume. Protects headroom for non-phase-stepping users (TMC register reads, ADCs, other SPI devices).
2. **Chain-time cap** (automatic): ensures the per-tick chain fits inside the tick period; refuses configs that would produce the 50% skip pattern described above.

`klippy.log` reports which guard fired (`rate_source=spi_load_guard` or `rate_source=chain_time_guard`). The `PHASE_STEPPER_STATUS` G-code prints the same.

### DMA SPI on STM32H7

`src/stm32/stm32h7_spi.c` provides `spi_dma_kick_tx` and DMA-TC IRQ handlers for streams 0-4 (one per SPI peripheral, SPI1-SPI5 TX). DMAMUX request lines:

| Peripheral | DMAMUX request |
|---|---|
| SPI1 TX | 38 |
| SPI2 TX | 40 |
| SPI3 TX | 62 |
| SPI4 TX | 84 |
| SPI5 TX | 86 |

NVIC priority 1 for DMA-TC IRQs (matches USB/CAN convention, one tier above SysTick). DMA buffers live in D2 SRAM at `0x30000000` via the `.dma_buf` linker section. STM32H7's `RAM_START = 0x20000000` is DTCM (CPU-private, *not* DMA-reachable) so default `.bss` placement would silently fail — `.dma_buf` exists specifically to put DMA-sourced data in an addressable region.

Chained motors on the same SPI peripheral skip the redundant `spi_prepare` between transfers (all motors on a shared bus have identical CFG1/CFG2). Inflight bus state is protected by a global `spi_bus_busy` flag; concurrent TMC register reads from the host wait for any in-flight phase-stepping DMA to drain before claiming the bus.

### Polled SPI fallback

On MCUs without DMA-coherent SRAM, `spidev_kick_dma_tx` falls through to `spidev_transfer` synchronously inside the group ISR and fires the completion callback immediately. Same correctness, but the CPU is blocked for the duration of each transfer, so the practical update-rate ceiling is much lower (~5-10 kHz).

### Not yet implemented

- **Per-motor sine compensation.** The firmware uses one shared 256-entry sine LUT (`sine_table` in `src/tmc_phase_stepper.c`) for every motor; only `current_scale` is per-motor. Prusa-style harmonic suppression — where each motor gets its own correction table derived from a velocity sweep with an accelerometer or current monitor — slots cleanly onto this architecture but is not present in this fork. Hooks would be a per-motor correction array applied to the LUT lookup in the XDIRECT build path, plus a host calibration routine that drives the sweep and writes the correction table.
- **Per-driver parallel SPI buses.** The `phase_bus` config key is plumbed and the firmware can run multiple buses in parallel, but no retail board exposes one hardware SPI peripheral per TMC driver. Lifting the single-bus update-rate ceiling needs either a custom board or rewiring.

---

## Verification

### Host self-tests (no MCU required)

```bash
~/klippy-env/bin/python tests/test_phase_mirror.py
~/klippy-env/bin/python tests/test_phase_compress_anchored.py
~/klippy-env/bin/python tests/test_scurve.py
```

- `test_phase_mirror.py` — closed-form `_advance_mirror` vs N-tick simulation; 11000 trials including int32 overflow paths.
- `test_phase_compress_anchored.py` — bit-exact segment-chain continuity; 200 fuzz trials and 6 boundary cases.
- `test_scurve.py` — s-curve continuity, phase compression, extruder retraction direction, PA-with-jerk integral correctness, print↔travel hard-stop rule (16 tests).

### Hardware checks

| Check | Procedure | Pass |
|---|---|---|
| Click test | `G1 Y20 F6000`, `G4 P5000`, `G1 Y200 F6000`. Repeat with waits of 0.1 / 1 / 5 / 30 / 300 s. | Silent at every wait. |
| Continuous emission | Run `PHASE_STEPPER_DEBUG MCU=1 TMC=1` during and after motion. | `write_count / event_count ≥ 0.99`. |
| AWD pair sync | With `phase_pair` configured: `G1 X20 Y20`, wait, `G1 X200 Y200`. | Both axes click-free. |
| M84 / G28 cycle | 100 print/M84/wait/home/print cycles. | No clicks; phase stepping resumes cleanly each cycle. |
| Find your ceiling | Sweep travel feedrate upward in 100 mm/s steps while watching `skip_count` and listening. | Note the threshold where skip count climbs or audible texture appears. |
| S-curve corner test | With `max_jerk = max_accel × 10`, print a 10 mm cube. Compare PA ringing to a `max_jerk = 0` baseline. | Less corner ringing under jerk-limited motion. |

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `Unknown command queue_phase_move` at startup | New MCU sources didn't compile. Verify `src/Makefile` includes `tmc_phase_stepper.c`, rebuild and re-flash. |
| Config parser errors on `phase_bus_slots`, `phase_bus_slot`, `phase_idle_lookback`, `phase_resume_lookback` | Stale config from earlier revisions. Remove those keys. |
| `write_count` ≈ `skip_count` / 2 (50% effective rate) | DMA chain exceeded tick period. Lower `phase_update_rate`, raise SPI speed, or tune `phase_chain_per_motor_overhead`. The chain-time guard should normally catch this before activation; if it didn't, your hardware's overhead is higher than the default estimate. |
| `klippy.log` shows `rate_source=spi_load_guard` and you want more rate | Raise `phase_spi_max_bus_load` (max 0.80) or raise TMC `spi_speed` (within datasheet limits). |
| `klippy.log` shows `rate_source=chain_time_guard` and you want more rate | Raise TMC `spi_speed`, reduce motors per bus, or — last resort — lower `phase_chain_per_motor_overhead`. |
| Click on every move start, motor turns wrong direction | Phase direction sign mismatch. `PHASE_STEPPER_SET_DIRECTION STEPPER=<name> SIGN=±1` to override at runtime, then persist via `invert_dir`. |
| Click after `M84` or `idle_timeout` | Phase stepping was disabled. Re-issue `PHASE_STEPPER_RESUME`. |
| Roughness or skips appear at high feedrates | At or above the architectural per-write-jump limit. Either keep that motion below threshold, or move the drivers to per-driver hardware SPI to lift the update-rate ceiling. |
| MCU firmware version line missing or older than host | MCU was not reflashed. `make clean && make -j4`, then re-flash. |

---

## What this fork inherits from upstream Kalico

Everything not described above is straight upstream [Kalico](https://github.com/KalicoCrew/kalico): MPC and velocity-PID heaters, dockable probes, sensorless homing extensions, `gcode_shell_command`, `danger_options`, the bleeding-edge motion improvements, and the rest of the features described in [docs/Kalico_Additions.md](docs/Kalico_Additions.md).

Upstream Kalico documentation: [docs.kalico.gg](https://docs.kalico.gg).

---

## License

Kalico (and this fork) is Free Software. See the [license](COPYING).

## Acknowledgements

- The [Kalico](https://github.com/KalicoCrew/kalico) and [Klipper](https://github.com/Klipper3d/klipper) communities for the firmware this is built on.
- The TMC2130 / TMC5160 datasheets and Prusa's reference XDIRECT handoff sequence.
- RepRapFirmware for the jerk-limited motion model and the print/travel hard-stop rule.

[![Join Kalico on Discord](https://discord.com/api/guilds/1297243471442214913/widget.png?style=banner2)](https://kalico.gg/discord)
