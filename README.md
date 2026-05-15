<p align="center"><a href="https://docs.kalico.gg"><img align="center" src="docs/logo/kalico-big.png" alt="Kalico Logo"></a></p>

# Kalico — Phase Stepping & S-Curve Motion Fork

This fork adds two large features on top of the upstream [Kalico](https://github.com/KalicoCrew/kalico) firmware:

1. **TMC phase stepping** — silent, high-speed motion by streaming microsteps directly into the TMC2130/TMC5160 driver over SPI every tick, instead of pulsing step/dir pins.
2. **S-curve (jerk-limited) motion planning** — smoother acceleration with continuous third-order extrusion, so pressure advance does not ring at corners.

If you already know what those mean, jump to [Installation](#installation) or [Configuration](#configuration).
If not, the next section explains what these change for your printer.

---

## What you get

**In plain terms**

| You will notice | How it happens |
|---|---|
| Quieter motors at every speed | Microsteps flow as analog-like coil currents instead of step pulses, so there is no detent click at each step |
| Higher reliable top speed | Step/dir hits a hardware ceiling at high microstepping; phase stepping bypasses it |
| Smoother corners | S-curve motion ramps acceleration in and out instead of applying it instantly, so the frame does not "punch" through direction changes |
| Cleaner pressure advance | The extruder now follows the same smooth curve as XY, so PA does not over- or under-shoot at junctions |
| Click-free pauses and resumes | The continuous-emission scheduler keeps writing the same microstep position during idle, so the driver never "snaps" back when motion restarts |

**Should you use this fork?**

| If your printer has... | This fork is... |
|---|---|
| TMC2130 or TMC5160 drivers on SPI, **and** an STM32H7 MCU (Octopus Pro / Manta M8P / etc.) | the recommended setup — you get every feature including DMA-accelerated SPI |
| TMC2130 / 5160 on SPI, but an STM32F4 / G4 / RP2040 MCU | usable — phase stepping runs on the **polled** SPI fallback, capped at roughly 10 kHz update rate |
| TMC2208 / 2209 (UART-only) drivers | not supported — those drivers do not expose the XDIRECT register that phase stepping streams to. S-curve motion still works. |
| No TMC drivers at all | s-curve motion is still useful. Phase stepping does not apply. |

---

## Installation

This fork is installed exactly like any other Klipper/Kalico distribution.

```bash
# Backup the existing install
mv ~/klipper ~/klipper_old

# Clone this fork
git clone https://github.com/mass3d/kalico.git ~/klipper
sudo systemctl restart klipper
```

After updating, make sure your Python environment has all required packages:

```bash
~/klippy-env/bin/pip install -r ~/klipper/scripts/klippy-requirements.txt
```

Then flash firmware for each MCU exactly as you would with stock Kalico. Phase stepping adds new MCU code, so **a re-flash is required** the first time you switch.

> **Tip:** verify the new MCU command compiled in before flashing.
> ```bash
> strings -a out/klipper.elf | grep "queue_phase_move oid"
> ```
> If nothing prints, the build did not pick up `src/tmc_phase_stepper.c` — re-check that `src/Makefile` was updated.

---

## Configuration

Phase stepping and s-curve motion are both **off** by default. You opt in by adding a few config keys.

### Minimum config to turn on phase stepping

```ini
# Required: a [tmc_phase_stepping] section enables the host-side scheduler.
[tmc_phase_stepping]
# Optional: explicit update rate. Default 25000 Hz, max 50000 Hz.
# The host will silently lower this if it cannot fit on the bus.
phase_update_rate: 25000

# Optional: pair motors that share a belt (AWD CoreXY).
# Use phase_pair, phase_pair2, phase_pair3, ... for additional pairs.
phase_pair: stepper_x, stepper_x1
phase_pair2: stepper_y, stepper_y1
```

There is **no per-stepper `phase_stepping: true` switch**. Any TMC2130/5160 stepper in your config is eligible — the `[tmc_phase_stepping]` module discovers them at activation time.

### Minimum config to turn on s-curve motion

```ini
[printer]
# Setting max_jerk > 0 switches the motion planner from trapezoidal
# (constant-acceleration) to a 7-segment jerk-limited s-curve.
# Reasonable starting value: max_accel * 10 (units: mm/s^3).
max_jerk: 50000

[extruder]
# Optional cap on jerk during extrude-only moves (no XY motion or retraction).
# 0 = no separate cap (uses max_jerk).
max_extrude_only_jerk: 0
```

If `max_jerk` is `0` (the default), the planner behaves exactly like upstream Kalico — every code path falls through to the legacy trapezoidal profile.

### Full configuration reference

| Section | Key | Default | Range / Notes |
|---|---|---|---|
| `[printer]` | `max_jerk` | `0` (off) | `mm/s^3`. `0` keeps legacy trapezoidal motion. Typical values: `max_accel × 5` (conservative) to `max_accel × 20` (aggressive). |
| `[extruder]` | `max_extrude_only_jerk` | `0` (no cap) | `mm/s^3`. Caps jerk on extrude-only moves; useful if the extruder cannot keep up with the kinematic jerk. |
| `[tmc_phase_stepping]` | `phase_update_rate` | `25000` | `Hz`. Max `50000`. The host's update-rate guard may lower this. |
| `[tmc_phase_stepping]` | `phase_spi_max_bus_load` | auto: `0.50` (DMA) / `0.20` (polled) | `0.05`–`0.80`. Fraction of bus time the group ISR is allowed to consume; leaves headroom for other SPI traffic. |
| `[tmc_phase_stepping]` | `phase_chain_per_motor_overhead` | `0` (auto) | seconds. Override the per-motor SPI overhead used by the chain-time guard. Auto picks `1.2 µs` on DMA, `1.4 µs` on polled (H7 @ 520 MHz). |
| `[tmc_phase_stepping]` | `phase_pair`, `phase_pair2`, … | unset | Two stepper names, comma-separated. Pairs receive an atomic clock reset so their phase counters never drift. |

### Build-time options (Kconfig)

These only matter when running `make menuconfig` on the firmware build. They are auto-selected sensibly on STM32H7 and do not require manual toggling on supported boards.

| Symbol | Default on STM32H7 | Default elsewhere | What it does |
|---|---|---|---|
| `WANT_SPI_DMA` | `y` | `n` | Builds the DMA SPI primitive (`spi_dma_kick_tx`). |
| `PHASE_STEPPER_EXPERIMENTAL_DMA` | `y` | `n` | Routes phase-stepping writes through DMA instead of polled SPI. |
| `HAVE_DMA_BUF_REGION` | `y` | `n` | Provides a DMA-coherent SRAM region (D2 SRAM at `0x30000000` on H7). |

After `make`, verify the DMA path is on:

```bash
grep PHASE_STEPPER_EXPERIMENTAL ~/klipper/.config
# Should print: CONFIG_PHASE_STEPPER_EXPERIMENTAL_DMA=y on H7
```

---

## G-code commands

| Command | Purpose |
|---|---|
| `PHASE_STEPPER_SUSPEND` | Hand control back to step/dir mode (e.g. before homing). |
| `PHASE_STEPPER_RESUME` | Re-enter phase stepping after `SUSPEND` or homing. |
| `PHASE_STEPPER_STATUS` | Print current update rate, bus load, segment rate per motor, and which guard (bus-load or chain-time) is active. |
| `PHASE_STEPPER_DEBUG MCU=1 TMC=1` | Dump live MCU clocks and TMC driver state (MSCNT, MSCURACT, DRV_STATUS, etc.). |
| `PHASE_STEPPER_TRACE` | Enable per-segment trace logging (verbose; for debugging only). |
| `PHASE_STEPPER_SET_DIRECTION STEPPER=<name> DIR=<+1|-1>` | Override the resolved phase direction at runtime, without re-editing `printer.cfg`. |
| `TEST_XDIRECT STEPPER=<name> A=<int> B=<int>` | Write raw coil A/B values directly to a driver for bench testing. |

### Homing workflow

Phase stepping is **not** automatically suspended around homing. The intentional workflow is:

```gcode
PHASE_STEPPER_SUSPEND
G28
PHASE_STEPPER_RESUME
```

(or wrap it in a `START_PRINT` macro, etc.)

This is by design — auto-suspend was tried in earlier revisions and removed because the edge cases (interrupted homes, multi-axis homes, sensorless homes) produced subtle drift on resume.

---

## Performance reference

Phase stepping streams a `phase_move` write to every active motor every tick. The host sets the tick rate (`phase_update_rate`); the MCU sustainability depends on (a) how fast each tick's SPI transfer can complete, and (b) how much CPU the ISR consumes.

### SPI transport paths

The firmware ships with two transport implementations and picks the best one available on your MCU at compile time.

| Transport | Where it runs | Per-tick CPU (4 motors) | Per-tick bus time | Sustainable update rate | Notes |
|---|---|---|---|---|---|
| **DMA** | STM32H7 (Octopus Pro, Manta M8P, …) | ~3.4 µs | ~24 µs (overlapped with CPU) | **25–33 kHz** | Default on H7. Writes are queued to the DMA engine; CPU returns to the rest of the firmware while the bus transfers. |
| **Polled** | STM32F4 / G4 / RP2040 / any other supported MCU | ~34 µs | ~24 µs (sequential with CPU) | **~5–10 kHz** | Used automatically where DMA is unavailable. Correct but capped — the CPU has to wait for each transfer to finish before starting the next motor. |
| 4 dedicated buses (theoretical) | Custom H7 board, one SPI per motor | ~3.6 µs | ~6 µs (parallel) | ~150 kHz | Not implemented; documented as the next bandwidth tier if a board with 4 SPI peripherals is ever built. |

### Velocity ceilings

These are illustrative — the actual ceiling on your printer depends on your microstep count, drive train, frame stiffness, and acceleration.

| Setup | Realistic top travel speed |
|---|---|
| TMC5160 + STM32F4 + polled SPI | ~300–400 mm/s |
| TMC5160 + STM32H7 + DMA SPI | 1000–1500 mm/s |
| TMC5160 + STM32H7, AWD CoreXY (4 motors on one SPI2 @ 4 MHz) | 1000 mm/s sustained, click-free |

### The two update-rate guards

The host applies **two independent caps** to whatever `phase_update_rate` you ask for, and uses whichever is tighter.

1. **Bus-load cap** (`phase_spi_max_bus_load`)
   Caps the fraction of SPI bus time consumed by phase stepping. Protects bus headroom for non-phase-stepping users (TMC register reads, ADCs, other SPI devices).
2. **Chain-time cap** (automatic)
   Requires that the total tick chain — byte time + per-motor overhead × N motors — fit inside one tick period. If it doesn't, the MCU group ISR drops writes mid-chain on the next tick, producing a **50% effective rate**. The guard refuses configs that would overrun.

`PHASE_STEPPER_STATUS` prints which cap fired:

```
update_hz: 25000 (chain_time_guard)
spi_bus_load: 0.46 (cap 0.50)
chain_time: 33.2 us (period 40.0 us)
```

If `chain_time_guard` is what's limiting you and you want a higher rate, options in priority order:

- Raise the SPI clock (`spi_speed` in your TMC sections).
- Lower the number of motors on a single SPI bus.
- Measure your actual per-motor overhead and tune `phase_chain_per_motor_overhead` down.

---

## Hardware quick-reference

What you'll need for each tier of performance:

| Goal | Drivers | MCU | SPI clock | Update rate |
|---|---|---|---|---|
| Just try phase stepping | TMC2130 or TMC5160 on SPI | Any supported (F4/G4/H7/RP2040) | 2–4 MHz | 5–10 kHz |
| Silent 400 mm/s travel | TMC2130/5160 | STM32F4 or G4 | 4 MHz | 10 kHz |
| Sustained 1000+ mm/s | TMC5160 | STM32H7 (DMA path) | 4–6 MHz | 25–33 kHz |
| AWD CoreXY (4 motors, one belt pair each) | 4× TMC5160 on shared SPI2 | STM32H7 | 4 MHz | 25 kHz |
| Multi-extruder / Trad-rack systems | TMC2130/5160 on shared SPI | STM32H7 recommended | 4 MHz | 25 kHz |

---

## Architecture (for the curious)

This section is the technical "what changed and why" — skip it if you just want to print.

### Phase stepping pipeline

```
toolhead trapq ──► host segment generator ──► anchored compressor (C) ──┐
                                                                          │
            ┌─────────────────────────────────────────────────────────────┘
            ▼
  host mirror (_mcu_anchor_pos, int 16.16)
            │   tracks MCU's ps->position bit-exactly via closed-form
            │   start + count*vel + count*(count-1)/2 * accel
            ▼
      queue_phase_move ──► MCU per-motor segment queue
                                       │
                                       ▼
                       group_phase_stepper_event (timer ISR, every tick)
                                       │
                                       ▼
                spi_dma_kick_tx (H7) or spidev_transfer (polled)
                                       │
                                       ▼
                              TMC XDIRECT register
```

The key invariant: **every segment's `start_position` is forced to equal the bit-exact mirror of the previous segment.** This is what makes long chains of segments — including idle holds — round-trip the same value on host and MCU without drift.

### Continuous emission (May 2026 rewrite)

The host no longer has an idle/resume state machine. Instead:

- The group ISR runs every tick from activation to deactivation.
- When a motor's segment queue empties, the firmware zeros `vel`/`accel` and holds `count` at 0. The polynomial holds; the same XDIRECT bytes are re-written each tick.
- **"Idle" is just a segment with `vel=0, accel=0, count=N`.** The compressor emits it naturally when the kinematic position doesn't change.
- Boundary continuity is structural: the next segment after idle starts at exactly the same anchor by construction.

This replaces a ~250-line state machine (idle_position, last_idle_vector, had_zero_samples, find_resume_window, …) that never converged on a click-free resume.

### S-curve motion (May 2026)

Gated on `[printer] max_jerk > 0`. When enabled and the move is kinematic, `_set_junction_scurve` builds a 7-phase profile (the standard accel-jerk / accel-const / accel-jerk-out / cruise / decel-jerk / decel-const / decel-jerk-in) and `trapq_append_scurve` lays it down on **both** the toolhead trapq **and** the extruder trapq, so the extrusion polynomial stays cubic alongside XYZ.

Notable rules under `max_jerk > 0`:

- **Z-only moves stay trapezoidal.** RRF's documented benefit is PA continuity and XY resonance damping; neither applies to slow single-axis Z motion. Z hops and tilt-correction use the legacy path.
- **Hard stop at print↔travel boundaries.** `Move.calc_junction` forces `max_start_v2 = 0` when crossing extrude/non-extrude, matching RRF: *"RepRapFirmware always comes to a stop between extruding and non-extruding moves."* This keeps PA continuity across `print → Z-hop → print` sequences by stop-then-go.
- **Per-move jerk cap.** `Move.max_jerk` (set per-instance from the toolhead) can be lowered by `move.limit_jerk(j)`. Extrude-only moves are capped by `max_extrude_only_jerk` when set.

### Pressure advance with third-order motion

`kin_extruder.c::extruder_integrate` and `extruder_integrate_time` include a `sixth_jerk` term. Antiderivatives gain `t^4/4 * sixth_jerk` and `t^5/5 * sixth_jerk` respectively. `pa_move_integrate` adds `half_accel += PA * 3 * sixth_jerk` alongside the existing `start_v += PA * 2 * half_accel`.

That math is what turns *"third-order extrusion with PA applied"* into a second-order velocity profile instead of a first-order one — the difference between visible PA ringing on every corner versus clean walls.

### AWD pair atomic clock reset

The MCU exposes `reset_phase_clock_group oids=%*s clock=%u`, which resets multiple motors' position counters atomically inside one `irq_disable()` window. The host config groups paired motors:

```ini
[tmc_phase_stepping]
phase_pair: stepper_x, stepper_x1
phase_pair2: stepper_y, stepper_y1
```

At first activation, `_resolve_pairs` walks these specs and sets each motor's `_pair_partner`. With continuous emission, resets are rare anyway — only at activate and after `PHASE_STEPPER_SUSPEND`/`RESUME`.

### DMA SPI on STM32H7

`src/stm32/stm32h7_spi.c` exposes `spi_dma_kick_tx`, `spi_dma_is_inflight`, and DMA-TC IRQ handlers for streams 0–4 (one per SPI peripheral, SPI1–SPI5 TX). DMAMUX request lines:

| Peripheral | DMAMUX request |
|---|---|
| SPI1 TX | 38 |
| SPI2 TX | 40 |
| SPI3 TX | 62 |
| SPI4 TX | 84 |
| SPI5 TX | 86 |

NVIC priority `1` for DMA-TC IRQs (matches USB/CAN convention). The DMA buffers live in D2 SRAM at `0x30000000` (CPU-coherent for DMA1/DMA2), placed into a dedicated `.dma_buf` linker section.

> **Why a separate SRAM region?** STM32H7 RAM_START is `0x20000000` = DTCM, which is CPU-private and *not* reachable by DMA1/DMA2. The default BSS would put `bus_tx_buf[][]` somewhere the DMA engine cannot read.

### Polled-SPI fallback

On boards without DMA-coherent SRAM (RP2040, STM32F4, STM32G4), `spidev_kick_dma_tx`'s `#else` branch calls `spidev_transfer` synchronously and fires the completion callback immediately. Same correctness, lower ceiling.

---

## Verification

After every change to phase stepping or the s-curve planner, run through this list.

### Host self-tests (no MCU required)

```bash
~/klippy-env/bin/python test_phase_mirror.py
~/klippy-env/bin/python test_phase_compress_anchored.py
~/klippy-env/bin/python test_scurve.py
```

- `test_phase_mirror.py` — closed-form `_advance_mirror` vs N-tick simulation, 11000 trials including int32 overflow paths.
- `test_phase_compress_anchored.py` — bit-exact segment chain continuity, 200 fuzz trials + 6 boundary cases.
- `test_scurve.py` — 16 tests covering s-curve continuity, phase compression, extruder retraction direction, PA-with-jerk integral correctness (closed form vs C), and the print↔travel hard-stop rule.

### Hardware tests

| Test | What to do | What "pass" looks like |
|---|---|---|
| The click test | `G1 Y20 F6000`, `G4 P5000`, `G1 Y200 F6000`. Repeat with wait of 0.1 s / 1 s / 5 s / 30 s / 5 min. | Silent. Zero clicks at any wait duration. |
| Velocity ramp | Print at 200 → 500 → 800 → 1000 mm/s travel speed. Inspect `PHASE_STEPPER_DEBUG MCU=1 TMC=1` after each. | `event_count` increments monotonically. `skip_count ≈ 0`. `drv_status` has no `s2ga` / `s2gb` / `ola` / `olb` flags. |
| AWD pair sync | With `phase_pair` configured: `G1 X20 Y20`, wait, `G1 X200 Y200`. | Both axes click-free at both ends. |
| M84 / G28 cycle | 100 print/M84/wait/home/print cycles. | No clicks. |
| Third-order print | With `max_jerk = max_accel × 10`, print a 10 mm cube. | Silent at every print↔travel transition (each is a stop). Visibly less PA ringing on corner walls vs `max_jerk = 0` baseline. |
| Segment rate sanity | At 1000 mm/s extruding move with `max_jerk` enabled: `PHASE_STEPPER_STATUS`. | Per-motor segment rate stays below ~30 kHz. |

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `Unknown command queue_phase_move` at startup | `src/Makefile` was not picked up during the firmware build. `tmc_phase_stepper.c` did not compile. Re-make and re-flash. |
| Config parser error: `phase_bus_slots is not a valid option` (or `phase_bus_slot`, `phase_idle_lookback`, `phase_resume_lookback`) | Old config from before the May 2026 rewrite. Remove those four keys from `printer.cfg`. |
| 50% effective update rate (every other tick dropped) | Chain-time cap is being exceeded. Lower `phase_update_rate`, raise `spi_speed`, or reduce motors per bus. `PHASE_STEPPER_STATUS` will say `chain_time_guard`. |
| Click on resume after `M84` | Phase stepping was disabled by `M84`/`idle_timeout`. Re-issue `PHASE_STEPPER_RESUME`. |
| Click on every move start | Direction sign mismatch on a stepper. Use `PHASE_STEPPER_SET_DIRECTION STEPPER=<name>` to override at runtime, then update `invert_dir` in `printer.cfg`. |
| Polled-path printer feels velocity-limited | Expected. The polled SPI path caps at ~10 kHz update, which translates to ~400 mm/s wall speed on most kinematics. The H7 DMA path is the next tier. |

---

## What this fork inherits from upstream Kalico

Everything not described above is straight upstream [Kalico](https://github.com/KalicoCrew/kalico) — the MPC and velocity-PID heater controllers, dockable probes, sensorless homing extensions, `gcode_shell_command`, `danger_options`, the bleeding-edge motion improvements, and all of the other features described in [docs/Kalico_Additions.md](docs/Kalico_Additions.md).

For documentation on upstream Kalico features, see [docs.kalico.gg](https://docs.kalico.gg).

---

## License

Kalico (and this fork) is Free Software. See the [license](COPYING).

## Acknowledgements

- The [Kalico](https://github.com/KalicoCrew/kalico) and [Klipper](https://github.com/Klipper3d/klipper) communities for the firmware this is built on.
- The TMC application notes (TMC2130 / TMC5160 datasheets) and Prusa's reference XDIRECT handoff sequence.
- RepRapFirmware for the jerk-limited motion model.

[![Join Kalico on Discord](https://discord.com/api/guilds/1297243471442214913/widget.png?style=banner2)](https://kalico.gg/discord)
