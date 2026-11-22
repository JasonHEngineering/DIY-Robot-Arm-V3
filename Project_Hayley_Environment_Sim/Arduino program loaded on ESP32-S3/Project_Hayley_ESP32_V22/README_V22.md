# Project_Hayley_ESP32_V22: faster motion, same accuracy

V22 is based on V21. The I2C protocol is unchanged, so it works with UI V09. It compiles for `esp32:esp32:esp32s3` (core 3.3.13): 922 KB, 70 % of flash.

## What changed

| # | Change | Why | Where (search for) |
|---|---|---|---|
| 1 | Trapezoid ramp on the master axis: `RAMP_START_SPEED` 200, `RAMP_ACCEL` 4000 steps/s² | V21 ran the first and last 200 master steps of every move at 100 steps/s (2 s at each end) | `runMove()` |
| 2 | Per-axis top speed `AXIS_MAX_STEP_RATE` (2000 steps/s = 15 °/s on axes 1-5; A6 1000 = 225 °/s) | Twice V21's 1000; the master rate is scaled so that no axis exceeds its own limit | `masterVmaxFor()` |
| 3 | Waypoint blending | Consecutive queued rows flow into each other with no stop and no correction pass, if no axis reverses and the row has no claw action. Junction speeds are matched per axis to within `RAMP_START_SPEED`. The last row and claw rows still stop and get the tight correction | Core 1, `xQueuePeek` |
| 4 | Correction loop: 15 ms settle, at most 6 passes, `CORRECTION_GAIN` (A6 0.5), status bit 2 = "target not reached" | The moving average still held motion samples, and the loop could hunt forever | Core 1 |
| 5 | Encoder moving average unwraps across 0/360° | Fixes the reading freezing at **A5 ≈ −10.9°, A4 ≈ +106.8°, A6 ≈ −116.3°** | `unwrapAngle()` |
| 6 | I2C receive uses its own buffer | V21 wrote incoming rows into the buffer Core 1 was executing from, so a claw command could fire on the wrong row | `receiveData()` |
| 7 | `Serial.begin(921600)`; debug prints behind `#define DEBUG_SERIAL 0` | The 9600-baud per-row prints blocked Core 1 for ~0.4 s per row | top of file |
| 8 | UI low/high speed (`set_low_speed`, ×0.2) now also slows queued moves | A safe way to try new settings | `masterVmaxFor()` |
| 9 | A move of 0 steps completes immediately | V21's Core 1 spun forever if every axis needed 0 steps | `startMove()` |

**Serial Monitor:** set it to **921600** baud. For per-move diagnostics set `DEBUG_SERIAL 1` (this slows motion).

## Expected timing (modelled; the twin and simulator use the same model)

| Move | V21 | V22 (defaults) |
|---|---|---|
| 5° on A2 | 4.3 s | 0.7 s |
| 30° on A2 | 7.6 s | 2.4 s |
| 60° path in 1° waypoints (TCP-lock export) | ≈ 80 s | ≈ 7 s |

## First flash and test (E-stop in reach)

1. Flash V22. In the Dash UI, set **low speed** first (Motor Modes page).
2. Jog each axis ±5° from the Angles page and watch the twin: direction, smooth start and stop, Δ settling near 0.
3. Run a short previewed CSV. Check that the final pose matches the target (Δ < 0.1°) and that no "WARN: target not reached" appears on serial.
4. Switch to **high speed** and repeat with the payload (claw) attached.
5. **Park A5 at −11°** and check that the reading keeps updating: this is the wrap fix (V21 froze there).

## Tuning further (one change at a time)

- **Speed:** raise `AXIS_MAX_STEP_RATE` 2000 → 3000 → 4000 (30 °/s). Raise A2 last: it carries the most load.
- **Acceleration:** raise `RAMP_ACCEL` 4000 → 8000. Short waypoint paths gain the most.
- **Stalls or missed steps** show up as large Δ after a move, or "target not reached". If you see them, step back one stage. At about 4000 steps/s and above, consider SpreadCycle on the TMC2209 (SPREAD pin, or UART), since StealthChop loses torque at speed.
- **A6:** keep it at or below about 1300 steps/s (300 °/s). Above that, the encoder's ±1° plausibility gate rejects readings while it moves.
- **After any change,** update `MOTION_PROFILES["V22"]` in `hayley_twin.py` so the twin's previews and run-time estimates keep matching.
