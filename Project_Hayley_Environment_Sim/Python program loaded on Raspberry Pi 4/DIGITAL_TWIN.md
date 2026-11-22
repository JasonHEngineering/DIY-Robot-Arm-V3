# Digital twin (Stage 1) – Project Hayley V3

A 3D twin of the arm served by the existing Dash UI. Your browser renders it, so the Pi only serves files and reads the encoders.

| Robot | Colour | Source |
|---|---|---|
| **Live** | metal/grey | AS5048A encoder angles read from the ESP32 over I2C |
| **Target ghost** | cyan, translucent | last angle command the UI sent over I2C |
| **Preview** | orange, translucent | CSV sequence playback or pose sandbox (never sent to the robot) |

Open it from the new **Digital Twin** tab in the UI, or full screen at `http://<pi-address>:8050/twin/`.

## What changed (V08 → V09)

`Project_Hayley_V5_IK_user_interface_V09.py` is V08 with five small edits (`diff` the two files to see them):

1. `bus = hayley_twin.make_bus(...)` replaces `smbus.SMBus(1)`. The wrapper passes every call through unchanged and records angle commands and encoder reads. It also serialises I2C access with a lock.
2. `hayley_twin.register(app.server, ...)` adds the `/twin/` routes.
3. The **Digital Twin** radio option and page (an iframe of `/twin/`).
4. `smbus2` is optional, so the app can start on a PC.
5. Changelog entry.

The ESP32 firmware is unchanged. The twin page only reads the encoders (5 Hz by default) while a browser has it open. Change the rate with `HAYLEY_TWIN_POLL_HZ`.

## Run on the Raspberry Pi (hardware)

Copy this whole folder to the Pi (including `hayley_twin.py` and `twin_web/`), then:

```
python3 Project_Hayley_V5_IK_user_interface_V09.py
```

## Run on a PC with no robot (simulation)

**Easiest: double-click `run_twin_sim.bat`.** On first run it creates a Python environment in `%LOCALAPPDATA%\hayley_twin_venv`. It uses `uv` with the Windows certificate store when available, so it works behind VPN/antivirus TLS inspection. It then starts the server and opens http://localhost:8050/twin/. Opening `twin_web/index.html` directly (`file://`) does **not** work: browsers block the modules and model loading there.

Manual equivalent:

```powershell
pip install -r requirements-sim.txt
$env:HAYLEY_SIM='1'; python Project_Hayley_V5_IK_user_interface_V09.py
```

Then open http://localhost:8050. The simulated ESP32 speaks the same I2C byte protocol. It queues moves and runs them the way `Project_Hayley_ESP32_V20.ino` does: all joints arrive together, and the slowest joint sets the duration. With a 1000 steps/s master clock, 8 microsteps and 30:1 gearing, that is 7.5 °/s on axes 1–5 and 225 °/s on axis 6. Change the clock with `HAYLEY_SIM_STEPS_PER_S`. Every page of the UI (Angles, Steppers, Path Plan sequence run, Motor Modes) works against the simulator.

## Firmware timing model

Previews, the simulator, the TCP-lock run-time estimate and the collision "after ~X s" use the same model of the ESP32 motion code (`twin_web/motion.js`, `Motion` in `hayley_twin.py`).
Choose it to match the firmware that is flashed:

```
HAYLEY_FIRMWARE=V21 python3 Project_Hayley_V5_IK_user_interface_V09.py   # V21 still on the ESP32
python3 Project_Hayley_V5_IK_user_interface_V09.py                        # V22 (default)
```

If you retune the V22 `#define`s (`RAMP_ACCEL`, `AXIS_MAX_STEP_RATE`, ...), update `MOTION_PROFILES` in `hayley_twin.py` to match. The UI low/high speed switch is followed automatically.

## Sequence preview

Load any CSV with `theta1..theta6` columns (from the IK solver, or recorded). The twin replays it starting from the live pose with the same timing model, so you see the estimated run time. Like `sequence_run()` it skips rows that have `Solve Status = False` or are outside the limits in `config - project Hayley V5.csv`. You can test with `sample_sequence_twin_demo.csv`.

## Collision checking

Collisions are checked against the real CAD meshes, using a BVH on each link (`twin_web/collision.js`). Two kinds of collision are checked:
- **Self-collision** between links that are not neighbours in the chain.
- **The table**: the plane under the base plate, 10 mm below the base frame.

Pairs that already touch at kinematic zero (bearings, shared covers) are ignored. Screws and standoffs are left out of the check.

| What | When it's checked | Shown as |
|---|---|---|
| Live robot | every pose update | links turn red, a "LIVE COLLISION" pill and a banner |
| Commanded move | as soon as a new command is seen, the move from the live pose to the target is swept (joint-synchronous, like the ESP32) | "Commanded move: L5 ↔ table after ~4.2 s" and a banner while the move runs |
| Preview / sandbox | every change | red links on the orange arm |
| CSV sequence | the whole path is swept at 1° steps when it is loaded | red rows, red marks on the scrub bar, and clicking a row jumps to the collision |

These are **warnings only**: nothing blocks a command. On real hardware, a warning about a move already in progress is your cue to hit the E-stop. Checking *before* sending is what the sequence preview and the drag mode are for. The servo claw is not in the CAD, so collisions caused by the claw itself are not detected.

## TCP lock & drag (re-pose the arm while the tool stays put)

1. Pose the preview (sandbox sliders, "copy live pose", or pause a CSV preview at a row).
2. Choose a lock mode and press **Lock TCP**:
   - **Position only (3D)** (default): only the tool point stays fixed; the tool may tilt. Three degrees of freedom remain.
   - **Position + tool direction (5D)**: with the TCP on the axis-6 centreline (row 27 = 93, no side offset) the only freedom left is spinning A6, so links can't be dragged in this mode.
   - **Full pose (6D)**: a 6-axis arm has *no* continuous freedom with all 6 TCP dimensions fixed. "Find alternative configurations" lists the discrete solutions within your joint limits. In practice that is the wrist flip (A4 ± 180°, −A5, A6 ± 180°) when A4 stays within ±120°. Elbow and shoulder flips fall outside the ±90° and ±100° limits.
3. Drag any orange link with the left mouse button. A task-priority IK solver (`twin_web/kin.js`, using your linkage table) moves the arm in the null space of the TCP lock. It stops at joint limits, and at collisions if "stop at collisions" is ticked.
4. Waypoints are recorded every 0.5, 1, 2 or 5° of joint motion. The panel shows the waypoint count, the estimated run time, and the **largest TCP drift between waypoints**. The ESP32 interpolates each row in joint space, so between waypoints the TCP strays slightly; a finer step means less drift.
5. **▶ preview path** replays it with the ESP32 timing and a collision sweep. **⬇ export CSV for robot** writes `recorded_joints_table_tcp_lock_*.csv` in the same 20-column structure as the Recording Mode download (`Rows`, coordinates, Euler, quaternion, `theta1..6`, `time_stay`, `Solve Status`, `Servo Status`; Euler/quaternion computed exactly like the Dash app). Load it on the **Path Plan** page and run it. `time_stay` is 0 because the waypoints are meant to flow one after another.

Limits to keep in mind when executing on the real arm:
- Every waypoint is a separate stop-start move. A1–A5 move at up to 7.5 °/s.
- **The ESP32 V20 queue holds 200 moves** (`xQueueCreate(200, …)`, and `xQueueSend` does not wait). `sequence_run` sends all rows back to back, so rows beyond 200 would be silently dropped. The twin warns about this, and the simulator now drops them the same way. Use a coarser waypoint step for long drags.
- Switching to an alternative configuration is a plain joint move; the TCP is *not* held during that transit, and the drift figure shows this.

## Model provenance

- `twin_web/description/hayley_v3.urdf` and `meshes/` are generated. Don't edit them by hand; regenerate with the tools in `../digital_twin/cad_tools/`:
  - `export_links.py` (FreeCAD 1.1 `freecadcmd`) splits `Robot_Assembly.step` into per-link STL meshes. The base frame is located from the CAD itself: the J1 reducer centre, the J4 reducer axis and the top of the base plate.
  - `make_urdf.py` builds the URDF from `hayley_kinematics.py`, which is a verbatim copy of `local_linkage_data`, and copies the result here.
- Joint angle 0 = kinematic zero = the CAD pose. URDF joint angles are the same numbers the UI and ESP32 use.
- Verified: the URDF tool frame matches the UI's own `input_linkage_angles()` forward kinematics to 0.1 mm at test poses.

## Known limitations

- In a browser console, `hayleyTwin` gives access to the robots, the collision model and the kinematics, for debugging.

- The servo claw is not in the CAD assembly, so it is not modelled. Its open/closed state is shown as a pill.
- Masses and inertias in the URDF are rough estimates from CAD volumes. They are fine for visualisation and RViz, not for dynamics tuning.
- The meshes total about 18 MB. The first page load from the Pi takes a few seconds; after that the browser caches them.
- The sim timing ignores the ESP32's final closed-loop correction pass.
