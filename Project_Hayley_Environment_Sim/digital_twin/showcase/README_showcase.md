# TCP showcase path: proposal

A "show-off" sequence that keeps the **tool tip doing simple, clean things** while the whole arm works underneath it:

1. **Translate** the TCP along base **X**, then **Y**, then **Z**, holding the tool orientation: 0 → +30 → −30 → 0 mm.
2. **Rotate** the tool about its own tip, holding the tip still: **roll**, then **pitch**, then **yaw**, each 0 → +30 → −30 → 0°.

Waypoints are every 5 mm / 5°. Each one is solved with the twin's full-pose IK, continuing from the previous solution.

## Start pose: tool pointing straight down

`A = [0, −5, 35, 0, 60, 0]` puts the TCP at (101.1, 0, 91.2) mm, with A5 = 60°.

Your arm is compact: 70 mm upper arm, about 114 mm from the elbow to the wrist centre, and 93 mm from the wrist to the TCP. The generator tried 187 tool-down start poses and picked the one that stays best conditioned along the **whole** path:

| Check (worst along the 145 rows) | Value | Why it matters |
|---|---|---|
| IK error | 0.01 mm / 0.002° | every row is an exact solution |
| Joint-limit margin | ≥ 9.9° | never touches a limit |
| Wrist singularity: \|A5\| | ≥ 35.6° (at pitch −30°) | A5 = 0 means A4 and A6 are collinear |
| Shoulder singularity: wrist-centre distance from the A1 axis | ≥ 69 mm | 0 would make A1 undefined |
| Elbow / overall conditioning: smallest singular value | ≥ 19.6 (start 34.9; 0 = singular) | stays far from the stretched-arm limit |
| Largest joint step between rows | 8.2° (pitch) | no sudden joint spins |
| TCP stray between rows | ≤ 0.13 mm | the ESP32 moves the joints in straight lines (joint space), not the tool tip in a straight line, so the tip strays slightly between rows; 5 mm / 5° rows keep that negligible |

## What each segment shows

| Segment | Joints that move | What it shows |
|---|---|---|
| Translate X | A2 −30…+21, A3 6…59, A5 60…65 | shoulder and elbow trade off to keep the tool vertical |
| Translate Y | A1 ±14, A4 ±3, A6 ±13 | A1 swings and the wrist cancels the yaw |
| Translate Z | A2, A3, A5 45…75 | a vertical "press" |
| Roll | **A6 only** (±30) | the TCP sits on the axis-6 centreline, so roll is a pure spin |
| Pitch | A2 −45…+39, A3 −5…+73, A5 36…90 | **the showpiece**: the whole arm swings to keep the tip still |
| Yaw | A1 ±22, A4 ±26, A6 ±34 | three joints coordinate a sideways tilt about the tip |

## Files

| File | Rows | Run time (V22) | Collisions (twin sweep) |
|---|---|---|---|
| `tcp_showcase_full.csv` | 145 | 0:45 (V21: about 8 min) | none |
| `tcp_showcase_1_translate.csv` | 73 | 0:23 | none |
| `tcp_showcase_2_rotate.csv` | 73 | 0:26 | none |
| `tcp_showcase_3_singularity_demo.csv` | 27 (15 marked not solved) | 0:10 | none |

All files are under the 200-row ESP32 queue limit and use the Dash recorded-table format, so they load directly on the **Path Plan** page.

## The singularity demo (twin first)

`tcp_showcase_3_singularity_demo.csv` starts near home (forearm level, tool pitched 25° down, A5 = 25°) with a small **3° yaw offset**. It then pitches the tool up toward the forearm axis:

- A5 drops 25 → 16 → 8°, while A4 and A6 counter-rotate faster and faster: 4° → 11° → 24° → **81° in one 5° pitch step**.
- Passing the singularity would need a ~180° wrist flip, but **A4 hits its +120° limit**. From there the arm cannot hold the commanded orientation (IK error 13–64 mm). Those 15 rows are marked `Solve Status = FALSE`, so Path Plan and the twin skip them.

This is the textbook reason to plan around A5 = 0. Watch it in the twin's sequence preview. On the real arm, run only the solvable rows, at **low speed**.

## How to run it

1. **Preview:** in the twin, load the CSV under *Sequence preview*. Check the collision sweep and the run time, then press Play.
2. **Move to the start pose:** run the first row on its own with Path Plan → *select target* → 1, at low speed.
3. **Run:** use *Run all sequence*. Watch **Path deviation now / max** in the twin: it measures how closely the real TCP follows the planned path.
4. **Regenerate:** with other strokes, angles or step sizes:
   ```
   node make_tcp_showcase.mjs --L 30 --A 30 --step 5 --astep 5 --minx 60 --minz 80
   node make_tcp_showcase.mjs --q0 0,-5,35,0,60,0      # force a start pose
   ```
   Bigger `--L` / `--A` push the arm toward its limits. The script refuses if any check fails.

## Notes
- The TCP is the axis-6 shaft tip, since there is no claw in the model. With a claw mounted, the tip sits lower, so either raise `--minz` or set the claw length in the linkage table first. Otherwise a tool-down pose at z = 91 mm could put the claw into the table.
- The joint angles were solved with the current linkage table (row 27 = 93, no 12 mm offset) and are identical in the Dash UI, both IK solver copies, and the twin.
