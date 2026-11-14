# Grasp matrix

Which parallel gripper can pick which object, and how robustly? This tests six grippers (Robotiq
2F-85, 2F-140 and Hand-E, Franka Hand, OnRobot RG2 and 2FG7) on 20 3D-scanned objects from
[SimReady Central](https://huggingface.co/spaces/nvidia/simready-central), in Isaac Sim with PhysX.
New grippers can be added without rerunning the others (see "Adding a gripper").

All results are **simulated**. Contact physics only approximates real fingertips, so read them as
rankings under stated assumptions, not real-world success rates. The low-friction and heavy
conditions show how sensitive each ranking is.

## Pipeline

| Step | Script | Runs in |
|---|---|---|
| 1. Measure each gripper's opening and where its pads meet, and check its set-up | `tools/calibrate_grippers.py` | Isaac Sim |
| 2. Build the object catalog from SimReady metadata (no geometry download) | `grasp_matrix/build_catalog.py` | venv (usd-core) |
| 3. Run every gripper × object pair, resumable | `grasp_matrix/run_matrix.py` → `run_pair.py` | venv → Isaac Sim |
| 4. Summarise | `grasp_matrix/report.py` | venv |
| Steps 1, 3 and 4 for one gripper | `grasp_matrix/evaluate_gripper.py` | venv → Isaac Sim |
| Draft a new gripper's config entry | `tools/gripper_geometry.py` | venv (usd-core) |
| Check a single trial by eye | `grasp_matrix/render_grasp.py` | Isaac Sim |

```powershell
C:\issacsim-6-2\python.bat tools\calibrate_grippers.py       # all grippers, or --grippers a,b
.venv\Scripts\python.exe grasp_matrix\build_catalog.py assets\allsides_meta --revision <dataset commit>
.venv\Scripts\python.exe grasp_matrix\run_matrix.py          # Ctrl+C is safe; rerun to resume
.venv\Scripts\python.exe grasp_matrix\report.py
C:\issacsim-6-2\python.bat grasp_matrix\render_grasp.py --run-id 1 --grasp 48
```

## Adding a gripper

Nothing stored is rerun when a gripper is added. Each pair's results are keyed by a hash of its
settings, which includes only that gripper's own configuration and calibration. Adding objects to
`config/objects.json` works the same way: only the missing pairs run.

1. Download the asset into `assets/<vendor>/`, for example
   `uvx --from huggingface_hub hf download onrobot/simready-grippers --repo-type dataset --include "onrobot_rg2/*" --local-dir assets/onrobot`.
2. Draft its config entry. This prints the asset's bodies, joints and materials, then a guess
   at the entry:
   `.venv\Scripts\python.exe tools\gripper_geometry.py <gripper.usda> <VariantSet>=<physics variant>`
3. Check the draft and add it to `config/grippers.json`.
4. Run a pilot on a few objects, then the rest:
   ```powershell
   .venv\Scripts\python.exe grasp_matrix\evaluate_gripper.py --gripper onrobot_rg2 --objects bolt_QABQ5FUC3Q,toy_block_6MPL7NVFVU
   .venv\Scripts\python.exe grasp_matrix\evaluate_gripper.py --gripper onrobot_rg2
   ```
   This calibrates the gripper (stopping if a check fails), runs only its unfinished pairs, and
   rewrites the report with every gripper side by side.
5. Look at one held and one failed trial with `render_grasp.py`.

Config keys:

| Key | Meaning |
|---|---|
| `usd`, `variants` | The asset and the variant selections that turn on PhysX |
| `articulation` | Where to look for the fingertip bodies, relative to the asset's default prim |
| `drive_joint`, `joint_type` | The joint that opens and closes the fingers (`revolute` or `prismatic`) |
| `approach_axis`, `closing_axis` | The gripper's own axes: towards the fingertips, and the direction the fingers close along |
| `fingertips` (optional) | The two pad bodies' names, if they don't contain "fingertip" |
| `coupled_joints` (optional) | Joints driven with the same target, for fingers with separate drives instead of a mimic linkage |
| `anchor_joint` (optional) | A fixed root joint with no `body0`, which pins the base to the world origin. It is re-anchored to the gripper's root prim so the gripper can be placed anywhere. |
| `pad_shapes` (optional) | For stepped fingertips: the collision shapes that should touch the object. They define the pad, whose centre is where the sampler puts the object. Without it the pad is the whole fingertip body. |
| `label`, `datasheet`, `source`, `notes` | Documentation only, left out of the settings hash. `datasheet.max_opening_m` is checked against the measured opening. |

The calibration finds which end of the joint range is open, measures the pad gap and pad height
across the stroke, and checks the set-up. A failed error check leaves the gripper uncalibrated, so
it cannot run. Warnings are stored with the calibration and listed under "Gripper notes" in the
report.

| Check | Level | Example it caught |
|---|---|---|
| Exactly two fingertip bodies | error | |
| The pads lie along `+approach_axis`, apart along `closing_axis`, and centred | error | |
| The fingertips follow when the gripper's root is moved, as the batch run moves it | error | Franka Hand: its root joint pinned the hand to the world origin, 711 mm away |
| If the asset isn't stored with the fingers open, the gripper starts open after a reset once each joint's open position is set | error | Franka Hand (stored closed), RG2 and 2FG7 (stored half open). Without this, every trial would open the fingers during placement, next to the object. |
| The gap shrinks steadily from open to closed | warning | |
| The pads meet somewhere in the stroke | warning | OnRobot 2FG7: the closest gap is 6.6 mm |
| The measured opening is within 10% of the datasheet's | warning | Hand-E: 36.8 mm against 50 mm, which led to finding its stepped fingertips. RG2: 97.7 mm against 110 mm. |
| The gripper has a physics material | warning | Franka Hand: none, so friction is PhysX's default and the low-friction condition is milder |
| The fingertip's innermost collision shapes cover most of the pad | warning | Franka Hand: only the tip of each stepped finger touches, so the whole-finger pad put the object 18 mm too deep. Hand-E: a step near the base reaches furthest in, so the opening was measured between the steps (see "Findings about the assets"). |

For an asset not stored open, the calibration records every joint's open position and the open
pad boxes as `open_state`. Each trial sets those joint positions before it starts.

A change to a gripper's config or calibration reruns only that gripper's pairs. A change to the
protocol (`gm_config.py`) reruns everything, and gets a new `PROTOCOL_VERSION`. A Robotiq pair
(2F-85 × bolt) rerun after the changes for other grippers gave bit-identical grasps and trials.

## Protocol (per gripper × object)

1. Drop the object on a table and let it settle. Its resting pose is the start pose for every trial.
2. Run a control trial: with the gripper parked away, switching the table's collision off must make
   the object fall. If it doesn't, the pair is marked failed.
3. Sample 200 antipodal grasp poses on the object's mesh (NVIDIA `isaacsim.replicator.grasping`),
   with 4 approach directions around each grasp axis.
4. Measure the object's width along each pose's closing line, by casting a ray through the grasp
   centre on the visual mesh.
5. Pre-filter the poses, with no physics:
   - **from_below**: approaching from below the table
   - **hits_table**: a fingertip pad would cut into the tabletop, either open or closed to the
     object's width, even after backing the gripper off along its approach axis by up to half the
     pad length. The 2F fingertips move along the approach axis as they close (see the calibration
     curve), so the closed check matters for them. A grasp backed off by less than that is kept,
     with the shift recorded as `lift_m`, so the sampled contact point stays on the pad.
     Contact between the rest of the gripper and the table or object is left to the physics trial.
6. Physics trial for each remaining pose: place the open gripper, close the fingers, then switch the
   table's collision off (equivalent to lifting the object) for one second. Every pose that
   survives placement gets this lift test.
7. Re-test every pose that held under two more conditions: friction ×0.5 and mass ×2.

| Outcome | Meaning |
|---|---|
| collided | Placing the open gripper moved the object more than 1 cm (no lift test) |
| held | With the table gone, the object came to rest in the fingers: it moved less than 5 mm in the last 0.5 s and sagged less than 5 cm in total |
| missed | Not held, and the fingers closed to where the pads meet, so nothing was between them |
| blocked | Not held, and the fingers stopped more than 1 cm wider than the object's width at the grasp, so something else (the table, another part of the object) was in the way |
| dropped | Not held otherwise: the fingers reached the object but it slipped out |

### Protocol history

Each change below altered results, so it got a new protocol version and a full rerun.

| Version | Change | Why |
|---|---|---|
| v2 | Table check on the fingertip pads only, backing grasps off by up to half a pad; "held" means the object came to rest | v1 checked the whole gripper's bounding box, which rejected nearly every grasp on low objects. Its fixed 2 cm slip threshold also made borderline grasps flip between reruns. |
| v3 | Table check also with the fingers closed to the object's width. Every grasp gets the lift test, and failures are labelled from the finger gap. | Renders showed 2F fingertips planting on the table while closing: they descend 13 mm (2F-85) and 23 mm (2F-140). This failed most 2F grasps on low objects (clamp: 0 held of 31 with the 2F-140). v2 also stopped trials early as "jammed" when the fingers barely moved, which wrongly failed objects nearly as wide as the opening. |

v2 results for 19 pairs are kept in `results/grasp_matrix_v2.sqlite` for comparison.

Every setting that affects results goes into a hash stored with each run (`gm_config.run_settings`),
so changing a threshold or a gripper's calibration reruns the affected pairs instead of mixing results.

## Results (protocol v3, 2026-11-09)

Six grippers on 20 objects: the three Robotiq models first, then Franka Hand, OnRobot RG2 and
OnRobot 2FG7, which were added with `evaluate_gripper.py`. The full table is in
`results/summary.md`, and per-pair numbers are in `results/summary_pairs.csv`. "Robust" means held
at baseline, with friction ×0.5, and with mass ×2.

| Gripper | Opening (measured) | Objects with a robust grasp | Robust / tested | Still held, mass ×2 | Still held, friction ×0.5 |
|---|---|---|---|---|---|
| 2F-140 | 140 mm | 18 / 20 | 426 / 918 (46%) | 92% | 86% |
| RG2 | 98 mm | 11 / 20 | 260 / 786 (33%) | 86% | 70% |
| 2F-85 | 87 mm | 12 / 20 | 153 / 735 (21%) | 76% | 72% |
| 2FG7 | 45 mm (pads stop 6.6 mm apart) | 10 / 20 | 76 / 541 (14%) | 69% | 59% |
| Franka Hand | 78 mm | 13 / 20 | 58 / 818 (7%) | 59% | 50% |
| Hand-E | 50 mm | 10 / 20 | 45 / 733 (6%) | 63% | 39% |

- **2F-140** is still the most versatile and most robust. It is the only gripper with many robust
  grasps on the wide objects (figurine 72, drum 65, jewellery box 21, lemon 49), and it is best on
  mug, jar, clamp, electronics housing and toolbox.
- **RG2** is second overall. It is best on book (50 robust against the 2F-140's 43), beer stein (46
  against 34) and bucket (18 against 12), and level with the 2F-85 on the toy block (67 against 69).
  It reaches the wider objects through their rims and edges, although its 98 mm opening is less than
  their smallest extent.
- **2F-85** is best on the toy block (69 robust), which fits its 87 mm opening squarely. On the
  beer stein it ties the 2F-140 (34 each), behind the RG2.
- **Low objects lying on the table** (wooden spoon, sausage, rolling pin, lid, mouse) split by
  finger type. The three linkage grippers, whose fingertips descend as they close (2F-85, 2F-140,
  RG2), had 3, 3 and 0 robust grasps on them in total. None of their spoon or sausage grasps was
  tested: about a quarter approached from below the table, and the rest had a pad hitting the
  table. The three parallel grippers (Hand-E, Franka Hand, 2FG7) got 2 to 12 of these grasps
  past the same filters on each object, and had 2, 6 and 8 robust grasps in total. Most of those came
  from the 2FG7 on the rolling pin (6 robust of 11 tested). The counts are small, but they point the
  same way as the descent: a gripper whose fingertips stay level can grip closer to the table. The rest are still at most two per cell. For these objects as
  presented, a top-down parallel grasp is still a weak choice; suction, a fixture or sliding them to
  an edge remain the options.
- **Franka Hand** reaches small objects well (bolt 5 robust, ball valve 5), but its grasps slip:
  247 of its 818 baseline trials were dropped, more than any other gripper. Its fingertips have no
  physics material, so they use PhysX's default friction against the Robotiq grip material's 1.0.
  That is a likely cause, not a tested one. Its low-friction condition is also milder than the
  others' (only the object's friction is scaled), yet only 50% of its grasps survive it.
- **2FG7** stops 6.6 mm short of closing, so thin features are missed (91 missed trials). Its
  45 mm opening limits it to rims, handles and small objects, where it does reasonably (jar 27
  robust).
- **Hand-E** was rerun after its fingertip pads were corrected (see "Findings about the assets").
  The earlier runs took its opening as 36.8 mm and sampled only features up to 35 mm. The rerun
  raised it from 14 robust grasps in 513 trials to 45 in 733, mostly on the beer stein (1 to 25) and
  the book (0 to 6). It is still last. It collides most often (355 placements), and its grasps fail
  most under low friction (39% survive). The collisions are not from the step near the base of its
  fingertips: grasps on features wider than 36.8 mm collided less often (38%) than narrower ones
  (52%). For all three parallel grippers, narrow features (rims, handles, edges) collide most.
  The earlier Hand-E runs are kept in the database; the report uses the latest run of each pair.
- Cells with one to five robust grasps are weak evidence. Random antipodal sampling gives few
  testable poses on low objects, so a "best" decided by one or two grasps (bolt, ball valve, lid,
  spoon, mouse) is not a real difference.

Caveats beyond simulation itself:
- Grip force comes from each asset's drive, not the datasheet. Both 2F assets ship with the same
  finger drive (stiffness 3, max force 26), although the datasheets differ (2F-85 up to 235 N,
  2F-140 up to 125 N).
- The RG2 measures 97.7 mm open between its fingertip bodies, against a 110 mm datasheet stroke. The
  cause is not investigated.
- Each object is tested in one resting pose.
- The lift test removes the table instead of accelerating the gripper.

## Memory

Batch trials need no rendering, so `run_pair.py` starts Isaac Sim with `apps/grasp_physics.kit`: USD,
PhysX and the grasp sampler only, without the RTX renderer, viewport or UI. Pass `--full-app` to
`run_matrix.py` or `run_pair.py` to use the stock Isaac Sim app instead. `render_grasp.py` always uses
the stock app, because it needs the renderer.

Measured on 2F-85 × bolt: the physics-only app adds 1.9 GB of RAM and 0.5 GB of GPU memory, against
6.0 GB and 2.2 GB for the stock app, and gives bit-identical grasps and trial results.
`run_matrix.py --workers 2` runs two pairs at once. On this machine (RTX 5070, Ryzen 7 9700X) it gives
no speedup: each trial gets about 2.5× slower, while neither the GPU (about 55% busy) nor the CPU
(about 34%) is saturated. That points to per-step GPU synchronisation, so the default stays at one
worker.

## Data

`results/grasp_matrix.sqlite` holds the following tables:

| Table | Contents |
|---|---|
| `objects` | Catalog with extents, mass, Wikidata Q-code, licence, source commit |
| `grippers` | Calibrated geometry |
| `runs` | One row per pair, with its settings, status and Isaac Sim version |
| `grasps` | Every sampled pose, the object's width along its closing line, the lowest pad point open and closed, and the filter that rejected it |
| `trials` | Outcome, finger closure and the pad gap it stopped at, placement move, total and late lift move, for each condition |

## Findings about the assets

- **Hand-E fingertips are stepped** (`Stroke50` variant, `Outside` mount). Each fingertip has four
  collision hulls. The one nearest the palm (`hull_1`, 99–109 mm from the gripper origin) reaches
  6.6 mm further in than the flat pad face further out (`hull_0` and `hull_2`, 117–146 mm), with a
  groove (`hull_3`) between them. Between the steps the opening is **36.8 mm**. Between the pad
  faces it is 2 × 24.8 = **49.6 mm**, in line with the datasheet's 50 mm. The first Hand-E runs used
  the whole fingertip, so their opening was 36.8 mm and their pad centre 123 mm from the gripper
  origin. This was first recorded here as the asset opening only 36.8 mm, and the calibration's
  contact check found the real cause. Since 2026-11-09 the config sets
  `"pad_shapes": ["hull_0", "hull_2"]`: opening 49.6 mm, pad centre 131.6 mm, pads meeting at the
  closed limit (0.025 m). The steps pass through each other past joint 0.01875 m, because
  self-collision is off inside the articulation.
- **Franka Hand** (`frankarobotics/simready-assets`):
  - Its root joint has no `body0`, so it pins the hand to the world origin (`anchor_joint`).
  - Each finger has its own drive, with no mimic joint (`coupled_joints`).
  - It has no physics material.
  - It is stored with the fingers closed.
  - The fingers are stepped, and only `box_3` at the tip reaches the object. Its centre, 103.6 mm
    from the hand origin, matches Franka's published TCP offset of 103.4 mm.
- **OnRobot RG2** (`onrobot/simready-grippers`): opens 97.7 mm between its fingertip bodies,
  against a 110 mm datasheet stroke. Like the Robotiq 2F, its fingertips descend while closing,
  by 34 mm. It is stored half open, at 60° of its 70° range.
- **OnRobot 2FG7**: its 38 mm stroke runs from 44.6 mm down to a 6.6 mm gap, so the pads never meet,
  and anything thinner than 6.6 mm is missed. It is stored half open.
- **Robotiq 2F fingertips:** the 2F models' fingertips move 13 mm (2F-85) and 23 mm (2F-140) along
  the approach axis, away from the palm, while closing. Most of that happens early in the stroke.
  The sampler's fingertip standoff uses the midpoint, and the table check uses the full curve.
  This follows from the 2F's linkage fingers, so it is not an asset error, and a grasp planner for
  a real 2F gripper has to account for it too.
