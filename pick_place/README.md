# Pick-and-place cell

Step 3 of the project: a robot arm uses the grasp matrix's results. A UR10 with a Robotiq 2F-140
on its flange (the best gripper in the matrix) picks a scanned object off a table and puts it on a
marked place spot. The grasp comes from the matrix's robust grasps for that object, and cuMotion
plans the motions. Isaac Sim's window shows the cell by default.

Step 4 adds a depth camera on the wrist: the robot then finds the object itself instead of being
told where it is by the simulation ([Step 4](#step-4-the-wrist-camera-eye-in-hand) below). Step 5
runs the perception as a ROS 2 node, with Isaac Sim as the robot ([Step 5](#step-5-ros-2) below,
and `ros2_ws/README.md`).

All results are **simulated**, like the grasp matrix's.

## Running it

```powershell
# watch it: opens Isaac Sim's window, runs 3 cycles, closes (add --keep-open to keep the window)
C:\issacsim-6-2\python.bat pick_place\run_cell.py --object mug_A07RDPW843
# headless, more cycles
C:\issacsim-6-2\python.bat pick_place\run_cell.py --object mug_A07RDPW843 --cycles 10 --headless
# several objects, one after another, then the report
.venv\Scripts\python.exe pick_place\run_batch.py --objects toy_block_6MPL7NVFVU,mug_A07RDPW843 --cycles 6
.venv\Scripts\python.exe pick_place\report.py
```

To choose the object in a window instead, run cycles one at a time and switch objects without
restarting, use `cell_app.py` (see [The interactive window](#the-interactive-window)).

Set `OMNI_KIT_ACCEPT_EULA=YES` first, and close large programs (Chrome) before long runs: memory is
the tight resource on this machine. `--capture <folder>` saves a picture of the window after each
phase. Another gripper from the matrix can be given with `--gripper` for objects where it has robust
grasps; only the 2F-140 has been run in batches, the others through a few trial cycles (see
[Other grippers](#other-grippers-first-trials-2026-11-14)). `CELL_DEBUG_STEPS=N` prints joint
tracking and contacts for the first N steps of every move.

| File | What it does |
|---|---|
| `cell.py` | Builds the cell: table, UR10, gripper mounted on the flange, object, place marker |
| `planner.py` | Motion planning, as Kit's processes use it: hands everything to the planning process (cuMotion must not run in Kit's process: `docs/web-stream-cumotion-crash.md`) |
| `cumotion_planner.py`, `planner_process.py` | cuMotion's trajectory optimizer (the UR10 plus collision spheres for the gripper, box obstacles), and the process it runs in (tests: `tests/test_planner_process.py`) |
| `controller.py` | One pick-and-place cycle as a phase machine, stepped once per physics step; the simulation pauses while it waits for a plan |
| `run_cell.py` | Runs cycles and stores them in `results/pick_place.sqlite` |
| `cell_app.py` | The cell in a window: choose the gripper and the object, run cycles one at a time (results in `results/pick_place_interactive.sqlite`) |
| `cell_control.py` | The window's rules without Kit: choices, running and stopping, what it says; the panel draws its state and events (tests: `tests/test_cell_control.py`) |
| `web_bridge.py` | The window's controller for a web page: the page's requests in, events and state out, over JSON messages (no Kit; tests: `tests/test_web_bridge.py`) |
| `livestream.py` | `web_bridge.py` on Kit's stream, for `cell_app.py --web`: turns on the WebRTC stream and its messaging (Kit only) |
| `remote_python.py` | Kit's Python from outside while the cell runs: `cell_app.py --dev` turns on Isaac Sim's Python server, and the script sends it code (the client needs only the standard library; tests: `tests/test_remote_python.py`) |
| `viewport_cameras.py` | The viewport's cameras for the web page, which has no Kit menus: which there are, which one it looks through, switching; and the stage's unit, for the page's legend (`KitViewport` in Kit; tests: `tests/test_viewport_cameras.py`) |
| `cell_session.py`, `cell_args.py` | The cell set up for running cycles, and the options, shared by `run_cell.py` and `cell_app.py` |
| `run_batch.py`, `report.py` | Several objects headless; `results/pick_place_summary.md` |
| `cell_db.py` | Database schema, and reading robust grasps from the grasp matrix |
| `camera_model.py` | Step 4. The wrist camera's optics, its mount on each gripper, look pose and simulated stereo depth (no Isaac Sim needed) |
| `wrist_camera.py` | Step 4. The camera in the stage, and the object's model points from its mesh |
| `perception.py` | Step 4. The object's pose from one depth image (no Isaac Sim needed) |
| `handeye.py` | Step 4. Hand-eye calibration with a ChArUco board, per gripper; `results/perception/handeye.json` (2F-140), `handeye_<gripper>.json` |
| `capture_views.py`, `perception_eval.py` | Step 4. Save the camera's view of each start pose; measure the perception on them offline |

### The interactive window

```powershell
C:\issacsim-6-2\python.bat pick_place\cell_app.py                         # pose from the simulation
C:\issacsim-6-2\python.bat pick_place\cell_app.py --perception camera     # from the wrist camera (step 4)
# step 5: start the locator first (ros2_ws\README.md), and set RMW_IMPLEMENTATION=rmw_fastrtps_cpp
C:\issacsim-6-2\python.bat pick_place\cell_app.py --perception ros
C:\issacsim-6-2\python.bat pick_place\cell_app.py --web                   # no window: the viewport streamed for a web page
C:\issacsim-6-2\python.bat pick_place\cell_app.py --dev                   # with any of these: Kit's Python, from outside
```

Isaac Sim opens with a *Pick-and-place cell* panel (a tab next to *Property*): a gripper list, an
object list, the start pose number, *Run 1 cycle*, *Run 5 cycles*, *Stop after this cycle*, the
cycle's latest step, and the results so far. Choosing another object swaps it in the stage while the
simulation is stopped; the arm, camera, planner and ROS 2 bridge stay as they are. The start pose is
seeded, as in the batch runs: start pose 3 is the batch's cycle index 3 (logged there as cycle 4/10).
Changing the number moves the object to that start, so it can be seen before running.

Choosing another gripper builds the cell again on a new stage, which takes a few seconds: the
gripper is merged into the arm's articulation, and the planner's collision model of it changes. The
object stays if the new gripper has robust grasps for it, otherwise the list's first object is
taken. With the camera, each gripper carries it at its own place and has its own hand-eye
calibration (see [the camera](#step-4-the-wrist-camera-eye-in-hand)); a gripper without one is marked
in the list and cannot be chosen. With `--perception ros` the gripper stays the one given with
`--gripper`, because the locator's camera mount comes from its launch file; at start-up the cell checks
that the locator's mount is this gripper's calibration.

The gripper list offers every calibrated gripper of the grasp matrix, the object list every object
with robust grasps for the chosen gripper, and marks the ones run in batches (the five, with the
2F-140). The others were never run through whole batches: some are larger than the area the camera
searches (0.18 m around the pick spot), and some have only one or two robust grasps, which the arm
cannot reach from many start poses.

The options are `run_cell.py`'s (`--perception`, `--look-first`, `--gripper` for the gripper to start
with, ...), plus `--grippers` and `--objects` to offer fewer, and `--script` to run steps as if
clicked (for tests and demos; see the file's docstring). Results go to
`results/pick_place_interactive.sqlite`, not the batch database. Each object's resting pose on the
grasp matrix's table is found by letting it settle, which needs a stage of its own and so cannot
happen once the cell is built: the first start settles every object offered, for every gripper, and
keeps the poses in `results/cell_rest_mounts.json`, and later starts read the file. With
`--perception ros`, the object's model is written to `results/perception/models` for the locator if
it is not there yet.

`--web` runs Isaac Sim without its window and streams the viewport over WebRTC (signalling on
49100/TCP, media on 47998/UDP), for a web page that draws the panel itself (`web/`, see its README):
`web_bridge.py` answers the page, and `livestream.py` puts it on the stream. The log says when the
page can connect. Kit's own panel runs too, hidden with the rest of Kit's UI
(`--/app/window/hideUi=false` streams it with the viewport); both draw the same controller, so they
agree. Having no Kit menus, the page switches the viewport's camera through `viewport_cameras.py`,
and names the stage's unit itself, so the stream leaves out Kit's unit box (the `m`).
Kit streams to one page at a time. Until 2026-11-20 every cycle crashed Isaac Sim while a
page was connected; the planner now runs in a process of its own (`docs/web-stream-cumotion-crash.md`).
While a cycle waits for a plan, the simulation pauses and Kit goes on drawing frames, about 10 a
second, so the page keeps its stream (before, the plans at a cycle's start held Kit up for 5 to 6 s,
and the page could lose the stream).

`--dev` turns on Isaac Sim's Python server (`isaacsim.code_editor.python_server`, the one Isaac
Sim's agent skills talk to), so that the running cell can be looked into from outside, without
starting Isaac Sim again for each question:

```powershell
python pick_place\remote_python.py -c "omni.usd.get_context().get_stage_url()"
python pick_place\remote_python.py probe.py        # a file of code
```

What is sent runs in Kit's Python, between the cell's steps, and can do anything the cell's own code
can. Looking is harmless; changing the stage while a cycle runs changes the cycle, and its result
then says nothing about the cell. The server listens on 127.0.0.1 only and runs only code that
carries the token `cell_app.py` makes at each start; the log says when it listens. The token is
left, with the port, in a file in the user's temp folder (`remote_python.connection_file`) for the
client, and the file is removed at exit. Kit also prints the token in its log. For two Isaac Sims
with `--dev`, give one another port: `--/exts/isaacsim.code_editor.python_server/port=8227`, and
`--port 8227` for the client.

Tested on 2026-11-23 with `--web --dev`, *Run 1 cycle* scripted on the toy block:
- The server listened once Kit had started, before the cell was built. Mid-cycle, the client read
  the stage's prims and the timeline, in about 0.2 s an answer.
- A name defined in one request was there in the next. A top-level `await` of Kit's next update
  worked, and an error came back as its traceback.
- A wrong token and no token were both refused, and the code did not run. The server listened
  on 127.0.0.1:8226 only.
- `post_quit()`, sent through the client, closed Isaac Sim cleanly, and the file was gone.
- Started again with the port already taken, the server could not listen: the log said so and why,
  and no file was left. The same cycle then came out identical, in every stored column but
  wall-clock times, to the first run's, which had been looked into mid-cycle.

Use the panel, not Isaac Sim's own controls, to run things. The toolbar's Play and Stop, and
selecting and dragging prims in the viewport, change the simulation under the cycle:
- A Stop or Pause during a cycle abandons the cycle, with a message saying so. While the cycle
  waits for a plan, the simulation is paused by the cycle itself (the toolbar shows it): a Stop then
  abandons the cycle too, and a Pause is undone when the plan comes.
- A moved wrist camera spoils every estimate after it, by a constant offset: its images stop
  matching the hand-eye calibration. That is the likely cause of a session on 2026-11-11 whose
  estimates went 12, then a steady 40 mm off, whatever the object. A scripted replay of the same
  sequence, untouched, stayed within 2.3 mm; the camera's moving is inferred from the steady offset,
  not seen in the log. Before each cycle the
  panel now puts the camera back on its mount if it was moved, and says so. A moved robot base
  stops the run.

While idle, the window shows the next cycle's start: the object at its start pose and the arm at
home. The stage itself holds the arm at zero joint angles, lying flat over the table; the arm goes
to home only once the simulation runs. So the window runs two physics steps and pauses. A cycle
always starts from a stopped simulation, as in the batch runs. With ROS 2, the status bar may show
*"ROS2 TF aggregation contributor '/ROS2Bridge/TF' did not submit transforms"* while paused. That is
the bridge noticing the pause, and it does no harm.

Tested on 2026-11-11, scripted:
- The camera: the mug, then the figurine, which had never been run through cycles. Both were
  placed, 2.5 and 1.6 mm from the target, and their poses were found 0.2 and 0.5 mm off.
- ROS 2: the mug, the lemon, the figurine. The locator switched objects with them, read the
  figurine's model that the window had just written, and both cycles were placed.

Tested on 2026-11-14, scripted, with the camera: the 2F-140 on the mug (start pose 0), then the RG2 on
the toy block, the Franka Hand on the jar, and back to the 2F-140 on the mug at start pose 0. The two
2F-140 cycles came out identical in every stored column but wall-clock times. The first start settled
the 56 gripper x object pairs not yet in `results/cell_rest_mounts.json` in about a minute.

The same script, run three more times on 2026-11-18, showed that windowed runs do not always repeat
exactly. One run matched 2026-11-14 in every column. In another, the Franka cycle came out slightly
different (same grasp, same outcome; motion 7.27 s against 7.23 s, finger closure 0.772 against
0.767). In the third, with the code as committed before, the Franka cycle and the final 2F-140 cycle
differed (placed 2.5 mm from the target, not 1.8 mm). The found poses were identical every time, so
the variation is in the motion and physics, not the camera; where it comes from is not known yet.

The panel's rules live in `cell_control.py`, which imports nothing of Kit: what can be chosen when,
running and stopping, and the lines the panel shows. The panel only draws the controller's state and
events, and its widgets call the controller's actions; `KitCell` in `cell_app.py` does the work in
the stage. Its tests use a fake cell and need no Isaac Sim:

```powershell
python -m unittest discover -s pick_place/tests     # any Python 3.10+, standard library only
C:\issacsim-6-2\python.bat -m unittest discover -s pick_place/tests   # also the planner's (need numpy)
```

The planner's tests (`tests/test_planner_process.py`) run Kit's side and the planning process with a
fake planner instead of cuMotion. They need numpy, so a plain Python skips them; Isaac Sim's
`python.bat` has it, and runs them without starting Kit.

`tests/test_web_contract.py` also records one session of the bridge's messages for the web page's
tests (`web/src/cell/contract.json`), and fails when they change: run it with `--write`, then the
page's tests.

Moving the rules there (2026-11-18) left the window's results as they were: the script above came out
identical to 2026-11-14 in every column on one of two runs, and the other differed only in the Franka
cycle, as the code before the move also did.

The refactoring into `cell_session.py` left `run_cell.py`'s results unchanged: the toy block (3
cycles, pose from the simulation) and the mug (2 cycles, camera) came out identical before and after,
in every stored column except wall-clock times.

Runs with the window open do not always repeat the headless batch runs. On the mug's start pose 0
with the camera, the headless batch chooses grasp 180, and windowed runs (`run_cell.py` without
`--headless`, and `cell_app.py`) choose grasp 116. The found pose agrees to 0.01 mm. The grasp choice
is that sensitive: in step 5 too, the ROS 2 runs chose differently on 3 of 50 cycles (see Step 5).
The measured results are the headless batches'.

## The cell

- **Robot**: Isaac Sim's UR10 asset (downloaded from NVIDIA's asset server to
  `assets/universal_robots/ur10`), because cuMotion ships a matching UR10 robot description. Its base
  sits at the world origin on the tabletop (z = 0, the same height as the matrix's table).
- **Gripper**: the same SimReady asset and calibration as in the matrix. `cell.py` removes its
  articulation root and its joint to the world and fixes its base to the UR10's `wrist_3_link`, so it
  becomes part of the arm's articulation. That link's frame is cuMotion's `tool0`, which every cycle
  checks (to 2 mm and 0.5°). The gripper's frame sits on it, so a stored grasp pose is directly the
  pose to send the tool to.
- **Object**: starts at the pose it settled to in the matrix (recomputed with the matrix's own
  code), turned about the vertical by up to 90° and shifted by up to 3 cm, seeded per cycle. The
  stored grasps are moved with it.
- **Pick spot** (0.75, −0.35) and **place spot** (0.75, 0.35), 0.7 m apart, in front of the robot.
- **Physics**: PhysX on the CPU at 60 steps a second, the matrix's step (see 4 below on the solver).

## A cycle

| Phase | Motion | Gate (failing ends the cycle) |
|---|---|---|
| Choose a grasp | The planner gets the pre-grasp pose (10 cm back along the approach axis) of every robust grasp at once, as a goalset, and picks the one it reaches best. Every straight move below is then planned from there, before the arm moves; if one cannot be, that grasp is ruled out and the planner picks again | `unreachable` if no grasp is left (or 10 were ruled out) |
| To pre-grasp | Collision-free, with the table and the object as boxes | |
| Approach | Straight in along the approach axis, orientation fixed | Tool within 3 mm and 1° of the grasp |
| Close | Fingers to the close target, 1 s (as in the matrix) | `missed` if they close to where the pads meet |
| Lift | Straight up 15 cm | `dropped_lift` if the object rose less than half that |
| Transport | Straight line to 15 cm above the place pose | `dropped_transport` if the object moved 2 cm in the hand |
| Lower | Straight down, aimed so the object (not the tool) ends 3 mm above its rest pose, which absorbs any slip; but never lower than the nominal place pose | Tool within 1 cm |
| Release, retract, home | Open; back out 10 cm along the approach axis; home | |
| Result | After 1 s | `placed` if within 2 cm of the target pose and tilted under 10°, else `misplaced` |

Every motion also stops as `tracking` if the tool falls 2 cm behind the planned path. The moves
executed are the ones planned before the arm moved. Only the lower and the retract are planned again
during the cycle, for the slip, and they fall back to the pre-planned move if that fails.
The place pose is the grasp moved by the same shift as the object, so the gripper keeps its grasp
orientation and the object lands in the pose it was picked in.

The planner checks the arm and the gripper (spheres around each gripper body) against the table
and, on the way to the pre-grasp pose, the object. During approach, lift, lower and retract the
table is left out: those are short straight moves, and the matrix already checked that the
fingertip pads clear the table at the grasp. The carried object is not part of the planner's model;
the transport is a straight line at 15 cm.

## What it took to make it work

Each of these failed a cycle, or the whole process, until fixed:

1. **Joint velocity targets.** Streaming only position targets made the drives' damping pull
   against the motion (the velocity target was zero). After about 0.25 s the UR10's `wrist_1` drive
   stopped holding, and the wrist fell back under gravity, 0.8 rad off its target. Velocity targets
   from the trajectory fixed it.
2. **Slower playback.** cuMotion times its trajectories at the URDF's joint limits. The simulated
   UR10's drives are force limited (56 N·m at the wrist) and lag at that pace, which takes the
   gripper off the planned, collision-free path. Motions are played at half speed (`--speed`).
3. **Solver iterations of the merged articulation.** The 2F-140 is simulated with 64 position
   iterations, the UR10 with 32. Merged at 32, a side grasp with the object wedged in the finger
   linkage spun `wrist_2` and `wrist_3` during the lift. The cell now gives the merged articulation
   the larger count.
4. **CPU physics.** Isaac Sim's default for this scene is PhysX's GPU dynamics and broadphase (with
   results read back to the CPU), which is also what the grasp matrix ran. With it, creating the
   second cuMotion optimizer of a cycle crashed Isaac Sim in 3 of 3 runs. Planning repeatedly with no
   physics never crashed, and with PhysX on the CPU no run has crashed in over 70 cycles. So the cell
   runs PhysX on the CPU. **The matrix and the cell therefore use different PhysX solvers**, which
   may account for part of any difference between their results. (2026-11-20: very likely CUDA work
   in Kit's process faulting cuMotion's managed memory, as the web page's stream later did;
   cuMotion now plans in a process of its own. See `docs/web-stream-cumotion-crash.md`. The
   physics stays on the CPU, so results stay comparable.)
5. **Plan the whole cycle before moving.** At first only the motion to the pre-grasp pose was planned
   up front, and each straight move was planned when its turn came. 8 of 30 cycles then failed with
   a move that could not be planned: the planner had picked a grasp whose approach, transport or
   lower was infeasible from there. Planning them all first, and ruling such grasps out, fixed that.
   Planning a move again from where the arm actually stopped sometimes failed even when the plan
   from the planned stop, a fraction of a millimetre away, had succeeded. So the pre-planned moves
   are the ones executed.
6. **Don't lower the fingers below the place pose.** The lower aims so the object, not the tool,
   lands on the table. A side grasp on the jar stopped 2 cm short of its lowered pose with the
   shoulder at its 330 N·m limit, pressing down. The likely cause: the jar had slid towards the palm,
   and the aim put the fingertips into the table, which they cleared by only a few millimetres at the
   grasp. The aim is now capped at the nominal place pose, and the same cycle placed the jar.
7. **Robot at the world origin.** The UR10's root joint has an empty `body0`, as the Franka Hand's
   did, which pinned that hand to the world origin in the matrix. The cell is built around the origin
   rather than moving the robot.

## Results (2026-11-09)

The 2F-140 on five objects, 10 cycles each, the object turned by up to 90° and shifted by up to 3 cm.
`results/pick_place_summary.md` has the full table and every failed cycle; `results/pick_place.sqlite`
the raw data. `results/renders/cell_mug/` has a picture of each phase of one cycle on the mug.

| Object | Robust grasps (matrix) | Placed | Failures | Side grasps chosen | Cycle s | Place error mm |
|---|---|---|---|---|---|---|
| Toy block | 31 | 9/10 | misplaced 1 | 2 | 10.0 | 0.4 |
| Mug | 28 | 9/10 | unreachable 1 | 0 | 16.1 | 0.4 |
| Jar | 45 | 10/10 | – | 6 | 16.3 | 1.2 |
| Beer stein | 34 | 7/10 | dropped in transport 2, unreachable 1 | 3 | 13.4 | 0.9 |
| Lemon | 49 | 8/10 | unreachable 2 | 6 | 16.3 | 0.2 |

**43 of 50 cycles placed (86%).** Cycle times and place errors are medians over placed cycles; the
cycle runs from the first planned motion to home, at half the planner's speed. Headless, a cycle took
about 30 s of wall time.

- **Placed objects land accurately**: median error 0.2 to 1.2 mm, and all but one within 5 mm; tilt
  at most 1.3°. Aiming the lower by the object rather than the tool absorbs the slip in the fingers
  (median 0 to 2.3 mm).
- **Unreachable (4 cycles)**: for those start poses the planner found no motion to the pre-grasp pose
  of any robust grasp. The mug has the fewest robust grasps (28). The planner's own failures can't
  be ruled out here, since it did fail spuriously on straight moves (see 5 above).
- **A robust grasp can still drop the object in transport.** Grasp 38 on the beer stein held under all
  three of the matrix's conditions, but each time the planner chose it, the stein slid 33 to 35 mm in
  the fingers during the carry. The matrix tests the hold against gravity alone; the arm also
  accelerates the object sideways.
- **Side grasps on the toy block let it turn in the fingers**, by 8° and 9° in two cycles. One fell
  over when released (tilted 89°); the other landed 18 mm off, the one placed cycle beyond 5 mm.
  The lower corrects the object's position for slip, not its orientation.
- **The planner uses few of the grasps**: 4 or 5 different ones per object over 10 cycles, chosen for
  reachability and motion, not for grasp quality. All of them are robust in the matrix.

## Step 4: the wrist camera (eye-in-hand)

In step 3 the simulation told the robot exactly where the object was. Here a depth camera on the
wrist looks at the pick area first, and the object's pose comes from that image alone. The grasps,
the planner's obstacle and the place target are all computed from the estimate; the true pose is
still read, but only to judge the gates and score the result.

```powershell
# once: hand-eye calibration (writes results/perception/handeye.json; another gripper: --gripper onrobot_rg2)
C:\issacsim-6-2\python.bat pick_place\handeye.py
# watch it
C:\issacsim-6-2\python.bat pick_place\run_cell.py --object mug_A07RDPW843 --perception camera
# several objects headless
.venv\Scripts\python.exe pick_place\run_batch.py --objects toy_block_6MPL7NVFVU,mug_A07RDPW843 --cycles 10 --perception camera
# the perception alone: save the camera's view of each start pose once, then measure offline
C:\issacsim-6-2\python.bat pick_place\capture_views.py --object mug_A07RDPW843
C:\issacsim-6-2\python.bat pick_place\perception_eval.py --noise perfect 1 2 --handeye results/perception/handeye.json --write
```

`--depth-noise` scales the depth noise (or `perfect`), `--handeye none` uses the mount as built
instead of the calibration. `--look-first --no-slip-correction` runs the step 3 controller (pose from
the simulation) under the camera mode's conditions, for comparisons. `--perception blind` is the
baseline without the camera: the robot takes the object to be at its nominal pose (at the pick spot,
not turned) and otherwise runs as in the camera mode (from the look pose, no slip correction).

### The camera

- **A RealSense D455, more or less.** Isaac Sim's D455 asset gives the optics (1.93 mm focal length on
  a 3.896 mm sensor: 90 x 64 degrees), at 640 x 400 pixels. The asset itself is not used: its body is
  a rigid body (it cannot sit inside the arm's articulation) and its render product has no resolution
  set (Replicator's default is 2048 x 1080).
- **On the gripper**, 2.5 cm outside the 2F-140's body on the side away from its fingers' travel and
  3 cm out from the flange, looking along the approach axis. The fingers show in the bottom corners of
  the image. The housing is a box of the D455's size, visual only; the planner knows it as spheres.
  The other grippers carry it by the same rule, each beside its own base (`camera_model.MOUNT_XYZ`):
  the Robotiq 2F-85 and Hand-E where the 2F-140 does (the same base), the Franka Hand 5.5 mm closer
  in (a narrower hand), the OnRobot RG2 and 2FG7 on their -y side, as they close along x. The RG2's
  and 2FG7's taller bodies fill the bottom third of the image; the object, in the middle of the image
  at the look pose, stays in view, and the perception leaves out everything more than 0.30 m above
  the table, the gripper included.
- **Depth** is the renderer's perfect depth with the effects of an active stereo camera added
  (`camera_model.simulate_depth`): disparity noise (about 1 mm at 0.45 m; an assumption, not a
  calibration), no depth where the second imager cannot see (beside near edges), on depth steps, or
  closer than the disparity search reaches. Isaac Sim has a stereo depth model of its own
  (`SingleViewDepthCameraSensor`), but it models passive stereo, which needs surface texture: on the
  plain table and objects it gave depth for 1 to 2% of the pixels. A D455 projects a dot pattern and
  measures such surfaces.
- **The look pose**: before choosing a grasp the arm puts the camera 0.22 m from the pick spot towards
  the robot's base, 0.45 m above the table, so it sees the object's top and the side facing the robot
  from about 28 degrees off vertical. One image is taken there.

### Hand-eye calibration

The camera's pose on the gripper is measured, not taken from the drawing (`handeye.py`). A ChArUco
board (5 x 4 squares of 50 mm, 38 mm markers) lies on the table; the arm shows it to the camera from
40 viewpoints 0.25 to 0.38 m away, tilted and rolled differently. At each, OpenCV finds the corners
and the board's pose in the camera (solvePnP, 0.06 to 0.26 px reprojection error), and the joints
give the gripper's pose. `cv2.calibrateHandEye` solves AX = XB with five methods; the one kept is the
one whose camera pose puts the board at the most consistent place in the world over the 40 views
(Park: 0.5 mm, 0.29 degrees spread), a check a real cell can make without ground truth.

Against the mount as built (which only the simulation knows) the calibration is 0.46 mm and 0.24
degrees off. The first attempt used 30 mm markers and views up to 0.45 m away: only 5 of 20 views
found enough corners (3 to 5 pixels per marker bit), and the result was 2.9 mm and 1.0 degree off.
One degree at 0.45 m is 8 mm on the object.

Each gripper carries the camera at its own place, so each has its own calibration, as a real cell
needs one after a tool change (`handeye.py --gripper <name>`, about a minute each; the 2F-140's is
`results/perception/handeye.json`, the others' `handeye_<gripper>.json`). Run on 2026-11-14, the same
40 viewpoints and the same selection by board spread:

| Gripper | Method kept | Views used | Board spread | Off the mount as built |
|---|---|---|---|---|
| 2F-140 | Park | 40 | 0.51 mm, 0.29° | 0.46 mm, 0.24° |
| 2F-85 | Park | 39 | 0.46 mm, 0.20° | 0.31 mm, 0.04° |
| Hand-E | Park | 39 | 0.48 mm, 0.23° | 0.29 mm, 0.07° |
| Franka Hand | Daniilidis | 39 | 0.43 mm, 0.20° | 0.11 mm, 0.05° |
| RG2 | Horaud | 39 | 0.52 mm, 0.24° | 0.45 mm, 0.07° |
| 2FG7 | Horaud | 38 | 0.51 mm, 0.24° | 0.87 mm, 0.17° |

Tsai, Park, Horaud and Daniilidis agree within 0.01 mm of board spread on every gripper, and within
0.2 mm of each other against the mount as built, so which one is kept matters little; Andreff is the
outlier each time (1.8 to 3.2 mm of spread, 4.9 to 8.9 mm off).

### Finding the object (`perception.py`)

1. Depth pixels become points in the world, with the camera's pose from the arm's joints and the
   calibrated mount, as a real robot would have it.
2. Points within 18 cm of the pick spot: the table plane is fitted (RANSAC) and removed, and the
   largest connected cluster above it is the object's visible surface (1300 to 3900 points).
3. The object's model, 3800 to 8800 points sampled from its visual mesh, is registered to it.
   4000 starting orientations spread over all rotations, each placed with its camera-facing side on
   the observed points and its lowest point on the table, are ranked coarsely; the best 300 get a
   short point-to-plane ICP and are scored, and the best 8 of those a full ICP. The score is the share
   of observed points the model explains, minus model points that the camera would have seen in
   front of what it saw (free space), minus model points sunk into the table.

No object is assumed symmetric. For the jar (about its axis) and the lemon the orientation about the
symmetry comes out arbitrary; the grasps then land on equivalent surface, and the place target is
computed from the object's centroid, which the estimate does get right (see 4 below). It takes 3.4
to 6.6 s of computing per image on this PC.

### What it took

1. **Plain Replicator annotators, not Isaac Sim's camera wrapper.** With `isaacsim.sensors`'
   `CameraSensor` attached to the camera, even if it was never read, creating the second cuMotion
   optimizer crashed Isaac Sim (3 of 3 runs), the same crash as with GPU physics in step 3. With
   `omni.replicator` annotators on the CPU it did not (6 of 6 plans, then every cycle since). The
   cause was very likely the one found on 2026-11-20 (`docs/web-stream-cumotion-crash.md`).
2. **A search that refines before it judges.** The first version fully refined only the 16 best of
   3000 coarse starts. Starts near the true pose ranked 100th to 450th on the coarse score, and the jar
   came out upside down in most noisy images while the true pose scored 1.0 against 0.6. The coarse
   rank now only preselects; the refined candidates' scores decide.
3. **A bigger calibration board**, as above.
4. **Place by the centroid, not the body origin.** The robot moves the object by what takes it, as
   seen, to the place spot. Computed with the body origin, a near-symmetric object's arbitrary
   orientation moved the origin: the lemon's origin is 37 mm below its centroid, an estimate turned 43
   degrees put it 27 mm off, and the lemon was put down 25 mm from the target. With the centroid the
   same cycle placed it 3.6 mm off.
5. **Plan the way home around the placed object.** The planner's object box stayed at the pick spot
   after the place. In one camera cycle a mug that had been released 2 mm from its target ended up
   109 mm away; rerun alone the same cycle placed it, so the cause is not proven. The way home is now
   planned around the object where the robot put it, and in the 50 camera cycles since, no placed
   object moved after release.

### Results (2026-11-10)

The 2F-140 on the same five objects and the same 10 start poses per object as step 3 (same seed),
under four conditions. *From the look pose*: the grasp choice starts at the camera's look pose, with
the camera's housing in the planner's model, as in the camera mode. *No correction*: the lower goes
to the place pose as planned, without the in-hand correction of step 3 (which needs the true pose).

| Object | Step 3: pose from the simulation | Step 4: pose from the camera | Simulation, from the look pose, no correction | Simulation, no correction |
|---|---|---|---|---|
| Toy block | 9/10 | 10/10 | 10/10 | 9/10 |
| Mug | 9/10 | 10/10 | 10/10 | 9/10 |
| Jar | 10/10 | 10/10 | 10/10 | 4/10 |
| Beer stein | 7/10 | 7/10 | 7/10 | 7/10 |
| Lemon | 8/10 | 10/10 | 10/10 | 8/10 |
| **All** | **43/50** | **47/50** | **47/50** | **37/50** |

- **The camera costs nothing here.** The camera mode and the third column differ only in where the
  pose comes from. Cycle by cycle, 46 cycles placed in both, one in each only and two in neither, all
  on the beer stein, whose failures are drops in transport (grasps 38 and 102) in every condition.
- **The pose estimates** (in the cycles, with the calibrated mount and the depth noise): centroid 0.5
  to 0.7 mm off (median per object; at most 1.3 mm), ADD-S 0.6 to 1.1 mm. The jar's orientation
  about its axis is arbitrary (median 99 degrees off), as is the lemon's once (43 degrees); both
  were placed 10 of 10.
- **Better than step 3, but not thanks to the camera.** Starting from the look pose, the planner
  chose top-down grasps (one side grasp in 50 cycles, against 17 in step 3) and found a motion every
  time: step 3's four `unreachable` cycles were the planner failing from home, not grasps out of reach.
- **Placing is less accurate without the in-hand correction**: median place errors of 1.4 to 8.1 mm
  per object with the camera and 1.6 to 8.2 mm with the simulation's pose under the same conditions,
  against 0.2 to 1.2 mm in step 3. Perception adds little to that.
- **Robust grasps that push.** Without the correction, the jar's side grasp 115 put the jar down 35
  to 45 mm off target in all 6 cycles that chose it, upright and without slipping. Measured along the
  cycle: the approach pushed the jar 8 to 22 mm, and after closing it sat 37 to 45 mm from where the
  grasp expected it. The matrix rates 115 robust and recorded 1.7 mm of movement while closing, but it
  starts its gripper at the grasp pose and never simulates the approach. Step 3 corrected the lower
  with the jar's true in-hand pose and so placed it within 1 mm, hiding the push. The camera mode did
  not choose grasp 115, so its 10/10 on the jar is luck, not a fix.

### Without the camera (2026-11-14)

Does the camera matter, or would the robot have placed the objects anyway? `--perception blind` runs
the same 50 start poses with the robot taking every object to be at its nominal pose, at the pick
spot and not turned; everything else is as in the camera mode.

| Object | Pose from the camera | Blind: nominal pose |
|---|---|---|
| Toy block | 10/10 | 4/10 |
| Mug | 10/10 | 3/10 |
| Jar | 10/10 | 6/10 |
| Beer stein | 7/10 | 4/10 |
| Lemon | 10/10 | 6/10 |
| **All** | **47/50** | **23/50** |

- **Blind works when the object happens to lie near where it is expected.** The start poses are
  shifted 6 to 35 mm from the pick spot and turned up to 90 degrees. Of the 20 cycles shifted 11 mm or
  less, blind placed 19; of the 30 shifted 22 mm or more, 4. A blind place is off by about the start
  shift (most misplaced cycles 26 to 34 mm, against a 20 mm tolerance), since the robot moves the
  object as if it had started at the pick spot.
- **The four exceptions are round objects**: the jar and the lemon, two cycles each, were moved
  towards the gripper's centre by the approach or the closing fingers, which took out most of the
  shift (placed 3 to 18 mm off).
- **Contacts it does not expect.** In three mug cycles the gripper ran into the mug on the way in (the
  approach is planned around the mug where it is assumed to be) and fell 22 to 70 mm behind its path.
  The beer stein was dropped in five cycles, all shifted 22 mm or more; in two it never rose.
- **Cycle by cycle against the camera**: 21 placed by both, 26 only with the camera, 2 only blind (the
  beer stein, whose marginal grasps drop in every condition), 1 by neither.
- **Blind chose the same grasp in nearly every cycle** (one per object, two for the toy block), since
  it always aims at the same pose.
- **The gap measures the test's spread, not the camera alone.** With objects fed to a fixed spot (a
  jig or a fixture) blind would do as well; with more spread it would do worse. The camera's value
  here is that the cell no longer needs the object where it is expected.

`results/pick_place_summary.md` has every condition, the cycle-by-cycle comparisons and every
failed cycle; `results/perception_summary.md` the perception alone on the 50 saved views at other
noise levels.

### Other grippers: first trials (2026-11-14)

The batches above are all the 2F-140's. The other five grippers were first run through a few cycles
each, to check that the cell works with them, on an object with robust grasps for each (start poses
0 to 4; scratch databases, not in `results/`):

| Gripper | Object | Robust grasps | Pose from the simulation (3 cycles) | Camera (5 cycles) | Camera's pose error |
|---|---|---|---|---|---|
| 2F-85 | toy block | 69 | 2 placed, 1 tracking | 4 placed, 1 dropped | 0.3 to 1.0 mm |
| Franka Hand | jar | 16 | 2 placed, 1 tracking | 3 placed, 2 dropped | 0.4 to 0.7 mm |
| Hand-E | beer stein | 25 | 2 dropped, 1 misplaced | 2 placed, 2 tracking, 1 misplaced | 0.2 to 0.5 mm |
| RG2 | toy block | 67 | (did not mount, since fixed) | 1 placed, 2 tracking, 1 dropped, 1 misplaced | 0.4 to 1.4 mm |
| 2FG7 | jar | 27 | 2 tracking, 1 dropped | 3 tracking, 2 dropped | 0.8 to 1.1 mm |

The camera found the object to about a millimetre with every gripper (centre error; the jar is turned
about its axis by up to 144 degrees in the estimate, which its symmetry hides: ADD-S 1.0 to 1.2 mm).
What fails is the grasp, and mostly the approach: the arm stops short of the grasp pose (`tracking`,
3 to 24 mm off) because the fingers meet the object or the table on the way in, or the approach
pushes the object (12 to 18 mm, the Hand-E's stein) before the fingers close. That is the limit noted
under step 4, that the grasp matrix starts its gripper at the grasp pose and never simulates the
approach. It bites harder on these grippers than on the 2F-140, why exactly is not looked into yet;
the two that open least (Hand-E 50 mm, 2FG7 38 mm of stroke) are among the worst. So the matrix's
robust rate does not rank the grippers for this cell: the RG2, second in the matrix (33% robust),
placed 1 of 5. These are a handful of cycles each, enough to see the cell works and where it fails,
not to rank the grippers.

The RG2 at first did not mount at all: its asset ties its base to the gripper's root Xform, not to
an empty target, and `cell.mount_gripper()` looked only for the latter. It now treats a joint to a
prim without a rigid body as a joint to the world too (it finds the same joint as before on the
other five).

## Step 5: ROS 2

`run_cell.py --perception ros` makes the cell a ROS 2 system. Isaac Sim plays the robot: through its
ROS 2 bridge it publishes `/clock`, `/joint_states` and the arm's TF tree every physics step, and the
wrist camera's depth, colour and intrinsics when the cycle looks (`ros_link.py`). The
`object_locator` node (`ros2_ws/src/pick_place_ros`, on ROS 2 Jazzy for Windows) gets the camera's
pose from TF, the arm's links from Isaac Sim and the camera mount from the hand-eye calibration
published as a static transform. It runs the step 4 perception unchanged and answers on
`/object_pose`. The cycle waits for that answer and goes on as in step 4. RViz2 shows the TF tree, the
camera image, the points taken as the object and the model at its estimated pose. How to run it, the
topics and the setup are in `ros2_ws/README.md`.

| Object | Camera, in Isaac Sim's process (step 4) | Camera via ROS 2 (step 5) |
|---|---|---|
| Toy block | 10/10 | 10/10 |
| Mug | 10/10 | 10/10 |
| Jar | 10/10 | 10/10 |
| Beer stein | 7/10 | 7/10 |
| Lemon | 10/10 | 10/10 |
| **All** | **47/50** | **47/50** |

The same estimates (centroid 0.57 mm off at the median, 1.28 mm at most), the same success. 43 of the
50 cycles had the same outcome with the same grasp. The beer stein's grasps 38 and 102 dropped or
held in four cycles either way: they are that marginal. In three cycles a different grasp was
chosen, after an estimate that differed in the last digits (the ROS 2 environment has numpy 1.26 and
scipy 1.14, Isaac Sim numpy 2.3 and scipy 1.17); those were placed either way. The answer took 4.3 s
of wall time at the median against 4.0 s in-process: about 0.3 s for the images over DDS and the
extra point clouds for RViz2. The simulation waits for it, as it does for the in-process perception.

What it took is in `ros2_ws/README.md`: Fast DDS instead of Isaac Sim's default Zenoh, the TF
publisher pointed at the articulation root, a TF buffer that survives the clock restarting every
cycle, and stopping the node's Python process as well as its Windows launcher. No WSL: ROS 2 Jazzy
for Windows and Isaac Sim's bundled Jazzy talk on the same PC.

## Not done yet

- **MoveIt 2** (the next step): plan the motions in ROS 2 instead of with cuMotion inside Isaac Sim,
  driving the UR10 through ros2_control. MoveIt 2 has no Windows build, so it needs Linux (WSL with
  mirrored networking, or Docker).
- The planner does not model the carried object, and the place spot is open table, not a bin.
- The grasp matrix never simulates the approach (it starts its gripper at the grasp pose), so a
  robust grasp can still push the object on the way in (the jar's side grasp 115, above). It could
  start from the pre-grasp pose and move in, and store the object's in-hand pose after closing.
- The robot does not know where the object sits in its hand after closing. Step 3 corrected the
  lower with the true in-hand pose; with the camera there is nothing to correct it with.
- The camera's view is easier than a real one: the model is the very mesh that is rendered, the
  intrinsics and the arm's kinematics are exact, the images are clean, and there is one object on a
  plain table. The depth noise model is an assumption. Learned pose estimators that handle clutter
  and model mismatch (FoundationPose, in Isaac ROS) fit step 5.
- Batches with one gripper only. The other five run (`--gripper`, or the window's gripper list), with
  their own camera mounts and hand-eye calibrations, but have had only a few trial cycles each, and
  their approaches fail far more often than the 2F-140's.
