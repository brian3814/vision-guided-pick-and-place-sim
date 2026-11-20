# Isaac Sim crashed when a web page watched a cycle: cuMotion and CUDA managed memory

**In short.** With the web page (`web/`) connected to `cell_app.py --web`, every pick-and-place
cycle crashed Isaac Sim. The crash was in cuMotion, the motion planner. cuMotion keeps its data in
CUDA *managed memory*, and on Windows the CPU may not touch managed memory while any CUDA work of
the same process runs on the GPU. The WebRTC stream runs CUDA work in Kit's process for every frame
it sends. The fix: cuMotion now plans in a process of its own (`pick_place/planner.py`,
`planner_process.py`, `cumotion_planner.py`), in every mode. It computes exactly what it computed
in Kit, and the cell's results came out the same. Found and fixed on 2026-11-20. A second change
the same day keeps the stream going while Kit waits for plans (the last section).

## The symptom

- `cell_app.py --web`, the page connected, *Run 1 cycle*: Isaac Sim stopped partway through the
  cycle, with exit code 3221225478 (0xC0000006, `STATUS_IN_PAGE_ERROR`).
- Kit's crash reporter put the fault inside `cumotion.dll`: `CreateTrajectoryOptimizer` when a new
  optimizer was made, or cuMotion's SDF code in `plan_to_task_space_target` when an existing one was
  reused. On the Python side the call was `planner.py`'s `_optimizer` or `plan_line`.
- Without a page, the same cycle placed the object (toy block, start pose 0: 0.4 mm from the
  target), with `--web` and in the window alike.
- Choosing grippers, objects and start poses from the page worked; only planning failed.

## How it was narrowed down

The tools were throwaway and not in the repository. A wrapper ran `cell_app.py` through an import
hook that patched the planner (logging each optimizer made, with the main thread's CUDA context),
the bridge and the controller. A small native exception handler, loaded into Kit, logged the
faulting address and the memory region it fell in. Every run counted only if the page's client was
connected at the time of the crash: Kit logs *Client connected* and *Client disconnected*, and a
scripted cycle can start before the page connects.

A cycle makes 9 cuMotion optimizers: 6 at its start, while the robot stands still (the moves it
will need), then the slip-corrected *lower*, *retract* and *home*. With a client connected, the
7th crashed every time.

| Tried, with a client connected | Result |
|---|---|
| GPU memory, sampled | 2.4 of 12 GB at most: not memory |
| `CUDA_MODULE_LOADING=EAGER` | crash at the 7th |
| Stream at 5 fps (`primaryStream/targetFps=5`) | crash at the 7th |
| Plans keep no world or optimizer alive | crash at the 7th |
| Viewport updates off while an optimizer is made | crash at the 7th |
| No data-channel messages during the cycle | crash at the 7th |
| 90 s between connecting and running | crash at the 7th: not time |
| A 6 s stall of Kit's main thread first (the stream logs a timing reset) | crash at the 7th |
| 10 s of zooming the camera first (the encoder busy with moving frames) | crash at the 7th |
| Optimizers reused instead of made | the crash moves into planning itself |
| The main thread's CUDA context before each call | always the device's primary context |
| cuMotion in a CUDA context of its own (`cuCtxCreate`) | crash at the 7th |
| A sleep, a `cuCtxSynchronize`, or both, before the 7th | crash, same address |
| Kit making no cuMotion call once the robot moves | Kit completes the cycle |
| cuMotion in a separate process during a cycle | 23 plans, all fine |

Two readings along the way were wrong, and are kept here because the data misled in the same way
twice:

- *"It crashes once the robot moves."* The 7th optimizer is simply the first after the stream has
  sent a frame: the first 6 are made back to back, in one update of Kit, before any frame goes out.
  A second session settled it with no cycle at all: a planner planning every 120 updates, the
  timeline stopped, made 19 plans with no client, and the first plan after a client connected
  crashed.
- *"A CUDA context of its own fixes it."* The one run that placed the object had lost its client
  6 s before the 7th optimizer, while the 6 plans before it held up Kit's main thread. With the
  client kept, the same set-up crashed.

## The cause

- **The fault.** It was a *write* by `cumotion.dll` to an address in a 16 GB read-write region
  mapped without a file. That looks like CUDA's unified-memory range. Underneath it was an access
  refusal (NTSTATUS 0xC0000022), not a disk error. The address was the same in every cycle run.
- **Why cuMotion.** `cumotion.dll` imports CUDA's managed-memory allocation (`cudaMallocManaged`,
  `cuMemAllocManaged`).
- **Why only on Windows, with the stream.** Windows has only CUDA's basic unified-memory model:
  while the GPU runs CUDA work of the process, the CPU may not touch any managed allocation, the
  programming guide says, "even to different managed memory allocations"
  ([CUDA Programming Guide, Unified and System Memory](https://docs.nvidia.com/cuda/cuda-programming-guide/02-basics/understanding-memory.html)).
  The WebRTC stream does CUDA work in Kit's process for every frame: Kit hands frames over as CUDA
  buffers, `NvStreamServer.dll` scales them with its own CUDA kernels, `NVCodec.dll` encodes with
  NVENC from CUDA input, and the extension ships its own CUDA runtime (`cudart64_12.dll`). Once a
  frame has gone out, cuMotion's next touch of its managed memory faults.
- **Why a separate process works.** A process has managed memory of its own, and no stream work
  runs in the planning process.

This is inferred from the fault data, the DLLs' imports and strings, and CUDA's documentation; the
driver itself was not looked into. It explains every row of the table above.

**Very likely the same: two earlier crashes.** In November, cuMotion crashed with GPU PhysX
(`cell_session.py` has kept the physics on the CPU since) and with Isaac Sim's CameraSensor wrapper
on the wrist camera (`wrist_camera.py` uses plain Replicator annotators since). Their crash reports
end in the same frames of `cumotion.dll`. Both put other CUDA work into Kit's process. They were not
run again.

## The fix: cuMotion plans in a process of its own

- `cumotion_planner.py` is the planner as it was (`CuMotionPlanner`), moved unchanged. It runs only
  in the planning process.
- `planner_process.py` is the planning process. It is started with Isaac Sim's own Python (`-I`),
  given the paths where Kit found numpy and cuMotion. It listens on a local port for a random key,
  serves one connection, and ends with it.
- `planner.py` is Kit's side. `Planner` and `Plan` keep the interface they had (`add_cuboid`,
  `set_enabled`, `tool_pose`, `plan_goalset/pose/line/joints`; `duration`, `target_index`,
  `final()`, `at(t)`), so no caller changed. Every call goes to the planning process. One process
  serves a session: it starts with the first planner, starts again if it ended, and closes at exit.
  What it prints shows in Kit's log as `[planner]` lines.

**The same computation, so the same results.** Every number still comes from cuMotion's own code:
plans, forward kinematics, and also each `Plan.at(t)` the controller asks for while it follows a
plan, which is evaluated in the planning process. Numbers travel as Python floats, which keep all
64 bits. Each `Plan.at` is a round trip of a fraction of a millisecond, and a cycle asks about a
thousand. Planning took no longer: 34.6 and 34.8 s for 4 headless cycles before, 34.8 and 34.5 s
after.

**Cost and lifetime.** The planning process starts in a few seconds, once per session. Kit never
runs cuMotion now, in any mode. The physics stays on the CPU, so results stay comparable with the
batches before. When Kit ends, the planning process ends with it: at a normal exit Kit closes it,
and when Kit was killed it noticed the lost connection and ended within about 10 s.

## How it was checked

- **Unit tests.** `pick_place/tests/test_planner_process.py` (10 tests) runs Kit's side and the
  planning process with a fake planner: over a pipe, and as a started process. It checks that the
  numbers arrive unchanged, that errors and the process's end reach Kit, and that plans are let go.
  Run them with Isaac Sim's Python, which has numpy:
  `C:\issacsim-6-2\python.bat -m unittest discover -s pick_place/tests`.
- **The same plans, byte for byte.** Outside Kit, one scenario was planned with cuMotion in the same
  process twice, and once through the planning process. It covered forward kinematics, goalset, pose,
  line and joint plans, an obstacle switched off, and every point the controller follows. All three
  were identical byte for byte (cuMotion is deterministic).
- **The cell, headless.** `run_cell.py --headless` with the mug and the 2F-140.
  - From the simulation's pose, 4 cycles: run twice with the planner as it was in Kit, and twice
    with the planning process. All four runs were identical in every stored column but wall-clock
    times (3 placed, 1 with no reachable grasp).
  - With the wrist camera, 2 cycles: five runs with each planner, alternated. Every run ended in one
    of two outcomes, and each outcome was identical in every column whichever planner ran. The fork
    is one borderline plan in the first cycle, the lift of grasp 116. Sometimes cuMotion finds it;
    otherwise the grasp is ruled out and grasp 180 is taken. With the planner in Kit it was found in
    1 of 5 runs, with the planning process in 3 of 5. So cuMotion does not always repeat that plan
    from run to run, in Kit or out of it (within one process, repeated plans were identical). Five
    runs each are too few to say whether the planning process changes how often.
- **The web page.** `cell_app.py --web` with the page, the 2F-140 and the toy block, on 2026-11-20.
  - *Run 5 cycles*: no crash. Start poses 0, 1 and 2 ran with the page connected throughout,
    including 9 plans made mid-cycle; before the fix, the first of these crashed Kit every time.
  - The results were 0.4 mm, misplaced, 0.5, 1.6 and 0.3 mm. Batch run 1 in
    `results/pick_place.sqlite` (step 3) got the same at start poses 0 to 4, misplaced included;
    only how far the misplaced object ended up differed (104.7 against 90.2 mm).
  - Then the RG2 was chosen in the page: the cell was built again and its cycle ran. It dropped the
    object in transport, a grasp's outcome, not a crash.
  - The page lost the stream twice, both times while the plans at the start of a cycle held Kit up
    (below). Kit carried on, and a reload brought the page back with all its results.

## Afterwards: the page lost the stream while Kit waited for plans

With the crash fixed, one problem was left. Kit still waited for each plan without updating: the 6
plans at the start of a cycle took 5 to 6 s, in which Kit sent no frame. In the live run above, the
page lost the stream 7.9 s into the start of a cycle whose first grasp was ruled out, so more plans
were made. Which side gave up first, the page's WebRTC library or Kit's stream server, Kit's log does
not say.

Kit waited by design: a cycle advances one physics step per Kit update, and plans between two steps,
so the arm is still where a plan starts when the plan comes, and results do not depend on how long
planning takes. With cuMotion in Kit's process, Kit could not have drawn anything while it planned
anyway. Now its main thread only waits for an answer, so it can do something else meanwhile.

The fix, the same day:

- `planner.Pending` runs a planner call on a thread of its own; the thread only waits for the
  planning process's answer. Plans dropped by Kit's garbage collector on the main thread meanwhile
  go into a queue that needs no lock.
- `Cycle.plan` (`controller.py`) yields `PLANNING` until the answer comes, and `run_cycle`
  (`cell_session.py`) answers by pausing the simulation and letting Kit update: the viewport and the
  stream get frames, and no physics step is taken. The first step after the plan plays the
  simulation again. The check that every step advances the simulation by exactly one physics step
  still holds, so a pause that took effect late would stop the cycle.
- The first try updated Kit as fast as it could while it waited. The results were the same, but
  planning took half as long again (4 headless cycles: 54 s instead of 35): Kit's rendering slowed
  cuMotion down in the other process. Kit now updates at most every 0.1 s while it waits, about 10
  frames a second of a still scene, and planning took 35.3 s.

Checked the same way as the crash fix:

- **Headless, from the simulation's pose** (4 mug cycles): identical in every stored column but
  wall-clock times to the runs before this change.
- **Headless, with the wrist camera** (2 cycles, three runs): two took grasp 180 and one grasp 116,
  the two outcomes seen before, and each was identical in every column but wall-clock times to a run
  before this change with the same outcome. Planning took 2 to 5% longer than before, which is
  what drawing 10 frames a second costs.
- **The web page** (`cell_app.py --web`, 2F-140, toy block): 10 cycles, start poses 0 to 9, with
  the page connected throughout: one *Client connected* in Kit's log and no disconnect. The page
  counted the frames its decoder decoded: 9 to 18 a second in the first 8 s of each cycle, mostly
  planning, and 52 to 55 while the arm moved. Every result was the same as before: poses 0 to 4 as in
  the live run above, misplaced 104.7 mm included, and poses 5 to 9 as in batch run 1.
