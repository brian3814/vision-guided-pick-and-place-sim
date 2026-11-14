# vision-guided-pick-and-place-sim

A UR10 picks scanned objects using grasps ranked in a gripper benchmark, locates them with a wrist
depth camera, and runs the loop over ROS 2, all in Isaac Sim.

Built one step at a time, each step measured before the next (step 1, setting up and checking Isaac
Sim on this PC, is not part of the repository):

| Step | What | Result | Details |
|---|---|---|---|
| 2. Grasp matrix | 6 parallel grippers x 20 scanned SimReady objects (120 pairs): which grasps hold under every test condition | Robotiq 2F-140 best (46% of tested grasps robust) | `grasp_matrix/README.md`, `results/summary.md` |
| 3. Pick-and-place cell | UR10 with the 2F-140, grasps from the matrix, motions planned with cuMotion | 43/50 placed | `pick_place/README.md` |
| 4. Eye-in-hand | Wrist depth camera; object pose from depth (RANSAC + ICP against the object's model); ChArUco hand-eye calibration with OpenCV | 47/50 placed; 23/50 without the camera | `pick_place/README.md` (Step 4) |
| 5. ROS 2 | The perception as a ROS 2 node (Jazzy on Windows), in a closed loop with Isaac Sim over its ROS 2 bridge | 47/50 placed | `ros2_ws/README.md` |

All results are simulated: read them as comparisons under stated conditions, not real-world success
rates. Every table in the READMEs comes from the SQLite databases in `results/` (summaries:
`results/summary.md`, `results/pick_place_summary.md`, `results/perception_summary.md`).

Next: MoveIt 2, planning the motions in ROS 2 instead of inside Isaac Sim.

## Layout

| Folder | Contents |
|---|---|
| `grasp_matrix/` | The gripper x object benchmark: trial protocol, runner, report |
| `pick_place/` | The cell: planner, cycle controller, wrist camera, perception, hand-eye calibration, interactive window |
| `ros2_ws/` | ROS 2 workspace: the `object_locator` node and its launch file |
| `tools/` | Gripper calibration and geometry |
| `data/` | Object catalog, gripper calibration |
| `results/` | Result databases and summaries, the hand-eye calibration, pictures of one cycle |

## Requirements

- Isaac Sim 6.2 (standalone, Windows). Scripts that need it run with its `python.bat`, with
  `OMNI_KIT_ACCEPT_EULA=YES` set.
- The assets are not in this repository. Download them into `assets/`: the objects and grippers from
  Hugging Face (`grasp_matrix/README.md`), the UR10 from NVIDIA's asset server (`pick_place/README.md`).
- `.venv`: Python 3.12 with `usd-core` (made with uv), for the batch runners and reports.
- Step 5: ROS 2 Jazzy for Windows (`ros2_ws/README.md`).
- Developed on an RTX 5070 (12 GB) with 32 GB of RAM, below Isaac Sim's published minimum; the
  READMEs note where that shaped the setup.
