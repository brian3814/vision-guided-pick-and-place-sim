# ROS 2 workspace (step 5)

The pick-and-place cell as a ROS 2 system. Isaac Sim plays the robot and the wrist camera and
publishes what a real robot would: the clock, joint states, the TF tree of the arm, and the
camera's depth image when the cycle looks. The object is found by a ROS 2 node, `object_locator`
(the step 4 perception, `pick_place/perception.py`, unchanged), which answers with the object's
pose; the cycle in Isaac Sim takes it from there. RViz2 shows it all.

Everything runs on Windows: Isaac Sim's ROS 2 bridge (its bundled ROS 2 Jazzy libraries) and
ROS 2 Jazzy for Windows talk Fast DDS on the same PC. No WSL is needed.

```
 Isaac Sim (Windows, bundled Jazzy)                      ROS 2 Jazzy for Windows (C:\pixi_ws)
 ┌──────────────────────────────┐   /clock /joint_states /tf   ┌──────────────────────────────┐
 │ UR10 + 2F-140, the cell      │ ───────────────────────────▶ │ static TF: wrist_3_link ->   │
 │ (OmniGraph ROS 2 bridge)     │                              │   wrist_camera_optical       │
 │                              │   /wrist_camera/depth ...    │   (hand-eye calibration)     │
 │ cycle: look ───────────────────────────────────────────────▶ │ object_locator               │
 │        wait  ◀─────────────────────────────────────────────── │   /object_pose, .../info     │
 │        grasp, place          │                              │ RViz2                        │
 └──────────────────────────────┘                              └──────────────────────────────┘
```

## Running it

Two terminals, from the project folder:

```bat
:: 1. the ROS 2 side: camera mount TF, object_locator, RViz2 (rviz:=false to skip it)
ros2_ws\ros2env.bat ros2 launch pick_place_ros locator.launch.py

:: 2. the cell, with Fast DDS (Isaac Sim's default on Windows is Zenoh)
set RMW_IMPLEMENTATION=rmw_fastrtps_cpp
set OMNI_KIT_ACCEPT_EULA=YES
C:\issacsim-6-2\python.bat pick_place\run_cell.py --object mug_A07RDPW843 --perception ros
```

To choose objects and run cycles from a window instead (the locator keeps running and follows the
object): `C:\issacsim-6-2\python.bat pick_place\cell_app.py --perception ros`, with the same two `set`
lines (see `pick_place/README.md`, "The interactive window").

Several objects headless: `set RMW_IMPLEMENTATION=rmw_fastrtps_cpp` then
`.venv\Scripts\python.exe pick_place\run_batch.py --objects ... --cycles 10 --perception ros`; one
locator serves them all (each run names its object on `/cell/object`).

Look around while it runs: `ros2_ws\ros2env.bat ros2 topic list`, `... ros2 topic echo /object_pose`,
`... ros2 topic hz /tf`. `ros2_ws\ros2env.bat` with no command opens a shell with ROS 2 and this
workspace sourced. Stop the ROS 2 side with
`powershell -ExecutionPolicy Bypass -File ros2_ws\stop_locator.ps1`.

If the launch fails with `ImportError: DLL load failed while importing _rclpy_pybind11`, the
terminal carries an old environment (seen in an IDE's terminal that was open from before): open a new
terminal and run it again.

## Topics and frames

| Topic | Type | From |
|---|---|---|
| `/clock` | rosgraph_msgs/Clock | Isaac Sim, every physics step (1/60 s) |
| `/joint_states` | sensor_msgs/JointState | Isaac Sim, every step: arm and gripper |
| `/tf` | tf2_msgs/TFMessage | Isaac Sim, every step: `world` -> `base_link` -> `shoulder_link` -> ... -> `wrist_3_link` -> the gripper's links (19 frames; the gripper's base is `World_UR10_Gripper_Robotiq_2F_140_base_link`, renamed because `base_link` was taken by the arm) |
| `/tf_static` | tf2_msgs/TFMessage | launch file: `wrist_3_link` -> `wrist_camera_optical`, from `results/perception/handeye.json` |
| `/cell/object` | std_msgs/String (latched) | Isaac Sim: which object to look for |
| `/wrist_camera/depth` | sensor_msgs/Image, 32FC1 metres, NaN = no depth | Isaac Sim, once per cycle at the look pose (simulated stereo depth) |
| `/wrist_camera/color`, `/wrist_camera/camera_info` | sensor_msgs/Image rgb8, CameraInfo | Isaac Sim, with the depth |
| `/object_pose` | geometry_msgs/PoseStamped (world, the image's stamp) | object_locator, when found |
| `/object_pose/info` | std_msgs/String, JSON | object_locator, one per image: found, fitness, points, detail, compute time |
| `/object_pose/model`, `/wrist_camera/object_points` | sensor_msgs/PointCloud2 | object_locator: the model at the estimate, the points taken as the object (for RViz2) |
| `/debug/object_true_pose` | geometry_msgs/PoseStamped | Isaac Sim: the simulation's truth, for comparison in RViz2 only |

## Setup (done once, 2026-11-10)

- **pixi** (the package manager ROS 2 for Windows uses) in `%USERPROFILE%\.pixi\bin`, from its
  GitHub release zip; it is not on the PATH, `ros2env.bat` calls it by its full path.
- **ROS 2 Jazzy for Windows** in `C:\pixi_ws`, per the official binary install: `pixi.toml` from
  `ros2/ros2` (branch `jazzy`), `pixi install`, and `ros2-jazzy-20260128-windows-release-amd64.zip`
  unpacked to `C:\pixi_ws\ros2-windows`. It includes the `ros2` CLI, colcon and RViz2. `scipy<1.15`
  was added to the pixi environment for the perception (`pixi add`).
- **This workspace**: `cd ros2_ws` then `ros2env.bat colcon build --base-paths src` (a pure Python
  package; no C++ compiler needed). Rebuild after changing `src/`.

## What it took

1. **Fast DDS on the Isaac Sim side.** Isaac Sim's `setup_ros_env.bat` makes Zenoh the middleware
   when `RMW_IMPLEMENTATION` is unset, and there is no Zenoh router in either install.
   `run_cell.py --perception ros` refuses to start without `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.
2. **Point the TF publisher at the articulation root.** Targeting the robot's prim `/World/UR10`
   published one frame, `world -> UR10`; the links appear only when it targets the prim with the
   articulation root, `/World/UR10/root_joint`.
3. **A TF buffer that survives the clock restarting.** The cell stops and replays the simulation
   every cycle, so `/clock` and the TF stamps start again from zero. A standard buffer would discard
   the new transforms as older than the last cycle's; the locator keeps its own buffer and clears it
   when time jumps back (the static camera mount is put back).
4. **Stop the node's Python, not just its launcher.** On Windows `object_locator.exe` is a small
   launcher that starts `python.exe`; stopping the launcher left old nodes running, and they answered
   first, with old code. `stop_locator.ps1` stops all of them.
5. **Activate from `C:\pixi_ws`.** The ROS 2 pixi environment sets Qt's plugin path from the current
   directory; `ros2env.bat` activates it there and returns.
6. **No WSL.** WSL's ROS 2 (Lyrical) could not see Windows over WSL's default NAT network, and would
   have meant a different ROS 2 release from Isaac Sim's Jazzy. ROS 2 for Windows avoids both. MoveIt 2
   has no Windows build, so it will need Linux (WSL) when it comes.

## Not done yet

- **MoveIt 2** (next): plan the motions in ROS 2 instead of with cuMotion inside Isaac Sim, with the
  UR10 driven through ros2_control. Linux only, so in WSL (mirrored networking) or Docker.
- The robot's model is not shown in RViz2 (no URDF on the ROS 2 side yet); TF axes stand in for it.
