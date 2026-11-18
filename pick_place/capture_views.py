"""Capture the wrist camera's view of each cycle's start pose, to tune and measure the perception
offline (perception_eval.py). The object is placed as run_cell.py places it (same seeds), the arm
is put at the look pose, and one RGB + depth frame is saved with the camera pose (from the arm's
joints) and the object's true pose. The object's model points are saved too.

    C:/issacsim-6-2/python.bat pick_place/capture_views.py --object mug_A07RDPW843 --cycles 10

Output (results/perception/): captures/<object>_<cycle>.npz and .png, models/<object>.npz
"""

import argparse
import asyncio
import pathlib
import sys

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--gripper", default="robotiq_2f_140")
parser.add_argument("--object", required=True)
parser.add_argument("--cycles", type=int, default=10)
parser.add_argument("--start-cycle", type=int, default=0)
parser.add_argument("--seed", type=int, default=1)
parser.add_argument("--max-yaw-deg", type=float, default=90.0)
parser.add_argument("--jitter-m", type=float, default=0.03)
parser.add_argument("--out", default="results/perception")
args, _ = parser.parse_known_args()

app = SimulationApp({"headless": True})

from isaacsim.core.experimental.utils.app import enable_extension  # noqa: E402

enable_extension("isaacsim.robot_motion.cumotion")
enable_extension("isaacsim.sensors.experimental.rtx")

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import camera_model  # noqa: E402
import cell  # noqa: E402
import controller  # noqa: E402
import gm_config  # noqa: E402
import numpy as np  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import planner  # noqa: E402
import wrist_camera  # noqa: E402
from PIL import Image  # noqa: E402

from isaacsim.core.experimental.prims import Articulation  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402

SETTLE_STEPS = 30  # as a cycle settles before it moves
LOOK_WAIT_STEPS = 30


def log(msg: str) -> None:
    print(f"[capture] {msg}", flush=True)


async def steps(n: int) -> None:
    for _ in range(n):
        await omni.kit.app.get_app().next_update_async()


async def main() -> None:
    out = gm_config.ROOT / args.out
    (out / "captures").mkdir(parents=True, exist_ok=True)
    (out / "models").mkdir(parents=True, exist_ok=True)
    SimulationManager.switch_physics_engine("physx")
    g = gm_config.load_grippers()[args.gripper]
    rest = await cell.rest_mount(args.gripper, g, args.object)
    c = cell.Cell(args.gripper, g, args.object, rest * cell.shift(*cell.PICK_XY))
    cam = wrist_camera.WristCamera(c.stage, args.gripper)
    model, used = wrist_camera.object_model(c.object_body)
    log(f"model: {len(model.points)} points from {used}")
    np.savez_compressed(out / "models" / f"{args.object}.npz", points=model.points, normals=model.normals)

    pl = planner.Planner([(np.array(ct), r) for ct, r in c.tool_spheres()] + camera_model.tool_spheres(args.gripper))
    pl.add_cuboid("table", size=[cell.TABLE_X[1] - cell.TABLE_X[0], cell.TABLE_Y[1] - cell.TABLE_Y[0], cell.TABLE_THICKNESS],
                  centre=[sum(cell.TABLE_X) / 2, sum(cell.TABLE_Y) / 2, -cell.TABLE_THICKNESS / 2])
    look = camera_model.look_tool_pose([*cell.PICK_XY, 0.04], args.gripper)
    p = pl.plan_pose(np.array(cell.HOME_Q), look)
    if p is None:
        raise RuntimeError("no motion to the look pose")
    q_look = p.final()
    SimulationManager.set_device("cpu")
    robot = Articulation(cell.ROBOT)
    arm_idx = [robot.dof_names.index(j) for j in cell.ARM_JOINTS]
    intr = camera_model.intrinsics()
    timeline = omni.timeline.get_timeline_interface()
    rest_np = wrist_camera.to_np(rest)

    for i in range(args.start_cycle, args.start_cycle + args.cycles):
        yaw, pick, move = cell.start_pose(args.seed, i, args.max_yaw_deg, args.jitter_m)
        c.set_object_mount(wrist_camera.to_gf(move @ rest_np))
        timeline.play()
        await steps(1 + SETTLE_STEPS)
        q = np.asarray(q_look, dtype=np.float32).reshape(1, -1)
        robot.set_dof_positions(q, dof_indices=arm_idx)
        robot.set_dof_position_targets(q, dof_indices=arm_idx)
        robot.set_dof_velocities(np.zeros_like(q), dof_indices=arm_idx)
        robot.set_dof_velocity_targets(np.zeros_like(q), dof_indices=arm_idx)
        await steps(LOOK_WAIT_STEPS)
        frame = cam.read()
        while frame is None:
            await steps(1)
            frame = cam.read()
        rgb, depth = frame
        q_now = np.asarray(robot.get_dof_positions().numpy(), dtype=float).reshape(-1)[arm_idx]
        T_wc = camera_model.camera_pose(pl.tool_pose(q_now), camera_model.mount(args.gripper))
        obj = controller.body_pose(str(c.object_body.GetPath()))
        name = f"{args.object}_{i}"
        np.savez_compressed(out / "captures" / f"{name}.npz", depth=depth.astype(np.float32), rgb=rgb, T_world_cam=T_wc,
                            obj_true=obj, q=q_now, gripper=args.gripper, yaw=yaw, pick=pick, seed=args.seed, cycle=i,
                            roi_centre=np.array(cell.PICK_XY),
                            intrinsics=np.array([intr.fx, intr.fy, intr.cx, intr.cy, intr.width, intr.height]))
        Image.fromarray(rgb).save(out / "captures" / f"{name}.png")
        log(f"{name}: yaw {yaw:+.0f} deg at ({pick[0]:.3f}, {pick[1]:.3f}); "
            f"{np.count_nonzero(np.isfinite(depth) & (depth > 0))} depth pixels")
        timeline.stop()
        await steps(1)
    log("done")


task = asyncio.ensure_future(main())
while not task.done():
    app.update()
if task.exception():
    raise task.exception()
app.close()
