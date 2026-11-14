"""Re-run one stored grasp trial and save pictures of it, to check the numbers by eye.

Reads the grasp pose from the database, rebuilds the same scene, runs the trial with the same
physics stepping as the batch run, and saves an image after each phase (placed, closed,
lifted) from two cameras. Prints the outcome next to the stored one.

    C:/issacsim-6-2/python.bat grasp_matrix/render_grasp.py --run-id 1 --grasp 48
"""

import argparse
import asyncio
import pathlib
import sqlite3
import sys

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--db", default="results/grasp_matrix.sqlite")
parser.add_argument("--run-id", type=int, required=True)
parser.add_argument("--grasp", type=int, required=True, help="grasp_index within the run")
parser.add_argument("--condition", default="baseline")
parser.add_argument("--out", default="results/renders")
args, _ = parser.parse_known_args()

app = SimulationApp({"headless": True})

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import gm_core  # noqa: E402
import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402

from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402

RESOLUTION = (640, 480)
HFOV_DEG = 50.0


def log(msg: str) -> None:
    print(f"[render] {msg}", flush=True)


def look_at_matrix(eye: Gf.Vec3d, target: Gf.Vec3d) -> Gf.Matrix4d:
    """Camera-to-world transform for a USD camera (looks down -Z, +Y up) at eye aimed at target."""
    forward = (target - eye).GetNormalized()
    right = Gf.Cross(forward, Gf.Vec3d(0, 0, 1)).GetNormalized()
    up = Gf.Cross(right, forward)
    rot = Gf.Matrix3d(*right, *up, *(-forward))
    m = Gf.Matrix4d()
    m.SetRotate(rot)
    m.SetTranslateOnly(eye)
    return m


def add_camera(stage, path: str, eye: Gf.Vec3d, target: Gf.Vec3d):
    cam = UsdGeom.Camera.Define(stage, path)
    cam.AddTransformOp().Set(look_at_matrix(eye, target))
    aperture = cam.GetHorizontalApertureAttr().Get()
    # Focal length is set on the attribute (same units as the aperture); near clip of 1 cm
    # instead of USD's default 1 scene unit (1 m here).
    cam.GetFocalLengthAttr().Set(float(aperture / (2 * np.tan(np.radians(HFOV_DEG) / 2))))
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100.0))
    rp = rep.create.render_product(path, RESOLUTION)
    annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    annotator.attach(rp)
    return annotator


async def main() -> None:
    conn = sqlite3.connect(gm_core.ROOT / args.db)
    run = conn.execute("SELECT gripper, object FROM runs WHERE run_id=?", (args.run_id,)).fetchone()
    pose = conn.execute(
        "SELECT pos_x, pos_y, pos_z, qw, qx, qy, qz, width_m FROM grasps WHERE run_id=? AND grasp_index=?",
        (args.run_id, args.grasp),
    ).fetchone()
    stored = conn.execute(
        "SELECT outcome FROM trials WHERE run_id=? AND grasp_index=? AND condition=?",
        (args.run_id, args.grasp, args.condition),
    ).fetchone()
    gripper_name, object_name = run
    log(f"{gripper_name} x {object_name}, grasp {args.grasp}, stored outcome: {stored[0] if stored else 'none'}")

    SimulationManager.switch_physics_engine("physx")
    scene = gm_core.GraspScene(gripper_name, gm_core.load_grippers()[gripper_name], object_name)
    stage = scene.stage
    from pxr import UsdLux

    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(900.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(2500.0)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(-45, 10, 35))

    step = gm_core.physics_steps
    await scene.settle_object(step)
    centre = Gf.Vec3d(*scene.object_position_usd()) + Gf.Vec3d(0, 0, 0.04)
    cameras = {
        "front": add_camera(stage, "/World/CamFront", centre + Gf.Vec3d(0.42, -0.42, 0.30), centre),
        "side": add_camera(stage, "/World/CamSide", centre + Gf.Vec3d(-0.45, -0.25, 0.12), centre),
    }

    out = gm_core.ROOT / args.out / f"run{args.run_id}_grasp{args.grasp}_{args.condition}"
    out.mkdir(parents=True, exist_ok=True)

    table = UsdGeom.Imageable(stage.GetPrimAtPath(gm_core.TABLE))

    async def snapshot(phase: str) -> None:
        # The lift test only switches the table's collision off; hide it too so the picture shows that.
        if phase == "lifted":
            table.MakeInvisible()
        await rep.orchestrator.step_async(rt_subframes=8, pause_timeline=False)
        table.MakeVisible()
        for name, annotator in cameras.items():
            data = annotator.get_data()
            if data is not None and data.size:
                Image.fromarray(np.asarray(data)[..., :3].astype(np.uint8)).save(out / f"{phase}_{name}.png")
        log(f"saved {phase}")

    location = Gf.Vec3d(*pose[:3])
    orientation = Gf.Quatd(pose[3], *pose[4:7])
    result = await scene.run_trial(location, orientation, pose[7], args.condition, step, on_phase=snapshot)
    log(f"re-run outcome: {result['outcome']} ({result}); images in {out}")


task = asyncio.ensure_future(main())
while not task.done():
    app.update()
if task.exception():
    raise task.exception()
app.close()
