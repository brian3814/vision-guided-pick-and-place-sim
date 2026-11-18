"""Hand-eye calibration of the wrist camera (eye-in-hand): where the camera sits on the gripper,
measured instead of taken from the drawing (camera_model.mount()). Each gripper carries the camera at
its own place, so each has its own calibration (as a real cell needs one after a tool change).

A ChArUco board lies on the table. The arm shows it to the camera from many viewpoints; at each,
OpenCV finds the board's corners in the RGB image and its pose relative to the camera (solvePnP),
and the arm's joints give the gripper's pose (forward kinematics). cv2.calibrateHandEye then
solves AX = XB for X, the camera's pose in the gripper frame. Of its methods, the one whose X puts
the board at the most consistent place in the world over all views is kept: a check a real cell
can make too, as it needs no ground truth. The comparison with the mount as built is reported
separately, as what only the simulation can tell.

    C:/issacsim-6-2/python.bat pick_place/handeye.py
    C:/issacsim-6-2/python.bat pick_place/handeye.py --gripper onrobot_rg2

Writes the gripper's calibration (camera_model.handeye_file(): results/perception/handeye.json for
the 2F-140, handeye_<gripper>.json for the others; read by run_cell.py and cell_app.py with the
camera) and a picture of one detection to results/perception/handeye_view.png.
"""

import argparse
import asyncio
import json
import pathlib
import sys

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--views", type=int, default=40, help="viewpoints to try (unreachable ones are skipped)")
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--gripper", default="robotiq_2f_140")
parser.add_argument("--out", help="default: camera_model.handeye_file(gripper)")
args, _ = parser.parse_known_args()

app = SimulationApp({"headless": True})

from isaacsim.core.experimental.utils.app import enable_extension  # noqa: E402

enable_extension("isaacsim.robot_motion.cumotion")
enable_extension("isaacsim.sensors.experimental.rtx")

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import camera_model as cm  # noqa: E402
import cell  # noqa: E402
import cv2  # noqa: E402
import gm_config  # noqa: E402
import numpy as np  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import perception  # noqa: E402
import planner  # noqa: E402
import wrist_camera  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdShade  # noqa: E402

from isaacsim.core.experimental.prims import Articulation  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402

# The board: 5 x 4 squares of 50 mm with 38 mm ArUco markers (4x4 bits), 250 x 200 mm, printed
# flat on the table between the pick and place spots. Markers this big still have about 5 pixels
# per bit at 0.38 m in the 640 x 400 image; with 30 mm markers most views found too few corners.
SQUARES, SQUARE_M, MARKER_M = (5, 4), 0.050, 0.038
DICTIONARY = cv2.aruco.DICT_4X4_50
BOARD_CENTRE = (0.62, 0.0)
BOARD_PX_PER_M = 10_000
BOARD = "/World/CalibrationBoard"
WAIT_STEPS = 20
METHODS = {"Tsai": cv2.CALIB_HAND_EYE_TSAI, "Park": cv2.CALIB_HAND_EYE_PARK, "Horaud": cv2.CALIB_HAND_EYE_HORAUD,
           "Andreff": cv2.CALIB_HAND_EYE_ANDREFF, "Daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS}


def log(msg: str) -> None:
    print(f"[handeye] {msg}", flush=True)


async def steps(n: int) -> None:
    for _ in range(n):
        await omni.kit.app.get_app().next_update_async()


def make_board() -> cv2.aruco.CharucoBoard:
    return cv2.aruco.CharucoBoard(SQUARES, SQUARE_M, MARKER_M, cv2.aruco.getPredefinedDictionary(DICTIONARY))


def board_pose() -> np.ndarray:
    """The board frame in the world, as OpenCV defines it: origin at the image's top-left corner,
    x to the right, y down the image, z into the board (here: into the table)."""
    w, h = SQUARES[0] * SQUARE_M, SQUARES[1] * SQUARE_M
    T = np.eye(4)
    T[:3, :3] = np.column_stack([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])
    T[:3, 3] = [BOARD_CENTRE[0] - w / 2, BOARD_CENTRE[1] + h / 2, 0.0005]
    return T


def add_board(stage, image_path: pathlib.Path) -> None:
    """A textured quad on the table showing the board image."""
    w, h = SQUARES[0] * SQUARE_M, SQUARES[1] * SQUARE_M
    T = board_pose()
    corners = perception.transform(T, np.array([[0, 0, 0], [w, 0, 0], [w, h, 0], [0, h, 0]], dtype=float))
    mesh = UsdGeom.Mesh.Define(stage, BOARD)
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in corners])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 3, 2, 1])  # counter-clockwise seen from above
    mesh.CreateDoubleSidedAttr(True)
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set([Gf.Vec2f(0, 1), Gf.Vec2f(1, 1), Gf.Vec2f(1, 0), Gf.Vec2f(0, 0)])  # image top-left at the board origin

    material = UsdShade.Material.Define(stage, f"{BOARD}_Material")
    shader = UsdShade.Shader.Define(stage, f"{BOARD}_Material/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    reader = UsdShade.Shader.Define(stage, f"{BOARD}_Material/st")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    texture = UsdShade.Shader.Define(stage, f"{BOARD}_Material/Texture")
    texture.CreateIdAttr("UsdUVTexture")
    texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(image_path))
    texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(texture.ConnectableAPI(), "rgb")
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)


def viewpoints(n: int, rng: np.random.Generator) -> list[np.ndarray]:
    """Camera poses (USD frame) looking at the board's centre from 0.25 to 0.38 m, tilted up to 30
    degrees from vertical in any direction, and rolled up to 45 degrees about the line of sight:
    rotations about different axes, which AX = XB needs."""
    target = np.array([*BOARD_CENTRE, 0.0])
    out = []
    for _ in range(n):
        d, tilt, azimuth, roll = rng.uniform(0.25, 0.38), np.radians(rng.uniform(5, 30)), rng.uniform(0, 2 * np.pi), np.radians(rng.uniform(-45, 45))
        z = np.array([np.sin(tilt) * np.cos(azimuth), np.sin(tilt) * np.sin(azimuth), np.cos(tilt)])  # back from the board
        x = np.cross([0.0, 0.0, 1.0], z) if tilt > 1e-3 else np.array([1.0, 0, 0])
        x /= np.linalg.norm(x)
        y = np.cross(z, x)
        c, s = np.cos(roll), np.sin(roll)
        x, y = c * x + s * y, -s * x + c * y
        T = np.eye(4)
        T[:3, :3] = np.column_stack([x, y, z])
        T[:3, 3] = target + d * z
        out.append(T)
    return out


def rt(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return T[:3, :3], T[:3, 3].reshape(3, 1)


def to_pose(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).reshape(3)
    return T


async def main() -> None:
    gripper = args.gripper
    out = gm_config.ROOT / (args.out or cm.handeye_file(gripper))
    out.parent.mkdir(parents=True, exist_ok=True)
    board = make_board()
    image_path = out.parent / "charuco_board.png"
    w_px, h_px = int(SQUARES[0] * SQUARE_M * BOARD_PX_PER_M), int(SQUARES[1] * SQUARE_M * BOARD_PX_PER_M)
    cv2.imwrite(str(image_path), board.generateImage((w_px, h_px), marginSize=0, borderBits=1))

    SimulationManager.switch_physics_engine("physx")
    g = gm_config.load_grippers()[gripper]
    obj = "toy_block_6MPL7NVFVU"  # the cell needs an object; it stays at the pick spot, out of the way
    rest = await cell.rest_mount(gripper, g, obj)
    c = cell.Cell(gripper, g, obj, rest * cell.shift(*cell.PICK_XY))
    add_board(c.stage, image_path)
    cam = wrist_camera.WristCamera(c.stage, gripper)
    pl = planner.Planner([(np.array(ct), r) for ct, r in c.tool_spheres()] + cm.tool_spheres(gripper))
    pl.add_cuboid("table", size=[cell.TABLE_X[1] - cell.TABLE_X[0], cell.TABLE_Y[1] - cell.TABLE_Y[0], cell.TABLE_THICKNESS],
                  centre=[sum(cell.TABLE_X) / 2, sum(cell.TABLE_Y) / 2, -cell.TABLE_THICKNESS / 2])
    SimulationManager.set_device("cpu")
    robot = Articulation(cell.ROBOT)
    arm_idx = [robot.dof_names.index(j) for j in cell.ARM_JOINTS]
    intr = cm.intrinsics()
    # OpenCV puts pixel centres at integer coordinates; camera_model's principal point is in
    # continuous coordinates (pixel u spans [u, u + 1)), so shift it by half a pixel.
    K = np.array([[intr.fx, 0, intr.cx - 0.5], [0, intr.fy, intr.cy - 0.5], [0, 0, 1]])
    marker_params = cv2.aruco.DetectorParameters()
    marker_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.CharucoDetector(board, cv2.aruco.CharucoParameters(), marker_params)

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    await steps(5)
    q = np.array(cell.HOME_Q)
    samples = []  # (gripper pose in base, board pose in camera, reprojection rms px)
    saved_view = False
    for k, view in enumerate(viewpoints(args.views, np.random.default_rng(args.seed))):
        # Aiming uses the mount as built, as an operator jogging the arm would aim roughly by eye.
        p = pl.plan_pose(q, view @ np.linalg.inv(cm.mount(gripper)))
        if p is None:
            log(f"view {k}: unreachable, skipped")
            continue
        q = p.final()
        qf = np.asarray(q, dtype=np.float32).reshape(1, -1)
        robot.set_dof_positions(qf, dof_indices=arm_idx)
        robot.set_dof_position_targets(qf, dof_indices=arm_idx)
        robot.set_dof_velocities(np.zeros_like(qf), dof_indices=arm_idx)
        robot.set_dof_velocity_targets(np.zeros_like(qf), dof_indices=arm_idx)
        await steps(WAIT_STEPS)
        frame = cam.read()
        if frame is None:
            log(f"view {k}: no image")
            continue
        rgb = frame[0]
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        corners, ids, marker_corners, marker_ids = detector.detectBoard(gray)
        if ids is None or len(ids) < 8:
            log(f"view {k}: {0 if ids is None else len(ids)} corners found, skipped")
            continue
        obj_pts, img_pts = board.matchImagePoints(corners, ids)
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, None, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            log(f"view {k}: solvePnP failed")
            continue
        rvec, tvec = cv2.solvePnPRefineLM(obj_pts, img_pts, K, None, rvec, tvec)
        proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, None)
        rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img_pts.reshape(-1, 2)) ** 2, axis=1))))
        q_now = np.asarray(robot.get_dof_positions().numpy(), dtype=float).reshape(-1)[arm_idx]
        samples.append((pl.tool_pose(q_now), to_pose(cv2.Rodrigues(rvec)[0], tvec), rms))
        log(f"view {k}: {len(ids)} corners, reprojection {rms:.3f} px")
        if not saved_view:
            vis = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            cv2.aruco.drawDetectedCornersCharuco(vis, corners, ids, (0, 0, 255))
            cv2.drawFrameAxes(vis, K, None, rvec, tvec, 0.06)
            cv2.imwrite(str(out.parent / "handeye_view.png"), vis)
            saved_view = True
    timeline.stop()
    await steps(1)
    if len(samples) < 5:
        raise RuntimeError(f"only {len(samples)} usable views")

    R_g2b, t_g2b = zip(*[rt(s[0]) for s in samples])
    R_t2c, t_t2c = zip(*[rt(s[1]) for s in samples])
    truth = cm.mount(gripper) @ cm.USD_TO_CV  # the OpenCV camera frame in the gripper frame, as built
    true_board = board_pose()
    results = {}
    for name, method in METHODS.items():
        R, t = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=method)
        X = to_pose(R, t)
        boards = [s[0] @ X @ s[1] for s in samples]  # the board in the world, per view
        mean_t = np.mean([b[:3, 3] for b in boards], axis=0)
        spread_mm = 1000 * float(np.sqrt(np.mean([np.sum((b[:3, 3] - mean_t) ** 2) for b in boards])))
        spread_deg = float(np.sqrt(np.mean([perception.angle_deg(b, boards[0]) ** 2 for b in boards[1:]])))
        results[name] = {
            "X": X, "board_spread_mm": spread_mm, "board_spread_deg": spread_deg,
            "error_mm": 1000 * float(np.linalg.norm(X[:3, 3] - truth[:3, 3])),
            "error_deg": perception.angle_deg(X, truth),
            "board_error_mm": 1000 * float(np.linalg.norm(mean_t - true_board[:3, 3])),
        }
        log(f"{name}: board spread {spread_mm:.2f} mm / {spread_deg:.3f} deg over the views; "
            f"vs the mount as built {results[name]['error_mm']:.2f} mm, {results[name]['error_deg']:.3f} deg")
    best = min(results, key=lambda n: results[n]["board_spread_mm"])
    X = results[best]["X"]
    record = {
        "gripper": gripper, "method": best, "views": len(samples), "views_tried": args.views,
        "reprojection_px_median": float(np.median([s[2] for s in samples])),
        "mount_usd": (X @ cm.USD_TO_CV).tolist(),  # the USD camera in the gripper frame, as camera_model.mount(gripper)
        "camera_cv_in_gripper": X.tolist(),
        **{k: v for k, v in results[best].items() if k != "X"},
        "all_methods": {n: {k: v for k, v in r.items() if k != "X"} for n, r in results.items()},
        "board": {"squares": SQUARES, "square_m": SQUARE_M, "marker_m": MARKER_M, "dictionary": "DICT_4X4_50",
                  "centre": BOARD_CENTRE},
    }
    out.write_text(json.dumps(record, indent=1), encoding="utf-8")
    log(f"kept {best}: {len(samples)} views; camera {record['error_mm']:.2f} mm, {record['error_deg']:.3f} deg "
        f"from the mount as built; wrote {out}")


task = asyncio.ensure_future(main())
while not task.done():
    app.update()
if task.exception():
    raise task.exception()
app.close()
