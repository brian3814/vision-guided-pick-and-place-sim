"""The pick-and-place cell ready to run cycles, shared by run_cell.py (a batch of cycles on one
object) and cell_app.py (a window to choose the gripper and object and run cycles one at a time):
the stage, the planner, the robot, and with the camera or ROS 2 the parts that find the object. The
object can be swapped between cycles (load_object); the gripper too, by building the cell again
(build(gripper=...)), except with ROS 2.

Runs inside Isaac Sim; import it only after cell_args.start() (it enables the extensions needed).
"""

import datetime
import json
import pathlib
import sys

import cell  # also puts grasp_matrix/ on the path
import cell_db
import controller
import gm_config
import numpy as np
import omni.kit.app
import omni.timeline
import planner
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager

SETTINGS = {
    "version": 1, "physics_device": "cpu",
    "home_q": cell.HOME_Q, "pick_xy": cell.PICK_XY, "place_xy": cell.PLACE_XY,
    "pre_grasp_m": controller.PRE_GRASP_M, "lift_m": controller.LIFT_M, "place_clearance_m": controller.PLACE_CLEARANCE_M,
    "tool_tolerance_m": controller.TOOL_TOLERANCE_M, "place_tolerance_m": controller.PLACE_TOLERANCE_M,
    "max_tilt_deg": controller.MAX_TILT_DEG, "dt": gm_config.DT,
}
MODELS_DIR = gm_config.ROOT / "results/perception/models"  # object models for the ROS 2 object_locator


def log(msg: str) -> None:
    print(f"[cell] {msg}", flush=True)


def to_np(m: Gf.Matrix4d) -> np.ndarray:
    """Gf matrix (row vectors) -> numpy pose acting on column vectors."""
    return np.array(m, dtype=float).T


def to_gf(m: np.ndarray) -> Gf.Matrix4d:
    return Gf.Matrix4d(m.T.tolist())


async def next_update() -> None:
    await omni.kit.app.get_app().next_update_async()


async def run_cycle(cyc: controller.Cycle) -> None:
    """Advance the cycle once per physics step until it finishes. It runs from the main loop, one
    app update per physics step (checked against the timeline's clock), so planning happens
    between steps rather than inside PhysX's step callback."""
    timeline = omni.timeline.get_timeline_interface()
    last = timeline.get_current_time()
    for _ in cyc.run():
        await next_update()
        if not timeline.is_playing():
            raise RuntimeError("the simulation was stopped or paused while the cycle ran, not by the cycle "
                               "(the toolbar's Play/Stop?); the cycle was abandoned")
        now = timeline.get_current_time()
        if abs(now - last - gm_config.DT) > 1e-5:
            raise RuntimeError(f"an app update advanced the simulation by {now - last:.5f} s, not one step of {gm_config.DT:.5f} s")
        last = now


class CellSession:
    """The cell with one object at a time. `args` are cell_args.parser()'s options."""

    def __init__(self, args):
        self.args = args
        self.gripper: str = args.gripper
        self.vision = args.perception in ("camera", "ros")
        self.camera = None
        self.link = None  # ros_link.RosLink with --perception ros
        self.conn = None
        self.run_id: int | None = None
        self.settings_extra: dict = {}

    # --- building ----------------------------------------------------------------------------
    def handeye_path(self, gripper: str) -> pathlib.Path | None:
        """The hand-eye calibration for the camera on this gripper (--handeye); None: the mount as built."""
        import camera_model

        a = self.args
        if a.handeye == "none":
            return None
        if a.handeye != "auto" and gripper == a.gripper:
            return gm_config.ROOT / a.handeye
        return gm_config.ROOT / camera_model.handeye_file(gripper)

    def cannot_use(self, gripper: str) -> str | None:
        """Why the cell cannot be built with this gripper (with the camera: no hand-eye calibration
        for it), or None."""
        if not self.vision:
            return None
        path = self.handeye_path(gripper)
        if path is not None and not path.exists():
            return (f"no hand-eye calibration for {gripper} ({path.relative_to(gm_config.ROOT).as_posix()}); "
                    f"run handeye.py --gripper {gripper}")
        return None

    async def build(self, object_name: str, rest: Gf.Matrix4d | None = None, gripper: str | None = None) -> bool:
        """Build the cell with this object at the pick spot, and this gripper (default: the current
        one; at first --gripper). `rest` is the object's mount after it settles on the grasp
        matrix's table (cell.rest_mount(), computed here if not given). False, and nothing changed,
        if the object has no robust grasps for the gripper.

        Building again, with another gripper, replaces the stage (the gripper is part of the arm's
        articulation, and the planner's model of it changes): only while the simulation is stopped,
        after finish_run(), and not with ROS 2 (the locator's camera mount comes from its launch file)."""
        a = self.args
        gripper = gripper or self.gripper
        if self.link is not None:
            raise RuntimeError("with ROS 2 the gripper cannot change: the object_locator's camera mount comes from "
                               "its launch file (start again with --gripper, and the locator with that gripper's handeye)")
        problem = self.cannot_use(gripper)
        if problem:
            raise RuntimeError(problem)
        SimulationManager.switch_physics_engine("physx")
        g = gm_config.load_grippers()[gripper]
        if not self._load_grasps(object_name, gripper):
            return False
        if self.camera is not None:  # built before: its render product goes with the old stage
            self.camera.close()
            self.camera = None
        self.gripper, self.g = gripper, g
        if rest is None:
            rest = await cell.rest_mount(gripper, g, object_name)
        self.rest = rest
        self.cell = c = cell.Cell(gripper, g, object_name, rest * cell.shift(*cell.PICK_XY))
        spheres = [(np.array(centre), r) for centre, r in c.tool_spheres()]
        if self.vision:
            import camera_model
            import wrist_camera

            self.camera = wrist_camera.WristCamera(c.stage, gripper)
            self._load_model()
            path = self.handeye_path(gripper)
            if path is None:
                self.mount, handeye = camera_model.mount(gripper), {"method": "none (as built)"}
            else:
                handeye = json.loads(path.read_text(encoding="utf-8"))
                if handeye.get("gripper", "robotiq_2f_140") != gripper:  # the first calibration (2F-140) names none
                    raise RuntimeError(f"{path.name} is the hand-eye calibration of {handeye.get('gripper')}, not {gripper}")
                self.mount = np.array(handeye["mount_usd"])
                log(f"camera mount from hand-eye calibration ({handeye['method']}, {handeye['views']} views): "
                    f"{handeye['error_mm']:.2f} mm, {handeye['error_deg']:.3f} deg from the mount as built")
            self.noise = None if a.depth_noise == "perfect" else float(a.depth_noise)
            spheres += camera_model.tool_spheres(gripper)
            self.settings_extra = {"depth_noise": a.depth_noise,
                                   "handeye": {k: handeye.get(k) for k in ("method", "views", "error_mm", "error_deg")}}
        else:
            blind = a.perception == "blind"  # from the look pose and without slip correction, as with the camera
            self.settings_extra = {"slip_correction": not (a.no_slip_correction or blind)}
            if a.look_first or blind:
                import camera_model

                spheres += camera_model.tool_spheres(gripper)
                self.settings_extra["look_first"] = True
        self.planner = pl = planner.Planner(spheres)
        pl.add_cuboid("table", size=[cell.TABLE_X[1] - cell.TABLE_X[0], cell.TABLE_Y[1] - cell.TABLE_Y[0], cell.TABLE_THICKNESS],
                      centre=[sum(cell.TABLE_X) / 2, sum(cell.TABLE_Y) / 2, -cell.TABLE_THICKNESS / 2])
        if controller.DEBUG_STEPS:
            self._report_contacts()
        # CPU physics: with PhysX on the GPU, creating a second cuMotion optimizer crashed the process.
        SimulationManager.set_device("cpu")
        self.robot = Articulation(cell.ROBOT)
        if a.perception == "ros":
            import ros_link

            root = ros_link.build_graph(cell.ROBOT)  # clock, joint states, TF: published every physics step once playing
            log(f"ROS 2 bridge: /clock, /joint_states and /tf from {root}")
            self.link = ros_link.RosLink()
            self.link.publish_object(object_name)
            log("waiting for the object_locator node (ros2_ws\\ros2env.bat ros2 launch pick_place_ros locator.launch.py)")
            if not self.link.wait_for_locator():
                self.link.close()
                self.link = None
                raise RuntimeError("no object_locator subscribed to /wrist_camera/depth within 60 s")
            problem = self.link.check_mount(self.mount)
            if problem:
                self.link.close()
                self.link = None
                path = self.handeye_path(gripper)
                raise RuntimeError(problem + (f"; launch it with handeye:={path.relative_to(gm_config.ROOT).as_posix()}"
                                              if path else ""))
            log("object_locator is listening, with this gripper's camera mount")
        if not getattr(a, "headless", False):
            from isaacsim.core.rendering_manager import ViewportManager

            ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[1.9, -1.6, 1.2], target=[0.6, 0.0, 0.15])
        return True

    def load_object(self, object_name: str, rest: Gf.Matrix4d) -> bool:
        """Swap the object for another (only while the simulation is stopped). False, and nothing
        changed, if it has no robust grasps. Finish the current run first (finish_run())."""
        if not self._load_grasps(object_name, self.gripper):
            return False
        self.rest = rest
        self.cell.load_object(object_name, rest * cell.shift(*cell.PICK_XY))
        if self.vision:
            self._load_model()
        if controller.DEBUG_STEPS:
            self._report_contacts()
        if self.link is not None:
            self.link.publish_object(object_name)
        return True

    def _load_grasps(self, object_name: str, gripper: str) -> bool:
        grasp_run, grasps = cell_db.robust_grasps(str(gm_config.ROOT / self.args.matrix_db), gripper, object_name)
        if not grasps:
            log(f"{gripper} x {object_name}: no robust grasps in the grasp matrix (run {grasp_run}); nothing to do")
            return False
        log(f"{gripper} x {object_name}: {len(grasps)} robust grasps from grasp-matrix run {grasp_run}")
        self.object_name, self.grasp_run, self.grasps = object_name, grasp_run, grasps
        self.grasps_matrix = [dict(gr, pose=controller.make_pose(gr["position"], controller.quat_matrix(*gr["quat_wxyz"])))
                              for gr in grasps]
        return True

    def _load_model(self) -> None:
        """The object's model for the perception, and its body pose at rest in the grasp matrix's
        world (the cell has just put it at the pick spot). With ROS 2 the locator reads the model
        from results/perception/models; it is written there if missing."""
        import wrist_camera

        c = self.cell
        self.model, meshes = wrist_camera.object_model(c.object_body)
        log(f"object model: {len(self.model.points)} points from {meshes}")
        body = to_np(UsdGeom.Xformable(c.object_body).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
        self.rest_body = controller.translation([-cell.PICK_XY[0], -cell.PICK_XY[1], 0.0]) @ body
        path = MODELS_DIR / f"{self.object_name}.npz"
        if self.args.perception == "ros" and not path.exists():
            MODELS_DIR.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(path, points=self.model.points, normals=self.model.normals)
            log(f"object model written for the locator: {path.relative_to(gm_config.ROOT)}")

    def _report_contacts(self) -> None:
        """Report contacts of every body, for controller.contacts()."""
        for prim in Usd.PrimRange(self.cell.stage.GetPrimAtPath("/World")):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)

    # --- runs and cycles -------------------------------------------------------------------------
    def start_run(self, db: str, **extra) -> int:
        """Start a run of the current object in the results database; extra: more settings to record."""
        a = self.args
        if self.conn is None:
            self.conn = cell_db.connect(str(gm_config.ROOT / db))
        settings = dict(SETTINGS, max_yaw_deg=a.max_yaw_deg, jitter_m=a.jitter_m, seed=a.seed, speed=a.speed)
        if a.perception != "truth" or a.no_slip_correction or a.look_first:  # step 3's runs stored none of these
            settings.update(perception=a.perception, **self.settings_extra)
        settings.update(extra)
        version = (pathlib.Path(sys.executable).resolve().parents[2] / "VERSION")
        self.run_id = self.conn.execute(
            "INSERT INTO cell_runs (gripper, object, settings, grasp_run_id, robust_grasps, started, isaac_sim_version) "
            "VALUES (?, ?, ?, ?, ?, ?, ?)",
            (self.gripper, self.object_name, json.dumps(settings), self.grasp_run, len(self.grasps),
             datetime.datetime.now().isoformat(timespec="seconds"), version.read_text().strip() if version.exists() else "unknown"),
        ).lastrowid
        self.conn.commit()
        return self.run_id

    def finish_run(self) -> None:
        if self.run_id is not None:
            self.conn.execute("UPDATE cell_runs SET finished=? WHERE run_id=?",
                              (datetime.datetime.now().isoformat(timespec="seconds"), self.run_id))
            self.conn.commit()
            self.run_id = None

    def start_pose(self, i: int) -> tuple[float, np.ndarray, np.ndarray]:
        """Cycle i's start pose (yaw in degrees, pick point, matrix world -> cell), and put the object there."""
        a = self.args
        yaw, pick, move = cell.start_pose(a.seed, i, a.max_yaw_deg, a.jitter_m)
        self.cell.set_object_mount(to_gf(move @ to_np(self.rest)))
        return yaw, pick, move

    async def run_cycle(self, i: int, label: str, on_phase=None, on_log=None) -> dict:
        """Run cycle i (its start pose is seeded by i) and record it in the current run. label: how
        the cycle is named in the log, e.g. "3/10". on_log: also called with each line the cycle logs."""
        a = self.args
        yaw, pick, move = self.start_pose(i)
        object_start = to_np(UsdGeom.Xformable(self.cell.object_body).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
        cycle_grasps = [dict(gr, pose=move @ gr["pose"], matrix_pose=gr["pose"]) for gr in self.grasps_matrix]
        place_shift = np.array([cell.PLACE_XY[0] - pick[0], cell.PLACE_XY[1] - pick[1], 0.0])
        line = f"cycle {label}: object turned {yaw:+.0f} deg at ({pick[0]:.3f}, {pick[1]:.3f})"
        log(line)
        if on_log:
            on_log(line)

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        await next_update()
        vision = None
        if self.vision:
            vision = controller.Vision(self.camera, self.model, self.mount, self.rest_body, self.noise,
                                       np.random.default_rng([a.seed, i, 7]), link=self.link)
        assumed = None
        if a.perception == "blind":  # the object where it would be with no turn and no shift
            _, nominal_pick, nominal = cell.start_pose(a.seed, i, 0.0, 0.0)
            assumed = (nominal @ np.linalg.inv(move) @ object_start,
                       np.array([cell.PLACE_XY[0] - nominal_pick[0], cell.PLACE_XY[1] - nominal_pick[1], 0.0]))
        cyc = controller.Cycle(self.robot, self.cell, self.planner, cycle_grasps, object_start, place_shift, gm_config.DT,
                               a.speed, on_phase, vision=vision, slip_correction=not a.no_slip_correction,
                               look_first=a.look_first or a.perception == "blind", assumed=assumed)
        if on_log:
            cycle_log = cyc.log
            cyc.log = lambda msg: (cycle_log(msg), on_log(msg))
        try:
            await run_cycle(cyc)
        finally:
            timeline.stop()
            await next_update()

        r = cyc.result
        r.update(yaw_deg=round(yaw, 2), pick_x=round(float(pick[0]), 4), pick_y=round(float(pick[1]), 4),
                 motion_s=round(r.get("motion_s", 0.0), 2), plan_wall_s=round(r["plan_wall_s"], 2),
                 max_tool_error_m=round(r["max_tool_error_m"], 4))
        self.conn.execute(f"INSERT INTO cell_cycles (run_id, cycle, {', '.join(cell_db.CYCLE_COLUMNS)}) "
                          f"VALUES (?, ?, {', '.join('?' * len(cell_db.CYCLE_COLUMNS))})",
                          (self.run_id, i, *[r.get(k) for k in cell_db.CYCLE_COLUMNS]))
        self.conn.commit()
        return r

    def close(self) -> None:
        self.finish_run()
        if self.link is not None:
            self.link.close()
            self.link = None
