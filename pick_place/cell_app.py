"""The pick-and-place cell in a window, to run interactively: choose the gripper and the object, run
a cycle (or five), watch it, choose another, all without restarting Isaac Sim. The cell is
run_cell.py's, with the same options (--perception camera or ros, --look-first, ...); the gripper
can be any calibrated gripper of the grasp matrix, the object any with robust grasps for it.
Results go to results/pick_place_interactive.sqlite, apart from the batch results.

    C:/issacsim-6-2/python.bat pick_place/cell_app.py
    C:/issacsim-6-2/python.bat pick_place/cell_app.py --perception camera
    (ROS 2: start the locator, set RMW_IMPLEMENTATION=rmw_fastrtps_cpp)  ... cell_app.py --perception ros

Changing the gripper builds the cell again on a new stage (the gripper becomes part of the arm's
articulation, and the planner's model of it changes); changing the object only swaps the object.
With the camera, each gripper carries it at its own place (camera_model.MOUNT_XYZ) and needs its
own hand-eye calibration (handeye.py --gripper). With ROS 2 the gripper stays the one given with
--gripper: the locator's camera mount comes from its launch file.

Each object's resting pose on the grasp matrix's table, which its grasps refer to, is found by
letting it settle there (cell.rest_mount(): a separate stage, so it cannot be done once the cell
is built). The first start settles every object offered, for every gripper, and keeps the poses in
results/cell_rest_mounts.json; later starts read the file. Delete the file after changing the
objects' assets or the grasp matrix's settling.

--script runs steps as if clicked, then stays open (or quits with a 'quit' step):
    --script "mug_A07RDPW843:1,gripper:onrobot_rg2,toy_block_6MPL7NVFVU:2,shot:results/app.png,quit"
<object>:<n> chooses the object and runs n cycles; gripper:<name> chooses the gripper; pose:<n>
sets the start pose; shot:<file> saves a picture of the window.
"""

import asyncio
import json
import pathlib
import sqlite3
import sys
import traceback

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import cell_args  # noqa: E402

parser = cell_args.parser(__doc__, "results/pick_place_interactive.sqlite", headless_option=False)
parser.add_argument("--grippers", help="comma-separated grippers to offer (default: every calibrated gripper with "
                                       "robust grasps; with ROS 2 only --gripper)")
parser.add_argument("--objects", help="comma-separated objects to offer (default: every object with robust grasps)")
parser.add_argument("--start-pose", type=int, default=0, help="the first cycle's start pose (seeded: the same number, the same start)")
parser.add_argument("--script", help="steps to run as if clicked (see above)")
args, _ = parser.parse_known_args()
app = cell_args.start(args, headless=False)

import cell  # noqa: E402
import cell_db  # noqa: E402
import cell_session  # noqa: E402
import gm_config  # noqa: E402
import numpy as np  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.ui as ui  # noqa: E402
from cell_session import log, next_update  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402

from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402

REST_MOUNTS = gm_config.ROOT / "results/cell_rest_mounts.json"
GOOD, BAD, DIM = 0xFF7FD67F, 0xFF6A8CFF, 0xFFA0A0A0  # omni.ui colours are 0xAABBGGRR
LABELS = {k: v.get("label", k) for k, v in gm_config.gripper_config().items()}


async def stop_simulation() -> None:
    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_stopped():
        timeline.stop()
        await next_update()


async def rest_mounts(gripper: str, g: dict, objects: list[str]) -> dict[str, Gf.Matrix4d]:
    """Each object's mount after it settles on the grasp matrix's table, from the file or settled now."""
    stored = json.loads(REST_MOUNTS.read_text(encoding="utf-8")) if REST_MOUNTS.exists() else {}
    mine = stored.setdefault(gripper, {})
    missing = [o for o in objects if o not in mine]
    for k, name in enumerate(missing):
        log(f"settling {name} on the grasp matrix's table for the {LABELS[gripper]} ({k + 1}/{len(missing)}; "
            f"once, kept in {REST_MOUNTS.name})")
        mine[name] = np.array(await cell.rest_mount(gripper, g, name), dtype=float).tolist()
        REST_MOUNTS.write_text(json.dumps(stored, indent=1), encoding="utf-8")
    return {name: Gf.Matrix4d(mine[name]) for name in objects}


def offered_objects(gripper: str) -> tuple[list[str], dict[str, int], set[str]]:
    """The objects to offer with this gripper, how many robust grasps each has, and which have batch results."""
    names = args.objects.split(",") if args.objects else sorted(gm_config.load_objects())
    counts = {}
    for name in names:
        counts[name] = len(cell_db.robust_grasps(str(gm_config.ROOT / args.matrix_db), gripper, name)[1])
    batch_db = gm_config.ROOT / "results/pick_place.sqlite"
    tested = set()
    if batch_db.exists():
        with sqlite3.connect(batch_db) as conn:
            tested = {r[0] for r in conn.execute("SELECT DISTINCT object FROM cell_runs WHERE finished IS NOT NULL AND gripper=?",
                                                 (gripper,))}
    return [n for n in names if counts[n] > 0], counts, tested


class Panel:
    """The window: gripper, object, start pose, run buttons, the cycle's progress and the results so far.
    menu: gripper -> (objects, robust grasps per object, objects with batch results);
    rests: gripper -> object -> rest mount."""

    def __init__(self, session: cell_session.CellSession, menu: dict[str, tuple[list[str], dict[str, int], set[str]]],
                 rests: dict[str, dict[str, Gf.Matrix4d]]):
        self.session, self.menu, self.rests = session, menu, rests
        self.grippers = list(menu)
        self.objects: list[str] = []
        self.busy, self.stop_requested = False, False
        self.lock = asyncio.Lock()
        self.fixed_gripper = args.perception == "ros"
        mode = {"truth": "the simulation (true pose)", "camera": "the wrist camera", "blind": "nowhere (blind)",
                "ros": "the wrist camera, via the ROS 2 object_locator"}[args.perception]
        info = f"The object's pose from {mode}."
        if self.fixed_gripper:
            info += " The gripper stays as started: the locator's camera mount comes from its launch file."

        self.window = ui.Window("Pick-and-place cell", width=460, height=660)
        with self.window.frame:
            with ui.VStack(spacing=6):
                with ui.VStack(spacing=6, height=0):
                    ui.Label(info, word_wrap=True, style={"color": DIM})
                    ui.Label("Gripper")
                    self.gripper_combo = ui.ComboBox(self.grippers.index(session.gripper),
                                                     *[self._gripper_label(g) for g in self.grippers])
                    self.gripper_combo.model.add_item_changed_fn(lambda model, item: self._on_gripper())
                    ui.Label("Object")
                    self.object_box = ui.VStack(height=0)  # its list changes with the gripper
                    with ui.HStack(spacing=6, height=0):
                        ui.Label("Start pose", width=0)
                        self.pose = ui.IntField(width=60)
                        self.pose.model.set_value(args.start_pose)
                        self.pose.model.add_value_changed_fn(lambda model: self._on_pose())
                        ui.Label("seeded: the same number puts the object at the same place and turn", word_wrap=True,
                                 style={"color": DIM})
                    with ui.HStack(spacing=6, height=30):
                        self.run1 = ui.Button("Run 1 cycle", clicked_fn=lambda: self.start(1))
                        self.run5 = ui.Button("Run 5 cycles", clicked_fn=lambda: self.start(5))
                        self.stop = ui.Button("Stop after this cycle", clicked_fn=self.request_stop, enabled=False)
                    self.status = ui.Label("Choose a gripper and an object, and run a cycle.", word_wrap=True, height=44)
                    ui.Label("Results  (start pose: outcome, place error; with the camera: how far off the found pose was)",
                             word_wrap=True, style={"color": DIM})
                with ui.ScrollingFrame():
                    self.results = ui.VStack(spacing=2, height=0)
        self.window.deferred_dock_in("Property", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)
        self._fill_objects()
        self._enable(True)
        self._heading()

    def _gripper_label(self, gripper: str) -> str:
        text = f"{LABELS[gripper]}   {len(self.menu[gripper][0])} objects"
        if self.session.cannot_use(gripper):
            text += "   (no hand-eye calibration)"
        return text

    def _fill_objects(self) -> None:
        """The object list for the current gripper."""
        s = self.session
        self.objects, counts, tested = self.menu[s.gripper]
        labels = [f"{n}   {counts[n]} grasps{'   (batch-tested)' if n in tested else ''}" for n in self.objects]
        self.object_box.clear()
        with self.object_box:
            self.combo = ui.ComboBox(self.objects.index(s.object_name), *labels)
        self.combo.model.add_item_changed_fn(lambda model, item: self._on_object())

    # --- actions -------------------------------------------------------------------------------
    # Every change to the stage waits for the one before it (self.lock), and happens with the
    # simulation stopped; a cycle starts from stopped, as in the batch runs.
    def _on_gripper(self) -> None:
        if not self.busy:
            asyncio.ensure_future(self._switch_gripper(self.grippers[self.gripper_combo.model.get_item_value_model().as_int]))

    def _on_object(self) -> None:
        if not self.busy:
            asyncio.ensure_future(self._switch(self.objects[self.combo.model.get_item_value_model().as_int]))

    def _on_pose(self) -> None:
        if not self.busy:
            asyncio.ensure_future(self._locked(self.show_next))

    def start(self, n: int) -> None:
        if self.busy:
            return
        self.busy, self.stop_requested = True, False
        self._enable(False)
        asyncio.ensure_future(self._run(n))

    def request_stop(self) -> None:
        self.stop_requested = True
        self.status.text += "  (stopping after this cycle)"

    async def _locked(self, fn) -> None:
        async with self.lock:
            await fn()

    async def _switch(self, name: str) -> None:
        async with self.lock:
            if name == self.session.object_name:
                return
            await stop_simulation()
            self.session.finish_run()
            if self.session.load_object(name, self.rests[self.session.gripper][name]):
                self._heading()
            await self.show_next()

    async def _switch_gripper(self, gripper: str) -> None:
        """Build the cell again with this gripper, keeping the object if the gripper has robust
        grasps for it (otherwise the first in its list)."""
        async with self.lock:
            s = self.session
            if gripper == s.gripper:
                return
            problem = s.cannot_use(gripper)
            if problem:
                self.status.text = problem
                self.gripper_combo.model.get_item_value_model().set_value(self.grippers.index(s.gripper))
                return
            objects = self.menu[gripper][0]
            name = s.object_name if s.object_name in objects else objects[0]
            self.busy = True
            self._enable(False)
            self.status.text = f"Building the cell with the {LABELS[gripper]}..."
            ok = False
            try:
                await stop_simulation()
                s.finish_run()
                await next_update()  # show the status first
                ok = await s.build(name, self.rests[gripper][name], gripper=gripper)
            except Exception as e:  # noqa: BLE001  shown in the window
                traceback.print_exc()
                self.status.text = f"error: {e}"
            finally:
                self.busy = False
                self._enable(True)
            if ok:
                self._fill_objects()
                self._heading()
                await self.show_next()

    async def show_next(self) -> None:
        """Show the cell as the next cycle starts: the object at its start pose, the arm at home.
        The stage holds the arm at zero joint angles (lying over the table) until the simulation
        runs, so run two physics steps and pause."""
        await stop_simulation()
        s, i = self.session, self.pose.model.as_int
        yaw, pick, _ = s.start_pose(i)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(2):
            await next_update()
        timeline.pause()
        await next_update()
        self.status.text = (f"{LABELS[s.gripper]} x {s.object_name} ({len(s.grasps)} robust grasps), start pose {i}: "
                            f"turned {yaw:+.0f} deg at ({pick[0]:.3f}, {pick[1]:.3f}). Ready.")

    async def _run(self, n: int) -> None:
        s = self.session
        async with self.lock:
            try:
                await stop_simulation()
                if s.run_id is None:
                    s.start_run(args.db, interactive=True)
                for k in range(n):
                    i = self.pose.model.as_int
                    self.guard_scene()
                    r = await s.run_cycle(i, f"start pose {i}" + (f" ({k + 1}/{n})" if n > 1 else ""), on_log=self._on_log)
                    self._result(i, r)
                    self.pose.model.set_value(i + 1)
                    if self.stop_requested:
                        break
            except Exception as e:  # noqa: BLE001  shown in the window; the app stays usable
                traceback.print_exc()
                self.status.text = f"error: {e}"
            finally:
                self.busy = False
                self._enable(True)
            if not self.status.text.startswith("error"):
                await self.show_next()

    # --- display -------------------------------------------------------------------------------
    def _on_log(self, line: str) -> None:
        self.status.text = line.replace("[cell] ", "")

    def _enable(self, idle: bool) -> None:
        self.combo.enabled = self.pose.enabled = self.run1.enabled = self.run5.enabled = idle
        self.gripper_combo.enabled = idle and not self.fixed_gripper
        self.stop.enabled = not idle

    def _heading(self) -> None:
        with self.results:
            ui.Label(f"{LABELS[self.session.gripper]} x {self.session.object_name}", height=22, style={"font_size": 16})

    def guard_scene(self) -> None:
        """Before a cycle: put back what the cycles rely on staying put, if it was moved in the stage
        (selected and dragged in the viewport, say). The wrist camera goes back on its mount (its
        images must match the hand-eye calibration); a moved robot base stops the run."""
        stage = self.session.cell.stage
        base = cell_session.to_np(UsdGeom.Xformable(stage.GetPrimAtPath(cell.ROBOT)).ComputeLocalToWorldTransform(0))
        if np.linalg.norm(base[:3, 3]) > 1e-4 or not np.allclose(base[:3, :3], np.eye(3), atol=1e-4):
            raise RuntimeError(f"the robot's base was moved in the stage ({np.linalg.norm(base[:3, 3]) * 1000:.1f} mm "
                               "from the origin); restart the window")
        if self.session.camera is not None:
            import camera_model
            import wrist_camera

            xf = UsdGeom.Xformable(stage.GetPrimAtPath(wrist_camera.CAMERA))
            local, mount = cell_session.to_np(xf.GetLocalTransformation()), camera_model.mount(self.session.gripper)
            off = np.linalg.norm(local[:3, 3] - mount[:3, 3])
            if off > 1e-4 or not np.allclose(local[:3, :3], mount[:3, :3], atol=1e-4):
                xf.ClearXformOpOrder()
                xf.AddTransformOp().Set(cell_session.to_gf(mount))
                text = f"the wrist camera had been moved in the stage ({off * 1000:.1f} mm); put back on its mount"
                log(text)
                with self.results:
                    ui.Label(text, word_wrap=True, style={"color": BAD})

    def _result(self, i: int, r: dict) -> None:
        text = f"{i:>4}   {r['outcome']}"
        if r.get("place_error_m") is not None:
            text += f", {r['place_error_m'] * 1000:.1f} mm from the target"
        if r.get("est_centre_mm") is not None:
            text += f"; found {r['est_centre_mm']:.1f} mm / {r['est_rot_deg']:.1f} deg off"
        if r.get("grasp_index") is not None:
            text += f"; grasp {r['grasp_index']}"
        if r["outcome"] != "placed" and r.get("detail"):
            text += f"  ({r['detail']})"
        with self.results:
            ui.Label(text, word_wrap=True, style={"color": GOOD if r["outcome"] == "placed" else BAD})


async def run_script(panel: Panel, steps: list[str]) -> None:
    async def settled() -> None:  # until the panel has done what it was asked
        for _ in range(5):
            await next_update()
        while panel.busy or panel.lock.locked():
            await next_update()

    for step in steps:
        if step == "quit":
            omni.kit.app.get_app().post_quit()
            return
        if step.startswith("shot:"):
            await save_window_picture(gm_config.ROOT / step[5:])
            continue
        if step.startswith("gripper:"):
            panel.gripper_combo.model.get_item_value_model().set_value(panel.grippers.index(step[8:]))
            await settled()
            continue
        if step.startswith("pose:"):
            panel.pose.model.set_value(int(step[5:]))
            await settled()
            continue
        name, n = step.rsplit(":", 1)
        panel.combo.model.get_item_value_model().set_value(panel.objects.index(name))
        await settled()
        panel.start(int(n))
        await settled()


async def save_window_picture(path: pathlib.Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    for _ in range(10):  # let the window draw what changed
        await next_update()
    try:
        import omni.appwindow
        import omni.kit.renderer.capture

        window = omni.appwindow.get_default_app_window()
        capture = omni.kit.renderer.capture.acquire_renderer_capture_interface()
        capture.capture_next_frame_swapchain(str(path), window)
        for _ in range(5):
            await next_update()
        capture.wait_async_capture(window)
        log(f"window picture: {path}")
    except Exception as e:  # noqa: BLE001  a picture is not worth stopping for
        log(f"no window picture: {e}")


state: dict = {}


async def main() -> None:
    SimulationManager.switch_physics_engine("physx")
    calibrated = gm_config.load_grippers()
    if args.gripper not in calibrated:
        raise RuntimeError(f"{args.gripper} is not a calibrated gripper of the grasp matrix")
    if args.perception == "ros":
        names = [args.gripper]
    elif args.grippers:
        names = args.grippers.split(",")
        unknown = [g for g in names if g not in calibrated]
        if unknown:
            raise RuntimeError(f"not calibrated grippers of the grasp matrix: {unknown}")
    else:
        names = list(calibrated)
    menu = {}
    for gripper in names:
        offer = offered_objects(gripper)
        if offer[0]:
            menu[gripper] = offer
    if args.gripper not in menu:
        raise RuntimeError(f"no object with robust grasps for {args.gripper}")
    # Settle every object offered now, before the cell is built: on its own stage, with the physics
    # as at start-up (the cell switches it to the CPU), as run_cell.py does.
    rests = {gripper: await rest_mounts(gripper, calibrated[gripper], offer[0]) for gripper, offer in menu.items()}
    objects = menu[args.gripper][0]
    first = args.object if args.object in objects else objects[0]
    session = state["session"] = cell_session.CellSession(args)
    await session.build(first, rests[args.gripper][first])
    panel = state["panel"] = Panel(session, menu, rests)
    await panel._locked(panel.show_next)
    log(f"ready: {len(menu)} grippers, {sum(len(o[0]) for o in menu.values())} gripper x object pairs; results go to {args.db}")
    if args.script:
        await run_script(panel, args.script.split(","))


task = asyncio.ensure_future(main())
while app.is_running():
    app.update()
    if task.done() and task.exception() is not None:
        break
try:
    if task.done() and task.exception() is not None:
        raise task.exception()
finally:
    if "session" in state:
        state["session"].close()
    app.close()
