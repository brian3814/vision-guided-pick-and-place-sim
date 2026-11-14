"""The pick-and-place cell in a window, to run interactively: choose the object, run a cycle (or
five), watch it, choose another object, all without restarting Isaac Sim. The cell is run_cell.py's,
with the same options (--perception camera or ros, --look-first, ...); the object can be any with
robust grasps in the grasp matrix. Results go to results/pick_place_interactive.sqlite, apart from
the batch results.

    C:/issacsim-6-2/python.bat pick_place/cell_app.py
    C:/issacsim-6-2/python.bat pick_place/cell_app.py --perception camera
    (ROS 2: start the locator, set RMW_IMPLEMENTATION=rmw_fastrtps_cpp)  ... cell_app.py --perception ros

Each object's resting pose on the grasp matrix's table, which its grasps refer to, is found by
letting it settle there (cell.rest_mount(): a separate stage, so it cannot be done once the cell
is built). The first start settles every object and keeps the poses in
results/cell_rest_mounts.json; later starts read the file. Delete the file after changing the
objects' assets or the grasp matrix's settling.

--script runs steps as if clicked, then stays open (or quits with a 'quit' step):
    --script "mug_A07RDPW843:1,lemon_8SUCD6125H:2,shot:results/app.png,quit"
<object>:<n> chooses the object and runs n cycles; shot:<file> saves a picture of the window.
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
        log(f"settling {name} on the grasp matrix's table ({k + 1}/{len(missing)}; once, kept in {REST_MOUNTS.name})")
        mine[name] = np.array(await cell.rest_mount(gripper, g, name), dtype=float).tolist()
        REST_MOUNTS.write_text(json.dumps(stored, indent=1), encoding="utf-8")
    return {name: Gf.Matrix4d(mine[name]) for name in objects}


def offered_objects() -> tuple[list[str], dict[str, int], set[str]]:
    """The objects to offer, how many robust grasps each has, and which have batch results."""
    names = args.objects.split(",") if args.objects else sorted(gm_config.load_objects())
    counts = {}
    for name in names:
        counts[name] = len(cell_db.robust_grasps(str(gm_config.ROOT / args.matrix_db), args.gripper, name)[1])
    batch_db = gm_config.ROOT / "results/pick_place.sqlite"
    tested = set()
    if batch_db.exists():
        with sqlite3.connect(batch_db) as conn:
            tested = {r[0] for r in conn.execute("SELECT DISTINCT object FROM cell_runs WHERE finished IS NOT NULL AND gripper=?",
                                                 (args.gripper,))}
    return [n for n in names if counts[n] > 0], counts, tested


class Panel:
    """The window: object, start pose, run buttons, the cycle's progress and the results so far."""

    def __init__(self, session: cell_session.CellSession, objects: list[str], counts: dict[str, int], tested: set[str],
                 rests: dict[str, Gf.Matrix4d]):
        self.session, self.objects, self.rests = session, objects, rests
        self.busy, self.stop_requested = False, False
        self.lock = asyncio.Lock()
        mode = {"truth": "the simulation (true pose)", "camera": "the wrist camera",
                "ros": "the wrist camera, via the ROS 2 object_locator"}[args.perception]
        labels = [f"{n}   {counts[n]} grasps{'   (batch-tested)' if n in tested else ''}" for n in objects]

        self.window = ui.Window("Pick-and-place cell", width=460, height=620)
        with self.window.frame:
            with ui.VStack(spacing=6):
                with ui.VStack(spacing=6, height=0):
                    ui.Label(f"Gripper {args.gripper}; the object's pose from {mode}.", word_wrap=True,
                             style={"color": DIM})
                    ui.Label("Object")
                    self.combo = ui.ComboBox(objects.index(session.object_name), *labels)
                    self.combo.model.add_item_changed_fn(lambda model, item: self._on_object())
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
                    self.status = ui.Label("Choose an object and run a cycle.", word_wrap=True, height=44)
                    ui.Label("Results  (start pose: outcome, place error; with the camera: how far off the found pose was)",
                             word_wrap=True, style={"color": DIM})
                with ui.ScrollingFrame():
                    self.results = ui.VStack(spacing=2, height=0)
        self.window.deferred_dock_in("Property", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)
        self._heading(session.object_name)

    # --- actions -------------------------------------------------------------------------------
    # Every change to the stage waits for the one before it (self.lock), and happens with the
    # simulation stopped; a cycle starts from stopped, as in the batch runs.
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
            if self.session.load_object(name, self.rests[name]):
                self._heading(name)
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
        self.status.text = (f"{s.object_name} ({len(s.grasps)} robust grasps), start pose {i}: turned {yaw:+.0f} deg "
                            f"at ({pick[0]:.3f}, {pick[1]:.3f}). Ready.")

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
        self.stop.enabled = not idle

    def _heading(self, name: str) -> None:
        with self.results:
            ui.Label(name, height=22, style={"font_size": 16})

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
            local, mount = cell_session.to_np(xf.GetLocalTransformation()), camera_model.mount()
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
    g = gm_config.load_grippers()[args.gripper]
    objects, counts, tested = offered_objects()
    if not objects:
        raise RuntimeError(f"no object with robust grasps for {args.gripper}")
    rests = await rest_mounts(args.gripper, g, objects)
    first = args.object if args.object in objects else objects[0]
    session = state["session"] = cell_session.CellSession(args)
    await session.build(first, rests[first])
    panel = state["panel"] = Panel(session, objects, counts, tested, rests)
    await panel._locked(panel.show_next)
    log(f"ready: {len(objects)} objects; results go to {args.db}")
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
