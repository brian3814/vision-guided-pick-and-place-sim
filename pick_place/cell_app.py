"""The pick-and-place cell in a window, to run interactively: choose the gripper and the object, run
a cycle (or five), watch it, choose another, all without restarting Isaac Sim. The cell is
run_cell.py's, with the same options (--perception camera or ros, --look-first, ...); the gripper
can be any calibrated gripper of the grasp matrix, the object any with robust grasps for it.
Results go to results/pick_place_interactive.sqlite, apart from the batch results.

    C:/issacsim-6-2/python.bat pick_place/cell_app.py
    C:/issacsim-6-2/python.bat pick_place/cell_app.py --perception camera
    (ROS 2: start the locator, set RMW_IMPLEMENTATION=rmw_fastrtps_cpp)  ... cell_app.py --perception ros
    C:/issacsim-6-2/python.bat pick_place/cell_app.py --web    (the window in a browser, below)

Changing the gripper builds the cell again on a new stage (the gripper becomes part of the arm's
articulation, and the planner's model of it changes); changing the object only swaps the object.
With the camera, each gripper carries it at its own place (camera_model.MOUNT_XYZ) and needs its
own hand-eye calibration (handeye.py --gripper). With ROS 2 the gripper stays the one given with
--gripper: the locator's camera mount comes from its launch file.

The window's rules (what can be chosen when, running and stopping, what it says) are in
cell_control.py, which knows nothing of Kit; the panel here draws its state and events, and KitCell
does the work in the stage.

With --web, Isaac Sim runs without its window and streams the viewport over WebRTC, for a web page
that draws the panel itself: web_bridge.py answers the page, livestream.py puts it on the stream.
The page switches the viewport's camera too (viewport_cameras.py), having no Kit menus, and its legend
names the stage's unit, so the stream leaves out Kit's unit box (the 'm' at the bottom left).
The page connects once the log says so. Kit's own panel is there too, hidden with the rest of Kit's
UI; add --/app/window/hideUi=false to stream it with the viewport. Kit streams to one page at a time.

Each object's resting pose on the grasp matrix's table, which its grasps refer to, is found by
letting it settle there (cell.rest_mount(): a separate stage, so it cannot be done once the cell
is built). The first start settles every object offered, for every gripper, and keeps the poses in
results/cell_rest_mounts.json; later starts read the file. Delete the file after changing the
objects' assets or the grasp matrix's settling.

--dev turns on Kit's Python server, so that remote_python.py can look into the running cell from
outside, without restarting Isaac Sim (on 127.0.0.1, with a token made at each start; see there).

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

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import cell_args  # noqa: E402

parser = cell_args.parser(__doc__, "results/pick_place_interactive.sqlite", headless_option=False)
parser.add_argument("--grippers", help="comma-separated grippers to offer (default: every calibrated gripper with "
                                       "robust grasps; with ROS 2 only --gripper)")
parser.add_argument("--objects", help="comma-separated objects to offer (default: every object with robust grasps)")
parser.add_argument("--start-pose", type=int, default=0, help="the first cycle's start pose (seeded: the same number, the same start)")
parser.add_argument("--script", help="steps to run as if clicked (see above)")
parser.add_argument("--web", action="store_true", help="stream the viewport for a web page that draws the panel (see above)")
parser.add_argument("--dev", action="store_true", help="turn on Kit's Python server, for remote_python.py (see above)")
args, _ = parser.parse_known_args()
app = cell_args.start(args, headless=args.web)

import cell  # noqa: E402
import cell_control  # noqa: E402
import cell_db  # noqa: E402
import cell_session  # noqa: E402
import gm_config  # noqa: E402
import livestream  # noqa: E402
import numpy as np  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.ui as ui  # noqa: E402
import remote_python  # noqa: E402
import viewport_cameras  # noqa: E402
import web_bridge  # noqa: E402
from cell_session import log, next_update  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402

from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402

if args.web:
    livestream.enable()  # the stream is up while the cell is built; the page connects once it is
token = remote_python.enable() if args.dev else None  # it listens while the cell is built

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


def offered_objects(gripper: str) -> cell_control.Offer:
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


class KitCell:
    """The cell in Isaac Sim, as cell_control.CellControl uses it: a CellSession, the objects' rest
    mounts (rests: gripper -> object -> mount), and the simulation's timeline. Every change to the
    stage happens with the simulation stopped; a cycle starts from stopped, as in the batch runs."""

    def __init__(self, session: cell_session.CellSession, rests: dict[str, dict[str, Gf.Matrix4d]]):
        self.session, self.rests = session, rests

    @property
    def gripper(self) -> str:
        return self.session.gripper

    @property
    def object_name(self) -> str:
        return self.session.object_name

    @property
    def grasp_count(self) -> int:
        return len(self.session.grasps)

    def cannot_use(self, gripper: str) -> str | None:
        return self.session.cannot_use(gripper)

    async def load_object(self, name: str) -> bool:
        s = self.session
        await stop_simulation()
        s.finish_run()
        return s.load_object(name, self.rests[s.gripper][name])

    async def build(self, gripper: str, name: str) -> bool:
        s = self.session
        await stop_simulation()
        s.finish_run()
        await next_update()  # show the status first
        return await s.build(name, self.rests[gripper][name], gripper=gripper)

    async def show(self, pose: int) -> tuple[float, tuple[float, float]]:
        """The object at the start pose, the arm at home. The stage holds the arm at zero joint angles
        (lying over the table) until the simulation runs, so run two physics steps and pause."""
        await stop_simulation()
        yaw, pick, _ = self.session.start_pose(pose)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(2):
            await next_update()
        timeline.pause()
        await next_update()
        return yaw, (float(pick[0]), float(pick[1]))

    async def begin_run(self) -> None:
        await stop_simulation()
        if self.session.run_id is None:
            self.session.start_run(args.db, interactive=True)

    def guard(self) -> list[str]:
        """Before a cycle: put back what the cycles rely on staying put, if it was moved in the stage
        (selected and dragged in the viewport, say). The wrist camera goes back on its mount (its
        images must match the hand-eye calibration); a moved robot base stops the run."""
        stage = self.session.cell.stage
        base = cell_session.to_np(UsdGeom.Xformable(stage.GetPrimAtPath(cell.ROBOT)).ComputeLocalToWorldTransform(0))
        if np.linalg.norm(base[:3, 3]) > 1e-4 or not np.allclose(base[:3, :3], np.eye(3), atol=1e-4):
            raise RuntimeError(f"the robot's base was moved in the stage ({np.linalg.norm(base[:3, 3]) * 1000:.1f} mm "
                               "from the origin); restart the window")
        if self.session.camera is None:
            return []
        import camera_model
        import wrist_camera

        xf = UsdGeom.Xformable(stage.GetPrimAtPath(wrist_camera.CAMERA))
        local, mount = cell_session.to_np(xf.GetLocalTransformation()), camera_model.mount(self.session.gripper)
        off = np.linalg.norm(local[:3, 3] - mount[:3, 3])
        if off <= 1e-4 and np.allclose(local[:3, :3], mount[:3, :3], atol=1e-4):
            return []
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(cell_session.to_gf(mount))
        text = f"the wrist camera had been moved in the stage ({off * 1000:.1f} mm); put back on its mount"
        log(text)
        return [text]

    async def run_cycle(self, pose: int, label: str, on_log) -> dict:
        return await self.session.run_cycle(pose, label, on_log=on_log)


class Panel:
    """The window, a view of the CellControl: gripper, object, start pose, run buttons, the cycle's
    latest step and the results so far. It draws the controller's state and events; its widgets
    call the controller's actions."""

    def __init__(self, control: cell_control.CellControl):
        self.control = control
        self.grippers: list[str] = []
        self.objects: list[str] = []
        self.objects_for: str | None = None  # the gripper the object list was filled for
        self.syncing = False  # widgets set from the state: not the user's choice
        state = control.state()

        self.window = ui.Window("Pick-and-place cell", width=460, height=660)
        with self.window.frame:
            with ui.VStack(spacing=6):
                with ui.VStack(spacing=6, height=0):
                    ui.Label(state["info"], word_wrap=True, style={"color": DIM})
                    ui.Label("Gripper")
                    self.grippers = [g["name"] for g in state["grippers"]]
                    self.gripper_combo = ui.ComboBox(self.grippers.index(state["gripper"]), *[g["text"] for g in state["grippers"]])
                    self.gripper_combo.model.add_item_changed_fn(lambda model, item: self._on_gripper())
                    ui.Label("Object")
                    self.object_box = ui.VStack(height=0)  # its list changes with the gripper
                    with ui.HStack(spacing=6, height=0):
                        ui.Label("Start pose", width=0)
                        self.pose = ui.IntField(width=60)
                        self.pose.model.set_value(state["pose"])
                        self.pose.model.add_value_changed_fn(lambda model: self._on_pose())
                        ui.Label("seeded: the same number puts the object at the same place and turn", word_wrap=True,
                                 style={"color": DIM})
                    with ui.HStack(spacing=6, height=30):
                        self.run1 = ui.Button("Run 1 cycle", clicked_fn=lambda: control.run(1))
                        self.run5 = ui.Button("Run 5 cycles", clicked_fn=lambda: control.run(5))
                        self.stop = ui.Button("Stop after this cycle", clicked_fn=control.stop)
                    self.status = ui.Label(state["status"], word_wrap=True, height=44)
                    ui.Label("Results  (start pose: outcome, place error; with the camera: how far off the found pose was)",
                             word_wrap=True, style={"color": DIM})
                with ui.ScrollingFrame():
                    self.results = ui.VStack(spacing=2, height=0)
        self.window.deferred_dock_in("Property", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)
        self._sync(state)
        self.unsubscribe = control.subscribe(self.on_event)

    # --- the user's choices ----------------------------------------------------------------------
    def _on_gripper(self) -> None:
        if not self.syncing:
            self.control.choose_gripper(self.grippers[self.gripper_combo.model.get_item_value_model().as_int])

    def _on_object(self) -> None:
        if not self.syncing:
            self.control.choose_object(self.objects[self.combo.model.get_item_value_model().as_int])

    def _on_pose(self) -> None:
        if not self.syncing:
            self.control.set_pose(self.pose.model.as_int)

    # --- drawing ---------------------------------------------------------------------------------
    def on_event(self, event: dict) -> None:
        kind = event["type"]
        if kind == "state":
            self._sync(event["state"])
            return
        with self.results:
            if kind == "heading":
                ui.Label(event["text"], height=22, style={"font_size": 16})
            elif kind == "result":
                ui.Label(event["text"], word_wrap=True, style={"color": GOOD if event["ok"] else BAD})
            elif kind == "warning":
                ui.Label(event["text"], word_wrap=True, style={"color": BAD})

    def _sync(self, state: dict) -> None:
        """Set the widgets to the state. While busy the lists keep what the user chose, so a gripper
        being built stays shown."""
        self.syncing = True
        try:
            self.status.text = state["status"]
            if state["gripper"] != self.objects_for:
                self._fill_objects(state)
            if not state["busy"]:
                self._choose(self.gripper_combo, self.grippers.index(state["gripper"]))
                self._choose(self.combo, self.objects.index(state["object"]))
            if self.pose.model.as_int != state["pose"]:
                self.pose.model.set_value(state["pose"])
            idle = not state["busy"]
            self.combo.enabled = self.pose.enabled = self.run1.enabled = self.run5.enabled = idle
            self.gripper_combo.enabled = idle and not state["fixed_gripper"]
            self.stop.enabled = not idle
        finally:
            self.syncing = False

    @staticmethod
    def _choose(combo: ui.ComboBox, index: int) -> None:
        item = combo.model.get_item_value_model()
        if item.as_int != index:
            item.set_value(index)

    def _fill_objects(self, state: dict) -> None:
        """The object list for the state's gripper."""
        self.objects = [o["name"] for o in state["objects"]]
        self.object_box.clear()
        with self.object_box:
            self.combo = ui.ComboBox(self.objects.index(state["object"]), *[o["text"] for o in state["objects"]])
        self.combo.model.add_item_changed_fn(lambda model, item: self._on_object())
        self.objects_for = state["gripper"]


async def run_script(control: cell_control.CellControl, steps: list[str]) -> None:
    """Steps as if chosen in the window, each once the one before it has done its work."""
    for step in steps:
        if step == "quit":
            omni.kit.app.get_app().post_quit()
            return
        if step.startswith("shot:"):
            await save_window_picture(gm_config.ROOT / step[5:])
            continue
        if step.startswith("gripper:"):
            problem = control.choose_gripper(step[8:])
        elif step.startswith("pose:"):
            problem = control.set_pose(int(step[5:]))
        else:
            name, n = step.rsplit(":", 1)
            problem = control.choose_object(name)
            await control.settled()
            problem = problem or control.run(int(n))
        if problem:
            raise RuntimeError(f"script step {step}: {problem}")
        await control.settled()


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
    if token is not None:
        state["published"], message = await remote_python.publish(token, next_update)
        log(message)
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
    control = cell_control.CellControl(KitCell(session, rests), menu, LABELS, args.perception, pose=args.start_pose)
    state["panel"] = Panel(control)
    if args.web:
        kit_viewport = viewport_cameras.KitViewport()
        if not kit_viewport.hide_unit_box():
            log("Kit's unit box was not found to hide; the stream shows it beside the page's legend")
        view = viewport_cameras.ViewportCameras(kit_viewport)
        bridge = state["bridge"] = web_bridge.WebBridge(control, view=view)
        state["disconnect"] = livestream.connect(bridge)
    await control.start()
    log(f"ready: {len(menu)} grippers, {sum(len(o[0]) for o in menu.values())} gripper x object pairs; results go to {args.db}")
    if args.web:
        log(f"the web page can connect over the stream (signalling on port {livestream.SIGNAL_PORT})")
    if args.script:
        await run_script(control, args.script.split(","))


task = asyncio.ensure_future(main())
while app.is_running():
    app.update()
    if task.done() and task.exception() is not None:
        break
try:
    if task.done() and task.exception() is not None:
        raise task.exception()
finally:
    if "disconnect" in state:
        state["disconnect"]()
        state["bridge"].close()
    if "session" in state:
        state["session"].close()
    if state.get("published") is not None:
        remote_python.withdraw(state["published"])
    app.close()
