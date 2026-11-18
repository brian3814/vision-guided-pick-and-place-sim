"""The interactive window's rules, apart from Kit: which gripper and object are chosen, the start
pose, running cycles and stopping, and what the window says about them. cell_app.py's omni.ui panel
is a view of it: it calls the actions and draws from the events. Another view (a web page) can do
the same, so views cannot drift apart. Nothing here imports Kit. The cell itself comes in through
`Cell` (cell_app.KitCell in Isaac Sim, a fake in tests/test_cell_control.py).

Actions (choose_gripper, choose_object, set_pose, run, stop) return None when taken, or why not.
They start the work and return at once; what happens comes as events, to every subscriber, in order:

    state     {"type": "state", "state": state()}: whenever what the controls show changed
    heading   a gripper x object starts in the results list
    result    one cycle's result, with the line the views show (`text`) and whether it placed (`ok`)
    warning   something put right before a cycle (the wrist camera moved in the stage)

Every change to the stage waits for the one before it (`lock`). `busy` is set while cycles run or
the cell is being built with another gripper: the controls are then off, and actions are refused.
"""

from __future__ import annotations

import asyncio
import traceback
from collections.abc import Callable, Coroutine
from typing import Any, Protocol

READY = "Choose a gripper and an object, and run a cycle."
BUSY = "busy: wait until the cycles run, or the cell being built, are done"
MODES = {"truth": "the simulation (true pose)", "camera": "the wrist camera", "blind": "nowhere (blind)",
         "ros": "the wrist camera, via the ROS 2 object_locator"}
RESULT_FIELDS = ("outcome", "place_error_m", "tilt_deg", "est_centre_mm", "est_rot_deg", "grasp_index", "detail", "cycle_s")

# gripper -> (objects offered, robust grasps per object, objects with batch results)
Offer = tuple[list[str], dict[str, int], set[str]]
Listener = Callable[[dict], None]


class Cell(Protocol):
    """What the controller needs of the cell. Each coroutine leaves the simulation stopped, or (show)
    paused at the next cycle's start."""

    @property
    def gripper(self) -> str: ...

    @property
    def object_name(self) -> str: ...

    @property
    def grasp_count(self) -> int: ...  # robust grasps of the gripper x object

    def cannot_use(self, gripper: str) -> str | None: ...  # why the cell cannot be built with it

    async def load_object(self, name: str) -> bool: ...  # False: no robust grasps, nothing changed

    async def build(self, gripper: str, name: str) -> bool: ...  # the cell again, with another gripper

    async def show(self, pose: int) -> tuple[float, tuple[float, float]]: ...  # the object's turn (deg) and place

    async def begin_run(self) -> None: ...  # before cycles: a run in the results database, if none is open

    def guard(self) -> list[str]: ...  # put back what was moved in the stage; what was put back; raises if it cannot run

    async def run_cycle(self, pose: int, label: str, on_log: Callable[[str], None]) -> dict: ...


def plain(value: Any) -> Any:
    """A numpy scalar as a Python number, so the events are JSON."""
    return value.item() if hasattr(value, "item") else value


def result_text(pose: int, r: dict) -> str:
    """The results list's line for a cycle: start pose, outcome, place error, how far off the found
    pose was (with the camera), the grasp, and why it failed."""
    text = f"{pose:>4}   {r['outcome']}"
    if r.get("place_error_m") is not None:
        text += f", {r['place_error_m'] * 1000:.1f} mm from the target"
    if r.get("est_centre_mm") is not None:
        text += f"; found {r['est_centre_mm']:.1f} mm / {r['est_rot_deg']:.1f} deg off"
    if r.get("grasp_index") is not None:
        text += f"; grasp {r['grasp_index']}"
    if r["outcome"] != "placed" and r.get("detail"):
        text += f"  ({r['detail']})"
    return text


class CellControl:
    """The window's state and actions. labels: gripper -> the name shown; fixed_gripper: the gripper
    cannot change (with ROS 2, whose locator's camera mount comes from its launch file)."""

    def __init__(self, cell: Cell, menu: dict[str, Offer], labels: dict[str, str], perception: str,
                 pose: int = 0, fixed_gripper: bool | None = None):
        self.cell, self.menu, self.labels, self.perception = cell, menu, labels, perception
        self.fixed_gripper = perception == "ros" if fixed_gripper is None else fixed_gripper
        self.pose = pose
        self.busy = self.stopping = False
        self.status = READY
        self.lock = asyncio.Lock()
        self._listeners: list[Listener] = []
        self._tasks: set[asyncio.Task] = set()

    # --- listeners -------------------------------------------------------------------------------
    def subscribe(self, listener: Listener) -> Callable[[], None]:
        """Call `listener` with every event from now on. Returns what unsubscribes it."""
        self._listeners.append(listener)

        def unsubscribe() -> None:
            if listener in self._listeners:
                self._listeners.remove(listener)
        return unsubscribe

    def _emit(self, event: dict) -> None:
        for listener in list(self._listeners):
            try:
                listener(event)
            except Exception:  # noqa: BLE001  one view's fault must not stop the cell or the other views
                traceback.print_exc()

    def _changed(self) -> None:
        self._emit({"type": "state", "state": self.state()})

    def _say(self, text: str) -> None:
        self.status = text
        self._changed()

    # --- state -----------------------------------------------------------------------------------
    def info(self) -> str:
        text = f"The object's pose from {MODES[self.perception]}."
        if self.fixed_gripper:
            text += " The gripper stays as started: the locator's camera mount comes from its launch file."
        return text

    def gripper_text(self, gripper: str) -> str:
        text = f"{self.labels[gripper]}   {len(self.menu[gripper][0])} objects"
        if self.cell.cannot_use(gripper):
            text += "   (no hand-eye calibration)"
        return text

    def state(self) -> dict:
        """What the controls show: the choices, what is chosen, whether the controls are on, and the
        cycle's latest step (`status`)."""
        gripper = self.cell.gripper
        objects, counts, tested = self.menu[gripper]
        return {
            "perception": self.perception,
            "info": self.info(),
            "gripper": gripper,
            "object": self.cell.object_name,
            "pose": self.pose,
            "busy": self.busy,
            "stopping": self.stopping,
            "fixed_gripper": self.fixed_gripper,
            "status": self.status,
            "grippers": [{"name": g, "text": self.gripper_text(g), "usable": self.cell.cannot_use(g) is None}
                         for g in self.menu],
            "objects": [{"name": n, "text": f"{n}   {counts[n]} grasps{'   (batch-tested)' if n in tested else ''}"}
                        for n in objects],
        }

    # --- actions ---------------------------------------------------------------------------------
    def choose_gripper(self, gripper: str) -> str | None:
        if self.busy:
            return BUSY
        if gripper not in self.menu:
            return f"{gripper} is not offered"
        if self.fixed_gripper and gripper != self.cell.gripper:
            return "the gripper stays as started: the locator's camera mount comes from its launch file"
        self._spawn(self._switch_gripper(gripper))
        return None

    def choose_object(self, name: str) -> str | None:
        if self.busy:
            return BUSY
        if name not in self.menu[self.cell.gripper][0]:
            return f"{name} is not offered with the {self.labels[self.cell.gripper]}"
        self._spawn(self._switch(name))
        return None

    def set_pose(self, pose: int) -> str | None:
        """The next cycle's start pose (seeded: the same number, the same start), shown at once."""
        if self.busy:
            return BUSY
        if int(pose) == self.pose:
            return None
        self.pose = int(pose)
        self._changed()
        self._spawn(self._locked(self._show))
        return None

    def run(self, n: int) -> str | None:
        """Run n cycles, from the start pose on (each the next pose)."""
        if self.busy:
            return BUSY
        if int(n) < 1:
            return "run at least one cycle"
        self.busy, self.stopping = True, False
        self._changed()
        self._spawn(self._run(int(n)))
        return None

    def stop(self) -> str | None:
        """Stop after the cycle that runs."""
        if not self.busy:
            return "nothing runs"
        self.stopping = True
        self._say(self.status + "  (stopping after this cycle)")
        return None

    async def start(self) -> None:
        """Once the cell is built and the views subscribed: the first heading, and the first start."""
        self._heading()
        self._changed()
        await self._locked(self._show)

    async def settled(self) -> None:
        """Until every action taken has done its work."""
        while self._tasks:
            await asyncio.gather(*list(self._tasks), return_exceptions=True)

    # --- the work --------------------------------------------------------------------------------
    def _spawn(self, work: Coroutine) -> None:
        task = asyncio.ensure_future(work)
        self._tasks.add(task)
        task.add_done_callback(self._done)

    def _done(self, task: asyncio.Task) -> None:
        self._tasks.discard(task)
        e = None if task.cancelled() else task.exception()
        if e is not None:
            traceback.print_exception(e)
            self.busy = False
            self._say(f"error: {e}")

    async def _locked(self, work: Callable[[], Coroutine]) -> None:
        async with self.lock:
            await work()

    async def _switch(self, name: str) -> None:
        async with self.lock:
            if name == self.cell.object_name:
                return
            if await self.cell.load_object(name):
                self._heading()
            await self._show()

    async def _switch_gripper(self, gripper: str) -> None:
        """Build the cell again with this gripper, keeping the object if the gripper has robust
        grasps for it (otherwise the first in its list)."""
        async with self.lock:
            cell = self.cell
            if gripper == cell.gripper:
                return
            problem = cell.cannot_use(gripper)
            if problem:
                self._say(problem)
                return
            objects = self.menu[gripper][0]
            name = cell.object_name if cell.object_name in objects else objects[0]
            self.busy = True
            self._say(f"Building the cell with the {self.labels[gripper]}...")
            ok = False
            try:
                ok = await cell.build(gripper, name)
            except Exception as e:  # noqa: BLE001  shown in the views; the window stays usable
                traceback.print_exc()
                self.status = f"error: {e}"
            finally:
                self.busy = False
                self._changed()
            if ok:
                self._heading()
                await self._show()

    async def _show(self) -> None:
        """Show the cell as the next cycle starts: the object at its start pose, the arm at home."""
        cell, i = self.cell, self.pose
        yaw, pick = await cell.show(i)
        self._say(f"{self.labels[cell.gripper]} x {cell.object_name} ({cell.grasp_count} robust grasps), start pose {i}: "
                  f"turned {yaw:+.0f} deg at ({pick[0]:.3f}, {pick[1]:.3f}). Ready.")

    async def _run(self, n: int) -> None:
        cell = self.cell
        failed = False
        async with self.lock:
            try:
                await cell.begin_run()
                for k in range(n):
                    i = self.pose
                    for text in cell.guard():
                        self._emit({"type": "warning", "text": text})
                    r = await cell.run_cycle(i, f"start pose {i}" + (f" ({k + 1}/{n})" if n > 1 else ""), self._on_log)
                    self._emit(self._result(i, r))
                    self.pose = i + 1
                    self._changed()
                    if self.stopping:
                        break
            except Exception as e:  # noqa: BLE001  shown in the views; the window stays usable
                traceback.print_exc()
                self.status, failed = f"error: {e}", True
            finally:
                self.busy = False
                self._changed()
            if not failed:
                await self._show()

    def _on_log(self, line: str) -> None:
        self._say(line.replace("[cell] ", ""))

    def _heading(self) -> None:
        gripper, name = self.cell.gripper, self.cell.object_name
        self._emit({"type": "heading", "gripper": gripper, "object": name, "text": f"{self.labels[gripper]} x {name}"})

    def _result(self, pose: int, r: dict) -> dict:
        return {"type": "result", "pose": pose, "ok": r["outcome"] == "placed", "text": result_text(pose, r),
                **{k: plain(r.get(k)) for k in RESULT_FIELDS}}
