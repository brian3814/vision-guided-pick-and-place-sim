"""The window in a browser: what a web page needs of the window's controller (cell_control.py).

The page shows Kit's viewport as a WebRTC stream and draws the panel itself, next to it. It reaches
the controller through the stream's data channel (livestream.py hooks this module to Kit's
messaging extension), with four kinds of JSON message:

    page -> Kit    cell:request    {action, request_id, ...}            one per user action
    Kit -> page    cell:response   {ok, error, state, request_id, ...}  whether the action was taken
    Kit -> page    cell:event      {event}    each controller event but state: heading, result, warning
    Kit -> page    cell:state      {state}    the controller's state, whenever it changed

Actions, as the panel's widgets call them: hello, gripper {name}, object {name}, pose {pose},
run {n}, stop. An action starts the work and is answered at once; what the work does comes as
events and state, as it does to the omni.ui panel.

The page has no Kit menus, so it also switches the viewport's camera, as Isaac Sim's camera menu
does: action camera {path}, taken at any time, even while cycles run, since it only changes what
the stream shows (viewport_cameras.py). The state carries the cameras, under `view`: which there
are, and which one the viewport looks through.

A page that connects, or reloads, says `hello`. The answer holds the events so far (`log`), so the
results list comes back as it was, and the state. Every event is stamped with the time it was sent
(`at`, seconds since the epoch).

The page's client library does not put the messaging extension's message id on the wire, and the
extension can then hand a message over twice. So the page puts its own `request_id` in each
request; the answer carries it back, and a request id already answered gets the same answer again
without running the action a second time.

Nothing here imports Kit: the tests drive it with the real controller, a fake cell and a fake
viewport.
"""

from __future__ import annotations

import json
import time
from collections import OrderedDict, deque
from collections.abc import Callable
from typing import Any

PROTOCOL = 2  # 2: the state carries the viewport's cameras (`view`), and the camera action
REQUEST = "cell:request"
RESPONSE = "cell:response"
EVENT = "cell:event"
STATE = "cell:state"
NO_VIEW = {"camera": None, "cameras": []}  # the state's `view` without a viewport
ANSWERS_KEPT = 64  # answers kept by request id, for a request handed over twice
LOG_KEPT = 2000  # events a page that connects later gets (about 400 cycles' results)

Send = Callable[[str, dict], None]


def jsonable(value: Any) -> Any:
    """`value` as JSON gives it back: anything JSON cannot hold, as text."""
    return json.loads(json.dumps(value, default=str))


class WebBridge:
    """The controller's side of the web page. `send(event_type, payload)` puts a message on the stream;
    it can be set later (livestream.connect does). `clock` stamps the events. `view`: the viewport's
    cameras (viewport_cameras.ViewportCameras)."""

    def __init__(self, control, send: Send | None = None, clock: Callable[[], float] = time.time, view=None):
        self._c = control
        self._view = view
        self.send: Send = send or (lambda _type, _payload: None)
        self._clock = clock
        self._log: deque[dict] = deque(maxlen=LOG_KEPT)
        self._state: dict | None = None  # the state the page has
        self._answered: OrderedDict[str, dict] = OrderedDict()
        self._unsubscribe = control.subscribe(self._on_event)

    def close(self) -> None:
        """Stop listening to the controller."""
        self._unsubscribe()
        self.send = lambda _type, _payload: None

    # --- from the controller -----------------------------------------------------------------------
    def _on_event(self, event: dict) -> None:
        if event["type"] == "state":
            self._push_state(event["state"])
            return
        event = jsonable({**event, "at": self._clock()})
        self._log.append(event)
        self.send(EVENT, {"event": event})

    def _with_view(self, state: dict) -> dict:
        return jsonable({**state, "view": NO_VIEW if self._view is None else self._view.state()})

    def _push_state(self, state: dict) -> None:
        state = self._with_view(state)
        if state != self._state:
            self._state = state
            self.send(STATE, {"state": state})

    # --- from the page ---------------------------------------------------------------------------
    def handle(self, action: str, args: dict | None = None) -> dict:
        """Run the page's request; the answer, with the state after it."""
        args = dict(args or {})
        request_id = args.pop("request_id", None)
        if request_id is not None and request_id in self._answered:
            return self._answered[request_id]
        answer = self._run(action, args)
        if request_id is not None:
            answer["request_id"] = request_id
            self._answered[request_id] = answer
            while len(self._answered) > ANSWERS_KEPT:
                self._answered.popitem(last=False)
        return answer

    def _run(self, action: str, args: dict) -> dict:
        method = self._ACTIONS.get(action)
        if method is None:
            result = {"ok": False, "error": f"unknown action: {action!r}"}
        else:
            try:
                result = method(self, **args)
            except (TypeError, ValueError, KeyError) as e:  # what the page sent does not fit the action
                result = {"ok": False, "error": f"{action}: {e}"}
        self._state = self._with_view(self._c.state())
        return {**result, "state": self._state}

    @staticmethod
    def _taken(problem: str | None) -> dict:
        return {"ok": True} if problem is None else {"ok": False, "error": problem}

    def _hello(self) -> dict:
        return {"ok": True, "protocol": PROTOCOL, "log": list(self._log)}

    def _gripper(self, name: str) -> dict:
        return self._taken(self._c.choose_gripper(str(name)))

    def _object(self, name: str) -> dict:
        return self._taken(self._c.choose_object(str(name)))

    def _pose(self, pose: int) -> dict:
        return self._taken(self._c.set_pose(int(pose)))

    def _run_cycles(self, n: int) -> dict:
        return self._taken(self._c.run(int(n)))

    def _stop(self) -> dict:
        return self._taken(self._c.stop())

    def _camera(self, path: str) -> dict:
        if self._view is None:
            return self._taken("no viewport to switch the camera of")
        return self._taken(self._view.set_camera(str(path)))

    _ACTIONS = {
        "hello": _hello,
        "gripper": _gripper,
        "object": _object,
        "pose": _pose,
        "run": _run_cycles,
        "stop": _stop,
        "camera": _camera,
    }
