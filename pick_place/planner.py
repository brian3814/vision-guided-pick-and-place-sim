"""Motion planning for the cell: cuMotion's trajectory optimizer, run in a process of its own.

Kit's process must not run cuMotion: cumotion_planner.py says why, and
docs/web-stream-cumotion-crash.md has the whole story. planner_process.py runs
cumotion_planner.CuMotionPlanner in a process started with Isaac Sim's Python. Planner and Plan here
are its stand-ins in Kit, with the planner's interface as it was: obstacles, the tool's pose,
planning, and the plans to follow. Every number comes from cuMotion in that process, as it did in
Kit. A plan's joint positions and velocities at a time are evaluated there too (each Plan.at is a
round trip of a fraction of a millisecond), so results are the same to the last digit.

One planning process serves every Planner of a session: it is started with the first one, ended at
exit, and started again if it ended. Pending runs a planner call on a thread of its own, so that Kit
can keep updating, and the stream keep getting frames, while the planning process works.

Inside Isaac Sim, make planners after the isaacsim.robot_motion.cumotion extension is on: it puts
cuMotion on sys.path and ships the robot's configuration.

Frames: the robot base is the world origin (cell.py puts it there). Poses are 4x4 numpy matrices
acting on column vectors.
"""

from __future__ import annotations

import atexit
import importlib.util
import os
import pathlib
import secrets
import subprocess
import sys
import threading
import time
from collections import deque
from multiprocessing.connection import Client, Connection
from typing import Any, Callable

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent


def _plain(value) -> list:
    """An array as nested lists of floats: what travels to the planning process, every digit kept."""
    return np.asarray(value, dtype=float).tolist()


def _found_under(package: str) -> str:
    """The sys.path entry a top-level package is found under, without importing it."""
    spec = importlib.util.find_spec(package)
    if spec is None or not spec.submodule_search_locations:
        raise RuntimeError(f"{package} is not importable here (inside Isaac Sim, enable isaacsim.robot_motion.cumotion first)")
    return str(pathlib.Path(list(spec.submodule_search_locations)[0]).parent)


def supported_robot_dir(name: str = "ur10") -> pathlib.Path:
    """The robot configurations shipped with the isaacsim.robot_motion.cumotion extension."""
    spec = importlib.util.find_spec("isaacsim.robot_motion.cumotion")
    for place in (spec.submodule_search_locations or []) if spec else []:
        found = pathlib.Path(place).resolve() / "robot_configurations" / name
        if found.is_dir():
            return found
    raise RuntimeError(f"no {name} robot configuration: enable the isaacsim.robot_motion.cumotion extension first")


class PlannerProcessError(RuntimeError):
    """The planning process could not be started, or it ended."""


def _relay(stream) -> None:
    for line in stream:
        print(f"[planner] {line.rstrip()}", flush=True)


class PlannerProcess:
    """A planning process (planner_process.py) and the connection to it. Requests are answered in
    order, one at a time, from whichever thread asks (see Pending). A planner or plan dropped in Kit,
    on any thread, is released there with the next request."""

    def __init__(self, conn: Connection, proc: subprocess.Popen | None = None, relay: threading.Thread | None = None):
        self.conn, self.proc, self._relay = conn, proc, relay
        self.closed = False
        self._dropped: deque[tuple[str, int]] = deque()  # appended and taken without a lock: a drop can come
        self._lock = threading.Lock()                     # from the garbage collector in the middle of a call

    @classmethod
    def start(cls, backend: str | None = None, paths: list[str] | None = None) -> PlannerProcess:
        """Start a planning process with this process's Python (Isaac Sim's), finding numpy and
        cuMotion where this process finds them. `backend`, `paths`: for the tests."""
        if paths is None:
            paths = [_found_under("numpy"), _found_under("cumotion")]
        key = secrets.token_bytes(32)
        command = [sys.executable, "-I", str(HERE / "planner_process.py")]
        if backend:
            command += ["--backend", backend]
        proc = subprocess.Popen(command + paths, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                env=dict(os.environ, PLANNER_KEY=key.hex()), text=True, encoding="utf-8", errors="replace")
        said: list[str] = []
        assert proc.stdout is not None
        for line in proc.stdout:
            if line.startswith("READY "):
                port = int(line.split()[1])
                break
            said.append(line.rstrip())
        else:
            proc.wait()
            proc.stdout.close()
            raise PlannerProcessError(f"the planning process did not start (exit code {proc.returncode}):\n" + "\n".join(said[-20:]))
        for line in said:
            print(f"[planner] {line}", flush=True)
        relay = threading.Thread(target=_relay, args=(proc.stdout,), daemon=True)
        relay.start()
        return cls(Client(("127.0.0.1", port), authkey=key), proc, relay)

    @property
    def alive(self) -> bool:
        return not self.closed and (self.proc is None or self.proc.poll() is None)

    def call(self, op: str, *args):
        """Ask the planning process; its answer, or its error raised here with its traceback."""
        with self._lock:
            if self.closed:
                raise PlannerProcessError("the planning process was closed")
            dropped = []
            while self._dropped:
                dropped.append(self._dropped.popleft())
            try:
                self.conn.send((op, args, dropped))
                status, value = self.conn.recv()
            except (EOFError, OSError) as e:
                self.closed = True
                code = self.proc.poll() if self.proc else None
                raise PlannerProcessError(f"the planning process ended (exit code {code}); "
                                          "what it said is in the log as [planner] lines") from e
        if status == "error":
            raise RuntimeError(f"in the planning process: {value.strip().splitlines()[-1]}\n{value}")
        return value

    def drop(self, kind: str, k: int) -> None:
        """Release a planner or plan with the next request."""
        if not self.closed:
            self._dropped.append((kind, k))

    def close(self) -> None:
        if self.closed:
            return
        try:
            self.call("close")
        except Exception:  # noqa: BLE001  already gone: nothing to close
            pass
        self.closed = True
        self.conn.close()
        if self.proc is not None:
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
            if self._relay is not None:
                self._relay.join(timeout=2)  # what the process said last, into the log
            if self.proc.stdout is not None:
                self.proc.stdout.close()


class Pending:
    """A planner call (e.g. a lambda around Planner.plan_line) running on a thread of its own, so that
    Kit can go on updating while the planning process works. The thread only waits for the answer:
    the planning process plans exactly as it does for a call made directly."""

    def __init__(self, call: Callable[..., Any], *args):
        self.wall_s = 0.0  # how long the call took, once done
        self._value: Any = None
        self._error: BaseException | None = None
        self._finished = threading.Event()
        threading.Thread(target=self._run, args=(call, args), name="planner call", daemon=True).start()

    def _run(self, call: Callable[..., Any], args: tuple) -> None:
        t0 = time.perf_counter()
        try:
            self._value = call(*args)
        except BaseException as e:  # noqa: BLE001  raised again by result(), on the thread that waits
            self._error = e
        self.wall_s = time.perf_counter() - t0
        self._finished.set()

    def done(self) -> bool:
        return self._finished.is_set()

    def wait(self, timeout: float) -> bool:
        """Wait until the call is done, for timeout seconds at most. Whether it is done."""
        return self._finished.wait(timeout)

    def result(self) -> Any:
        """The call's answer, or its error raised here. Waits for it if it is not done."""
        self._finished.wait()
        if self._error is not None:
            raise self._error
        return self._value


_shared: PlannerProcess | None = None


def planner_process() -> PlannerProcess:
    """The session's planning process: started on first use, and again if it ended."""
    global _shared
    if _shared is None or not _shared.alive:
        _shared = PlannerProcess.start()
        atexit.register(_shared.close)
    return _shared


class Plan:
    """A planned motion, held in the planning process: its duration, its final joint positions,
    and for a goalset which target was chosen."""

    def __init__(self, process: PlannerProcess, k: int, duration: float, target_index: int, final: list):
        self._process, self._k = process, k
        self.duration = duration
        self.target_index = target_index
        self._final = np.array(final, dtype=float)

    def final(self) -> np.ndarray:
        return self._final.copy()

    def at(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        """Joint positions and velocities at time t from the start (clamped to the end)."""
        pos, vel = self._process.call("at", self._k, float(t))
        return np.array(pos, dtype=float), np.array(vel, dtype=float)

    def __del__(self):
        self._process.drop("plan", self._k)


class Planner:
    """UR10 + gripper spheres in a world of cuboid obstacles (cumotion_planner.CuMotionPlanner, in
    the planning process)."""

    def __init__(self, tool_spheres: list[tuple[np.ndarray, float]], robot_dir: pathlib.Path | None = None,
                 process: PlannerProcess | None = None):
        self._process = process or planner_process()
        robot_dir = robot_dir or supported_robot_dir("ur10")
        self._id = self._process.call("planner", str(robot_dir), [(_plain(c), float(r)) for c, r in tool_spheres])

    def __del__(self):
        if hasattr(self, "_id"):
            self._process.drop("planner", self._id)

    # --- world ---------------------------------------------------------------------------
    def add_cuboid(self, name: str, size, centre=None, pose: np.ndarray | None = None) -> None:
        """Add a box obstacle, or move/resize the one with this name. Give its centre, or its pose."""
        self._process.call("add_cuboid", self._id, name, _plain(size),
                           None if centre is None else _plain(centre), None if pose is None else _plain(pose))

    def set_enabled(self, name: str, enabled: bool) -> None:
        self._process.call("set_enabled", self._id, name, bool(enabled))

    # --- kinematics ----------------------------------------------------------------------
    def tool_pose(self, q) -> np.ndarray:
        return np.array(self._process.call("tool_pose", self._id, _plain(q)), dtype=float)

    # --- planning ------------------------------------------------------------------------
    def _plan(self, kind: str, q0, target, *extra) -> Plan | None:
        made = self._process.call("plan", self._id, kind, _plain(q0), target, *extra)
        return None if made is None else Plan(self._process, *made)

    def plan_goalset(self, q0, poses: list[np.ndarray]) -> Plan | None:
        """Collision-free motion to whichever of the tool poses suits best."""
        return self._plan("goalset", q0, [_plain(p) for p in poses])

    def plan_pose(self, q0, pose: np.ndarray) -> Plan | None:
        return self._plan("pose", q0, _plain(pose))

    def plan_line(self, q0, pose: np.ndarray, deviation_m: float = 0.005) -> Plan | None:
        """Straight-line tool motion to the pose, orientation held constant on the way."""
        return self._plan("line", q0, _plain(pose), float(deviation_m))

    def plan_joints(self, q0, q1) -> Plan | None:
        return self._plan("joints", q0, _plain(q1))
