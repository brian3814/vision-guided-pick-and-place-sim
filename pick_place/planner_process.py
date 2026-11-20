"""The planning process: cuMotion's trajectory optimizer, in a process of its own, away from Kit's
CUDA work (why: cumotion_planner.py). planner.PlannerProcess starts it with Isaac Sim's Python:

    <python> -I planner_process.py [--backend module:Class] <path>...

<path>s go on sys.path (where Kit found numpy and cuMotion). It listens on a local port for the key in
PLANNER_KEY, says "READY <port>" on stdout, and serves one connection: requests (op, args, dropped)
answered with ("ok", value) or ("error", traceback). `dropped` lists the planners and plans Kit no
longer holds. Arrays travel as lists of floats, which keeps every digit. It ends with the connection.

Nothing here imports Kit; the tests serve a fake planner over a pipe.
"""

from __future__ import annotations

import importlib
import itertools
import os
import pathlib
import sys
import traceback
from collections.abc import Callable
from multiprocessing.connection import Connection, Listener
from typing import Any

PLAN_KINDS = ("goalset", "pose", "line", "joints")


def _array(value):
    import numpy as np

    return np.asarray(value, dtype=float)


def serve(conn: Connection, make_planner: Callable[[list, pathlib.Path], Any]) -> None:
    """Answer requests on `conn` until it closes or Kit says close. `make_planner(tool_spheres,
    robot_dir)` builds a planner with cumotion_planner.CuMotionPlanner's methods."""
    planners: dict[int, Any] = {}
    plans: dict[int, Any] = {}
    ids = itertools.count(1)

    def new_planner(robot_dir: str, spheres: list) -> int:
        i = next(ids)
        planners[i] = make_planner([(_array(c), float(r)) for c, r in spheres], pathlib.Path(robot_dir))
        return i

    def add_cuboid(i: int, name: str, size, centre, pose) -> None:
        planners[i].add_cuboid(name, _array(size), centre=None if centre is None else _array(centre),
                               pose=None if pose is None else _array(pose))

    def set_enabled(i: int, name: str, enabled: bool) -> None:
        planners[i].set_enabled(name, bool(enabled))

    def tool_pose(i: int, q) -> list:
        return planners[i].tool_pose(_array(q)).tolist()

    def plan(i: int, kind: str, q0, target, *extra):
        if kind not in PLAN_KINDS:
            raise ValueError(f"no plan kind {kind!r}")
        goal = [_array(p) for p in target] if kind == "goalset" else _array(target)
        p = getattr(planners[i], f"plan_{kind}")(_array(q0), goal, *extra)
        if p is None:
            return None
        k = next(ids)
        plans[k] = p
        return k, float(p.duration), int(p.target_index), p.final().tolist()

    def at(k: int, t: float) -> tuple[list, list]:
        pos, vel = plans[k].at(float(t))
        return pos.tolist(), vel.tolist()

    handlers = {"planner": new_planner, "add_cuboid": add_cuboid, "set_enabled": set_enabled,
                "tool_pose": tool_pose, "plan": plan, "at": at, "close": lambda: None}
    while True:
        try:
            op, args, dropped = conn.recv()
        except (EOFError, OSError):
            return
        for kind, k in dropped:
            (planners if kind == "planner" else plans).pop(k, None)
        try:
            if op not in handlers:
                raise ValueError(f"no request {op!r}")
            conn.send(("ok", handlers[op](*args)))
        except Exception:  # noqa: BLE001  Kit gets the traceback; the process keeps serving
            conn.send(("error", traceback.format_exc()))
        if op == "close":
            return


def main() -> None:
    args = sys.argv[1:]
    backend = "cumotion_planner:CuMotionPlanner"
    if args[:1] == ["--backend"]:
        backend, args = args[1], args[2:]
    sys.path[:0] = [str(pathlib.Path(__file__).resolve().parent), *args]
    module, name = backend.split(":")
    make_planner = getattr(importlib.import_module(module), name)
    key = bytes.fromhex(os.environ.pop("PLANNER_KEY"))
    with Listener(("127.0.0.1", 0), authkey=key) as listener:
        print(f"READY {listener.address[1]}", flush=True)
        with listener.accept() as conn:
            serve(conn, make_planner)


if __name__ == "__main__":
    main()
