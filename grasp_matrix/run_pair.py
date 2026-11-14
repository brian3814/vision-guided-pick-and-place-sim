"""Evaluate one gripper x object pair and store the results in the grasp-matrix database.

Every grasp that passes the pre-filters is tested under the baseline condition; grasps that
hold are then re-tested under the other conditions (lower friction, heavier object) to see
how robust they are. Usually launched by run_matrix.py, but it can run alone:

    C:/issacsim-6-2/python.bat grasp_matrix/run_pair.py --gripper robotiq_2f_85 --object mug_A07RDPW843
"""

import argparse
import asyncio
import datetime
import json
import pathlib
import sys
import time

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--gripper", required=True)
parser.add_argument("--object", required=True)
parser.add_argument("--db", default="results/grasp_matrix.sqlite")
parser.add_argument("--candidates", type=int, default=200, help="grasp poses to sample")
parser.add_argument("--orientations", type=int, default=4, help="approach directions around each grasp axis")
parser.add_argument("--conditions", default="baseline,low_friction,heavy")
parser.add_argument("--seed", type=int, default=7)
parser.add_argument("--full-app", action="store_true",
                    help="load the full Isaac Sim app instead of the physics-only one (uses more memory)")
args, _ = parser.parse_known_args()

PHYSICS_ONLY_APP = pathlib.Path(__file__).resolve().parent / "apps/grasp_physics.kit"
app = SimulationApp({"headless": True}, experience="" if args.full_app else str(PHYSICS_ONLY_APP))

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import gm_config  # noqa: E402
import gm_core  # noqa: E402
import gm_db  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402

CONTROL_MIN_DROP_M = 0.5


def log(msg: str) -> None:
    print(f"[pair] {msg}", flush=True)


def isaac_sim_version() -> str:
    version_file = pathlib.Path(sys.executable).resolve().parents[2] / "VERSION"
    return version_file.read_text().strip() if version_file.exists() else "unknown"


async def main() -> None:
    conditions = args.conditions.split(",")
    assert conditions[0] == "baseline", "the first condition must be baseline"
    settings = gm_config.run_settings(args.gripper, args.candidates, args.orientations, conditions, args.seed)
    gripper = settings["gripper"]
    digest = gm_db.settings_hash(settings)
    db_path = gm_config.ROOT / args.db
    db_path.parent.mkdir(parents=True, exist_ok=True)
    conn = gm_db.connect(str(db_path))
    gm_db.clear_unfinished(conn, args.gripper, args.object, digest)
    if gm_db.pair_done(conn, args.gripper, args.object, digest):
        log(f"{args.gripper} x {args.object} already done with these settings; nothing to do")
        return

    t_start = time.perf_counter()
    cur = conn.execute(
        "INSERT INTO runs (gripper, object, settings_hash, settings, status, started, isaac_sim_version) "
        "VALUES (?, ?, ?, ?, 'running', ?, ?)",
        (args.gripper, args.object, digest, json.dumps(settings), datetime.datetime.now().isoformat(timespec="seconds"),
         isaac_sim_version()),
    )
    run_id = cur.lastrowid
    conn.commit()

    SimulationManager.switch_physics_engine("physx")
    engine = SimulationManager.get_active_physics_engine()
    if engine != "physx":
        raise RuntimeError(f"expected PhysX, got {engine}")

    scene = gm_core.GraspScene(args.gripper, gripper, args.object)
    step = gm_core.physics_steps
    drop = await scene.settle_object(step)
    log(f"object settled after dropping {drop:.3f} m")

    control_move = await scene.control_trial(step)
    control_ok = control_move > CONTROL_MIN_DROP_M
    log(f"control: object moved {control_move:.3f} m with the table off -> {'ok' if control_ok else 'FAILED'}")
    if not control_ok:
        conn.execute("UPDATE runs SET status='failed', control_ok=0, notes=? WHERE run_id=?",
                     (f"control trial moved only {control_move:.3f} m", run_id))
        conn.commit()
        return

    poses = scene.sample_grasps(args.candidates, args.orientations, args.seed)
    log(f"sampled {len(poses)} grasp poses")
    widths = scene.grasp_widths(poses)
    log(f"object width along the closing line: {sum(w is None for w in widths)} of {len(widths)} unknown")
    to_test = []
    for idx, ((sampled_loc, quat), width) in enumerate(zip(poses, widths)):
        f = scene.pre_filter(sampled_loc, quat, width)
        loc = f["location"]  # backed off from the table if needed
        conn.execute(
            "INSERT INTO grasps VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
            (run_id, idx, loc[0], loc[1], loc[2], quat.GetReal(), *quat.GetImaginary(),
             f["approach_z"], f["lowest_pad_z"], f["closed_pad_z"], f["width_m"], f["lift_m"], f["filter"]),
        )
        if f["filter"] is None:
            to_test.append((idx, loc, quat, width))
    conn.commit()
    log(f"{len(to_test)} grasps pass the pre-filters")

    held = []
    for condition in conditions:
        candidates = to_test if condition == "baseline" else [g for g in to_test if g[0] in held]
        counts: dict[str, int] = {}
        for idx, loc, quat, width in candidates:
            r = await scene.run_trial(loc, quat, width, condition, step)
            conn.execute(
                "INSERT INTO trials VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                (run_id, idx, condition, r["outcome"], r["finger_position"], r["finger_fraction"], r["finger_gap_m"],
                 r["placement_move_m"], r["lift_move_m"], r["late_move_m"], r["seconds"]),
            )
            conn.commit()  # per trial, so parallel pairs (run_matrix --workers) never wait long for the write lock
            counts[r["outcome"]] = counts.get(r["outcome"], 0) + 1
            if condition == "baseline" and r["outcome"] == "held":
                held.append(idx)
        log(f"{condition}: {len(candidates)} trials -> {counts}")

    seconds = time.perf_counter() - t_start
    conn.execute(
        "UPDATE runs SET status='done', finished=?, seconds=?, control_ok=1, num_sampled=? WHERE run_id=?",
        (datetime.datetime.now().isoformat(timespec="seconds"), round(seconds, 1), len(poses), run_id),
    )
    conn.commit()
    log(f"done in {seconds:.0f} s (run_id {run_id})")


task = asyncio.ensure_future(main())
while not task.done():
    app.update()
if task.exception():
    raise task.exception()
app.close()
