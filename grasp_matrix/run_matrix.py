"""Run the whole grasp matrix: every configured gripper against every configured object.

Each pair runs in its own Isaac Sim process (run_pair.py), so a crash or memory build-up in
one pair cannot affect the others. --workers runs several pairs at once (about 2 GB of RAM each). Pairs already finished with the same settings are
skipped, so the run can be stopped with Ctrl+C and started again later. Also loads the
object catalog and gripper calibration into the database, so it is self-contained.

Run with the project venv or any Python 3.10+ (standard library only):
    .venv/Scripts/python.exe grasp_matrix/run_matrix.py
    .venv/Scripts/python.exe grasp_matrix/run_matrix.py --objects mug_A07RDPW843 --grippers robotiq_2f_85
"""

import argparse
import concurrent.futures
import csv
import datetime
import json
import os
import pathlib
import subprocess
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import gm_config  # noqa: E402
import gm_db  # noqa: E402

ROOT = gm_config.ROOT


def load_reference_tables(conn) -> None:
    with (ROOT / "data/object_catalog.csv").open(encoding="utf-8") as f:
        for r in csv.DictReader(f):
            conn.execute(
                "INSERT OR REPLACE INTO objects VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                (r["asset_name"], r["category"], r["description"], float(r["min_dim_m"]), float(r["mid_dim_m"]),
                 float(r["max_dim_m"]), float(r["mass_kg"]), r["wikidata_qcode"], r["license"],
                 r["package_version"], r["source_dataset"], r["source_revision"]),
            )
    for name, g in gm_config.load_grippers().items():
        conn.execute(
            "INSERT OR REPLACE INTO grippers VALUES (?, ?, ?, ?, ?, ?, ?)",
            (name, g["joint_type"], g["max_aperture_m"], g["pads_touch_position"], g["fingertip_standoff_m"],
             g["usd"], json.dumps(g["variants"])),
        )
    conn.commit()


def run_one(isaac_python: pathlib.Path, gripper: str, obj: str, args, log_path: pathlib.Path) -> tuple[int, float]:
    """Run one pair in its own Isaac Sim process; return its exit code (-1 on timeout) and duration."""
    t0 = time.time()
    cmd = ["cmd", "/c", str(isaac_python), str(ROOT / "grasp_matrix/run_pair.py"),
           "--gripper", gripper, "--object", obj, "--db", args.db,
           "--candidates", str(args.candidates), "--orientations", str(args.orientations),
           "--conditions", args.conditions, "--seed", str(args.seed)]
    if args.full_app:
        cmd.append("--full-app")
    env = dict(os.environ, OMNI_KIT_ACCEPT_EULA="YES")
    with log_path.open("w", encoding="utf-8", errors="replace") as log:
        proc = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
        try:
            code = proc.wait(timeout=args.timeout_min * 60)
        except subprocess.TimeoutExpired:
            subprocess.run(["taskkill", "/T", "/F", "/PID", str(proc.pid)], capture_output=True)
            code = -1
    return code, time.time() - t0


def pair_status(conn, gripper: str, obj: str, digest: str, code: int) -> str:
    run = conn.execute(
        "SELECT run_id, status, num_sampled, notes FROM runs WHERE gripper=? AND object=? AND settings_hash=? "
        "ORDER BY run_id DESC LIMIT 1", (gripper, obj, digest),
    ).fetchone()
    if run is not None and run["status"] == "done":
        outcomes = dict(conn.execute(
            "SELECT outcome, COUNT(*) FROM trials WHERE run_id=? AND condition='baseline' GROUP BY outcome",
            (run["run_id"],),
        ).fetchall())
        return f"sampled {run['num_sampled']}, baseline {outcomes}"
    note = run["notes"] if run is not None and run["notes"] else f"exit code {code}, see log"
    return f"FAILED ({note})"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--isaac-sim", default=os.environ.get("ISAAC_SIM_DIR", r"C:\issacsim-6-2"))
    parser.add_argument("--db", default="results/grasp_matrix.sqlite")
    parser.add_argument("--grippers", help="comma-separated subset (default: all configured)")
    parser.add_argument("--objects", help="comma-separated subset (default: all configured)")
    parser.add_argument("--candidates", type=int, default=200)
    parser.add_argument("--orientations", type=int, default=4)
    parser.add_argument("--conditions", default="baseline,low_friction,heavy")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--timeout-min", type=float, default=30)
    parser.add_argument("--workers", type=int, default=1, help="pairs to run at once (about 2 GB of RAM each)")
    parser.add_argument("--full-app", action="store_true",
                        help="run pairs in the full Isaac Sim app instead of the physics-only one (more memory)")
    parser.add_argument("--dry-run", action="store_true", help="list what would run and exit")
    args = parser.parse_args()

    calibrated = gm_config.load_grippers()
    grippers = args.grippers.split(",") if args.grippers else list(calibrated)
    missing = [g for g in grippers if g not in calibrated]
    if missing:
        sys.exit(f"not configured and calibrated: {missing}. Add them to grasp_matrix/config/grippers.json and run "
                 f"tools/calibrate_grippers.py --grippers {','.join(missing)}")
    objects = args.objects.split(",") if args.objects else gm_config.load_objects()
    conditions = args.conditions.split(",")
    isaac_python = pathlib.Path(args.isaac_sim) / "python.bat"
    (ROOT / args.db).parent.mkdir(parents=True, exist_ok=True)
    conn = gm_db.connect(str(ROOT / args.db))
    load_reference_tables(conn)

    pairs = [(g, o) for o in objects for g in grippers]
    todo = []
    for g, o in pairs:
        digest = gm_db.settings_hash(gm_config.run_settings(g, args.candidates, args.orientations, conditions, args.seed))
        if not gm_db.pair_done(conn, g, o, digest):
            todo.append((g, o, digest))
    print(f"{len(pairs)} pairs, {len(pairs) - len(todo)} already done, {len(todo)} to run")
    if args.dry_run:
        for g, o, _ in todo:
            print(f"  {g} x {o}")
        return

    log_dir = ROOT / "results/logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    t_start = time.time()
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = {
            pool.submit(run_one, isaac_python, g, o, args, log_dir / f"{g}__{o}.log"): (g, o, digest)
            for g, o, digest in todo
        }
        try:
            for i, fut in enumerate(concurrent.futures.as_completed(futures), 1):
                g, o, digest = futures[fut]
                code, seconds = fut.result()
                status = pair_status(conn, g, o, digest, code)
                elapsed = time.time() - t_start
                eta = datetime.timedelta(seconds=int(elapsed / i * (len(todo) - i)))
                print(f"[{i}/{len(todo)}] {g} x {o}: {status} ({seconds:.0f} s, ETA {eta})", flush=True)
        except KeyboardInterrupt:
            pool.shutdown(wait=True, cancel_futures=True)  # running pairs get the Ctrl+C too and stop
            raise

    print(f"finished in {datetime.timedelta(seconds=int(time.time() - t_start))}")


if __name__ == "__main__":
    main()
