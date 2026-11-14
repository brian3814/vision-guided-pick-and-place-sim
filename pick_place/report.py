"""Summarise the pick-and-place cell's results: for each gripper x object x where the object's pose
came from (the simulation, step 3; or the wrist camera, step 4), its latest finished run's cycles.
Writes results/pick_place_summary.md and prints it.

    .venv/Scripts/python.exe pick_place/report.py
"""

import argparse
import json
import pathlib
import sqlite3
import statistics
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "grasp_matrix"))
import gm_config  # noqa: E402

FAILURES = ["not_found", "unreachable", "plan_failed", "tracking", "missed", "dropped_lift", "dropped_transport", "misplaced"]


def median(values: list) -> float | None:
    values = [v for v in values if v is not None]
    return statistics.median(values) if values else None


def maximum(values: list) -> float | None:
    values = [v for v in values if v is not None]
    return max(values) if values else None


def fmt(value: float | None, scale: float = 1.0, digits: int = 1) -> str:
    return "–" if value is None else f"{value * scale:.{digits}f}"


def mode_of(settings: dict) -> str:
    """Where the object's pose came from, as a label."""
    if settings.get("perception") == "blind":
        return "nowhere: blind, nominal pose"
    if settings.get("perception", "truth") in ("camera", "ros"):
        noise = settings.get("depth_noise", "1")
        return ("camera" if settings["perception"] == "camera" else "camera via ROS 2") + ("" if noise == "1" else f", depth {'perfect' if noise == 'perfect' else 'noise x' + noise}") \
            + (", as-built mount" if settings.get("handeye", {}).get("method", "").startswith("none") else "")
    return "simulation" + (", from the look pose" if settings.get("look_first") else "") \
        + ("" if settings.get("slip_correction", True) else ", no slip correction")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--db", default="results/pick_place.sqlite")
    parser.add_argument("--out", default="results/pick_place_summary.md")
    args = parser.parse_args()

    conn = sqlite3.connect(ROOT / args.db)
    conn.row_factory = sqlite3.Row
    finished = conn.execute("SELECT r.*, (SELECT COUNT(*) FROM cell_cycles c WHERE c.run_id = r.run_id) AS n_cycles "
                            "FROM cell_runs r WHERE finished IS NOT NULL ORDER BY run_id").fetchall()
    latest: dict[tuple, sqlite3.Row] = {}
    for run in finished:  # later runs replace earlier ones of the same gripper x object x mode, unless they are
        key = (run["gripper"], run["object"], mode_of(json.loads(run["settings"])))  # shorter (a run to watch in the window)
        if key not in latest or run["n_cycles"] >= latest[key]["n_cycles"]:
            latest[key] = run
    labels = {k: v.get("label", k) for k, v in gm_config.gripper_config().items()}
    has_column = {row[1] for row in conn.execute("PRAGMA table_info(cell_cycles)")}

    lines = ["# Pick-and-place cell results", "",
             "A UR10 with the gripper on its flange picks the object off the table and puts it on the place spot,",
             "using the object's robust grasps from the grasp matrix (held under every test condition). Each cycle",
             "starts with the object turned and shifted (seeded: the same start poses in every run with the same",
             "seed); the planner picks whichever robust grasp it can reach. *Pose from*: the simulation tells the",
             "robot where the object is (step 3), or the wrist camera finds it (step 4), or nothing does (*blind*:",
             "the robot takes it to be at its nominal pose, not turned or shifted; the baseline for the camera).",
             "Latest finished run per gripper x object x pose source (a shorter run does not replace a longer one).", "",
             "Columns: *grasps used* is how many different robust grasps the planner chose; *ruled out* counts",
             "grasps it chose first but dropped because a later move of the cycle could not be planned; *side",
             "grasps* approached at less than 30 degrees below horizontal (approach z above -0.5); *slip* is how far",
             "the object moved in the fingers between closing and lowering. Medians are over placed cycles.", "",
             "| Gripper | Object | Pose from | Robust grasps | Placed | Failures | Grasps used | Ruled out | Side grasps "
             "| Cycle s | Place error mm | Slip mm | Slip deg |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    totals: dict[str, dict[str, int]] = {}
    failure_rows, cycles_by_key = [], {}
    for key in sorted(latest, key=lambda k: (k[2] != "simulation", k[2], k[0], k[1])):
        run = latest[key]
        cycles = conn.execute("SELECT * FROM cell_cycles WHERE run_id=? ORDER BY cycle", (run["run_id"],)).fetchall()
        if not cycles:
            continue
        cycles_by_key[key] = (run, cycles)
        placed = [c for c in cycles if c["outcome"] == "placed"]
        fails: dict[str, int] = {}
        for c in cycles:
            if c["outcome"] != "placed":
                fails[c["outcome"]] = fails.get(c["outcome"], 0) + 1
                failure_rows.append((run["object"], key[2], c))
        used = {c["grasp_index"] for c in cycles if c["grasp_index"] is not None}
        side = sum(1 for c in cycles if c["approach_z"] is not None and c["approach_z"] > -0.5)
        lines.append(
            f"| {labels.get(run['gripper'], run['gripper'])} | {run['object']} | {key[2]} | {run['robust_grasps']} "
            f"| {len(placed)}/{len(cycles)} | {', '.join(f'{k} {v}' for k, v in sorted(fails.items())) or '–'} "
            f"| {len(used)} | {sum(c['rejected_grasps'] or 0 for c in cycles)} | {side} "
            f"| {fmt(median([c['cycle_s'] for c in placed]))} "
            f"| {fmt(median([c['place_error_m'] for c in placed]), 1000)} | {fmt(median([c['slip_m'] for c in placed]), 1000)} "
            f"| {fmt(median([c['slip_deg'] for c in placed]))} |")
        t = totals.setdefault(key[2], {"cycles": 0, "placed": 0})
        t["cycles"] += len(cycles)
        t["placed"] += len(placed)
    if totals:
        lines.append("")
        lines += [f"Overall, pose from {mode}: {t['placed']}/{t['cycles']} cycles placed ({100 * t['placed'] / t['cycles']:.0f}%)."
                  for mode, t in totals.items()]

    camera_keys = [k for k in cycles_by_key if k[2].startswith("camera")]
    if camera_keys and "est_centre_mm" in has_column:
        lines += ["", "## The camera's pose estimates", "",
                  "Against the true pose at the moment of the image. *Centre*: distance between the object model's centroid",
                  "as estimated and as true. *ADD-S*: mean distance from each estimated model point to the nearest true one",
                  "(near zero for a pose that a symmetric object cannot be told apart from). *Look s*: simulated time from",
                  "home to the image. *Compute s*: wall time of perception.locate().", "",
                  "| Object | Pose from | Found | Centre mm (median / max) | Rotation deg (median / max) | ADD-S mm (median / max) "
                  "| Look s | Compute s |",
                  "|---|---|---|---|---|---|---|---|"]
        for key in sorted(camera_keys, key=lambda k: (k[1], k[2])):
            _, cycles = cycles_by_key[key]
            found = [c for c in cycles if c["est_centre_mm"] is not None]
            lines.append(f"| {key[1]} | {key[2]} | {len(found)}/{len(cycles)} "
                         f"| {fmt(median([c['est_centre_mm'] for c in found]))} / {fmt(maximum([c['est_centre_mm'] for c in found]))} "
                         f"| {fmt(median([c['est_rot_deg'] for c in found]))} / {fmt(maximum([c['est_rot_deg'] for c in found]))} "
                         f"| {fmt(median([c['est_adds_mm'] for c in found]))} / {fmt(maximum([c['est_adds_mm'] for c in found]))} "
                         f"| {fmt(median([c['look_s'] for c in found]))} | {fmt(median([c['perception_wall_s'] for c in found]))} |")

        # the same start poses with the pose from the camera and from elsewhere (the simulation, or blind)
        pairs = []
        for key, sim in [(k, s) for k in camera_keys for s in cycles_by_key
                         if s[:2] == k[:2] and not s[2].startswith("camera")]:
            (crun, ccycles), (srun, scycles) = cycles_by_key[key], cycles_by_key[sim]
            if json.loads(crun["settings"]).get("seed") != json.loads(srun["settings"]).get("seed"):
                continue
            s_by_cycle = {c["cycle"]: c["outcome"] for c in scycles}
            both = [(s_by_cycle[c["cycle"]], c["outcome"]) for c in ccycles if c["cycle"] in s_by_cycle]
            counts = {"both": sum(1 for s, c in both if s == "placed" and c == "placed"),
                      "sim": sum(1 for s, c in both if s == "placed" and c != "placed"),
                      "cam": sum(1 for s, c in both if s != "placed" and c == "placed"),
                      "neither": sum(1 for s, c in both if s != "placed" and c != "placed")}
            pairs.append((key, sim[2], len(both), counts))
        if pairs:
            lines += ["", "## Same start poses, pose from the camera vs from elsewhere", "",
                      "Cycle by cycle: each cycle index has the same start pose in every run with the same seed. *Blind*: the",
                      "robot takes the object to be at its nominal pose (not turned or shifted), what it would do without the",
                      "camera.", "",
                      "| Object | Camera run | Compared with | Cycles | Placed by both | Only with the other "
                      "| Only with the camera's | Neither |",
                      "|---|---|---|---|---|---|---|---|"]
            for key, sim_mode, n, cnt in sorted(pairs, key=lambda p: (p[1], p[0][1])):
                lines.append(f"| {key[1]} | {key[2]} | {sim_mode} | {n} | {cnt['both']} | {cnt['sim']} | {cnt['cam']} | {cnt['neither']} |")

    if failure_rows:
        lines += ["", "## Failed cycles", "",
                  "| Object | Pose from | Cycle | Grasp | Approach z | Outcome | Detail | Slip mm | Slip deg | Place error mm | Tilt deg |",
                  "|---|---|---|---|---|---|---|---|---|---|---|"]
        for obj, mode, c in failure_rows:
            lines.append(f"| {obj} | {mode} | {c['cycle']} | {c['grasp_index'] if c['grasp_index'] is not None else '–'} "
                         f"| {fmt(c['approach_z'], 1, 2)} | {c['outcome']} | {c['detail'] or ''} "
                         f"| {fmt(c['slip_m'], 1000)} | {fmt(c['slip_deg'])} | {fmt(c['place_error_m'], 1000)} "
                         f"| {fmt(c['tilt_deg'])} |")
    if cycles_by_key:
        lines += ["", "## Settings (latest run of each pose source)", ""]
        for mode in sorted({k[2] for k in cycles_by_key}):
            run = max((cycles_by_key[k][0] for k in cycles_by_key if k[2] == mode), key=lambda r: r["run_id"])
            lines += [f"{mode}:", "", "```json", json.dumps(json.loads(run["settings"]), indent=1), "```", ""]

    text = "\n".join(lines) + "\n"
    out = ROOT / args.out
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text, encoding="utf-8")
    sys.stdout.reconfigure(encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()
