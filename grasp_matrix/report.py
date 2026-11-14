"""Summarise the grasp matrix: which gripper handles which object, and how robustly.

Writes results/summary_pairs.csv (one row per gripper x object) and prints a Markdown report
(also saved to results/summary.md). Standard library only.

Rates are over grasps that reached the physics test (passed the pre-filters):
  held rate    held at baseline / tested
  robust rate  held under every condition / tested
  held_firm    held at baseline with the object slipping at most FIRM_SLIP_M in total (CSV only)
A gripper is recommended for an object by robust count, then held count.

    .venv/Scripts/python.exe grasp_matrix/report.py
"""

import argparse
import csv
import pathlib
import sqlite3
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import gm_config  # noqa: E402

ROOT = gm_config.ROOT
OUTCOMES = ["held", "dropped", "blocked", "missed", "collided"]
FIRM_SLIP_M = 0.005


def pair_rows(conn: sqlite3.Connection) -> list[dict]:
    runs = conn.execute(
        "SELECT r.run_id, r.gripper, r.object, r.num_sampled, r.seconds, r.settings, "
        "       o.category, o.min_dim_m, o.mass_kg, g.max_aperture_m "
        "FROM runs r JOIN objects o ON o.asset_name = r.object JOIN grippers g ON g.name = r.gripper "
        "WHERE r.status = 'done' AND r.run_id IN (SELECT MAX(run_id) FROM runs WHERE status='done' GROUP BY gripper, object)"
    ).fetchall()
    rows = []
    for r in runs:
        filters = dict(conn.execute(
            "SELECT COALESCE(filter, 'passed'), COUNT(*) FROM grasps WHERE run_id=? GROUP BY 1", (r["run_id"],)
        ).fetchall())
        base = dict(conn.execute(
            "SELECT outcome, COUNT(*) FROM trials WHERE run_id=? AND condition='baseline' GROUP BY outcome", (r["run_id"],)
        ).fetchall())
        conditions = [c for (c,) in conn.execute("SELECT DISTINCT condition FROM trials WHERE run_id=?", (r["run_id"],))]
        robust = conn.execute(
            "SELECT COUNT(*) FROM (SELECT grasp_index FROM trials WHERE run_id=? GROUP BY grasp_index "
            "HAVING COUNT(*) = ? AND SUM(outcome = 'held') = COUNT(*))", (r["run_id"], len(conditions)),
        ).fetchone()[0] if conditions else 0
        firm = conn.execute(
            "SELECT COUNT(*) FROM trials WHERE run_id=? AND condition='baseline' AND outcome='held' AND lift_move_m <= ?",
            (r["run_id"], FIRM_SLIP_M),
        ).fetchone()[0]
        tested = filters.get("passed", 0)
        row = {
            "gripper": r["gripper"], "object": r["object"], "category": r["category"],
            "min_dim_m": r["min_dim_m"], "mass_kg": r["mass_kg"],
            "fits_by_extent": r["min_dim_m"] <= r["max_aperture_m"],
            "sampled": r["num_sampled"], "from_below": filters.get("from_below", 0),
            "hits_table": filters.get("hits_table", 0), "tested": tested,
            **{o: base.get(o, 0) for o in OUTCOMES},
            "held_firm": firm,
            "robust": robust,
            "held_rate": round(base.get("held", 0) / tested, 3) if tested else 0.0,
            "robust_rate": round(robust / tested, 3) if tested else 0.0,
            "seconds": r["seconds"],
        }
        rows.append(row)
    return rows


def gripper_labels(grippers: list[str]) -> dict[str, str]:
    """Column labels: the config's "label", else the name without its vendor prefix."""
    cfg = gm_config.gripper_config()
    return {g: cfg.get(g, {}).get("label") or g.replace("robotiq_", "").replace("_", "-").upper() for g in grippers}


def calibration_notes(grippers: list[str]) -> dict[str, list[str]]:
    """Warnings from each gripper's calibration checks (tools/calibrate_grippers.py)."""
    calib = gm_config.load_calibration()
    return {g: calib.get(g, {}).get("checks", {}).get("warnings", []) for g in grippers}


def markdown(rows: list[dict], grippers: list[str], objects: list[str],
             survival: dict[tuple[str, str], tuple[int, int]] | None = None,
             notes: dict[str, list[str]] | None = None) -> str:
    by = {(r["gripper"], r["object"]): r for r in rows}
    short = gripper_labels(grippers)
    lines = ["# Grasp matrix summary", "",
             "Cells show **robust / held / tested**: grasps held under every condition, held at baseline, "
             "and grasps that reached the physics test. `·` = not run yet.", ""]
    lines.append("| Object | Smallest extent | Mass | " + " | ".join(short[g] for g in grippers) + " | Best |")
    lines.append("|---|---|---|" + "---|" * len(grippers) + "---|")
    for o in objects:
        cells, best, best_key = [], "", (-1, -1)
        first = next((by[(g, o)] for g in grippers if (g, o) in by), None)
        for g in grippers:
            r = by.get((g, o))
            if r is None:
                cells.append("·")
                continue
            cells.append(f"{r['robust']} / {r['held']} / {r['tested']}" + ("" if r["fits_by_extent"] else " †"))
            key = (r["robust"], r["held"])
            if key > best_key and r["held"] > 0:
                best, best_key = short[g], key
        size = f"{first['min_dim_m'] * 1000:.0f} mm" if first else ""
        mass = f"{first['mass_kg']:.2f} kg" if first else ""
        category = first["category"] if first else o
        lines.append(f"| {category} | {size} | {mass} | " + " | ".join(cells) + f" | {best or '—'} |")
    lines += ["", "† the object's smallest extent is wider than this gripper's measured opening.", "",
              "## Per gripper", "",
              "| Gripper | Pairs | Tested | Held | Robust | Held rate | Robust rate | Objects with a robust grasp |",
              "|---|---|---|---|---|---|---|---|"]
    for g in grippers:
        rs = [r for r in rows if r["gripper"] == g]
        tested = sum(r["tested"] for r in rs)
        held = sum(r["held"] for r in rs)
        robust = sum(r["robust"] for r in rs)
        covered = sum(1 for r in rs if r["robust"] > 0)
        lines.append(f"| {short[g]} | {len(rs)} | {tested} | {held} | {robust} | "
                     f"{held / tested if tested else 0:.0%} | {robust / tested if tested else 0:.0%} | {covered} / {len(rs)} |")
    lines += ["", "## Outcome mix at baseline", "",
              "| Gripper | " + " | ".join(OUTCOMES) + " |", "|---|" + "---|" * len(OUTCOMES)]
    for g in grippers:
        rs = [r for r in rows if r["gripper"] == g]
        lines.append(f"| {short[g]} | " + " | ".join(str(sum(r[o] for r in rs)) for o in OUTCOMES) + " |")
    if survival:
        conditions = sorted({c for (_, c) in survival})
        lines += ["", "## Robustness", "",
                  "Of the grasps held at baseline, the share still held under each condition.", "",
                  "| Gripper | Held at baseline | " + " | ".join(conditions) + " |",
                  "|---|---|" + "---|" * len(conditions)]
        for g in grippers:
            base = next((n for (gg, _), (n, _) in survival.items() if gg == g), 0)
            cells = [f"{survival[(g, c)][1] / base:.0%}" if base and (g, c) in survival else "·" for c in conditions]
            lines.append(f"| {short[g]} | {base} | " + " | ".join(cells) + " |")
    if notes and any(notes.values()):
        lines += ["", "## Gripper notes", "", "Warnings from the calibration checks (tools/calibrate_grippers.py).", ""]
        lines += [f"- **{short[g]}**: {w}" for g in grippers for w in notes.get(g, [])]
    return "\n".join(lines) + "\n"


def condition_survival(conn: sqlite3.Connection) -> dict[tuple[str, str], tuple[int, int]]:
    """(gripper, condition) -> (grasps held at baseline, of those still held under the condition)."""
    rows = conn.execute(
        "SELECT r.gripper, t.condition, COUNT(*), SUM(t.outcome = 'held') FROM trials t JOIN runs r USING (run_id) "
        "WHERE r.status = 'done' AND r.run_id IN (SELECT MAX(run_id) FROM runs WHERE status='done' GROUP BY gripper, object) "
        "AND t.condition != 'baseline' GROUP BY 1, 2"
    ).fetchall()
    return {(g, c): (n, held) for g, c, n, held in rows}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--db", default="results/grasp_matrix.sqlite")
    args = parser.parse_args()
    conn = sqlite3.connect(ROOT / args.db)
    conn.row_factory = sqlite3.Row
    rows = pair_rows(conn)
    if not rows:
        print("no finished runs yet")
        return
    out = ROOT / "results"
    with (out / "summary_pairs.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    grippers = list(gm_config.load_grippers())
    text = markdown(rows, grippers, gm_config.load_objects(), condition_survival(conn), calibration_notes(grippers))
    (out / "summary.md").write_text(text, encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()
