"""SQLite storage for the pick-and-place cell, and the grasps it takes from the grasp matrix.
Standard library only, so both Isaac Sim's Python and the project venv can import it.

Tables (results/pick_place.sqlite):
  cell_runs    one row per run of run_cell.py: gripper, object, settings, which matrix run the
               grasps came from and how many robust grasps it offered
  cell_cycles  one row per pick-and-place cycle: the object's start pose, the grasp the planner
               chose, the outcome and where it failed, times, placement error and in-hand slip;
               with the camera (step 4), the pose estimate's error and the time it took
"""

import sqlite3

SCHEMA = """
CREATE TABLE IF NOT EXISTS cell_runs (
    run_id INTEGER PRIMARY KEY AUTOINCREMENT,
    gripper TEXT NOT NULL, object TEXT NOT NULL, settings TEXT NOT NULL,
    grasp_run_id INTEGER, robust_grasps INTEGER,
    started TEXT, finished TEXT, isaac_sim_version TEXT
);
CREATE TABLE IF NOT EXISTS cell_cycles (
    run_id INTEGER NOT NULL, cycle INTEGER NOT NULL,
    yaw_deg REAL, pick_x REAL, pick_y REAL,
    grasp_index INTEGER, approach_z REAL, rejected_grasps INTEGER,
    outcome TEXT NOT NULL, detail TEXT,
    pick_s REAL, motion_s REAL, cycle_s REAL, plan_wall_s REAL,
    finger_fraction REAL, slip_m REAL, slip_deg REAL, place_error_m REAL, tilt_deg REAL, max_tool_error_m REAL,
    PRIMARY KEY (run_id, cycle)
);
"""

# Added for the camera (step 4); NULL for cycles told the object's pose. est_*: the estimate
# against the true pose (perception.pose_errors), its observed points, fitness and free-space
# violation; look_s: simulated time from home to the image; perception_wall_s: computing the pose.
VISION_COLUMNS = ["look_s", "perception_wall_s", "est_points", "est_centre_mm", "est_rot_deg", "est_adds_mm",
                  "est_add_mm", "est_fitness", "est_violation",
                  "push_m"]  # how far the approach moved the object before the fingers closed (all modes)
CYCLE_COLUMNS = ["yaw_deg", "pick_x", "pick_y", "grasp_index", "approach_z", "rejected_grasps", "outcome", "detail", "pick_s", "motion_s", "cycle_s",
                 "plan_wall_s", "finger_fraction", "slip_m", "slip_deg", "place_error_m", "tilt_deg", "max_tool_error_m",
                 *VISION_COLUMNS]


def connect(path: str) -> sqlite3.Connection:
    conn = sqlite3.connect(path, timeout=60)
    conn.row_factory = sqlite3.Row
    conn.executescript(SCHEMA)
    have = {row[1] for row in conn.execute("PRAGMA table_info(cell_cycles)")}
    for column in VISION_COLUMNS:  # databases from before step 4
        if column not in have:
            conn.execute(f"ALTER TABLE cell_cycles ADD COLUMN {column} REAL")
    conn.commit()
    return conn


def robust_grasps(matrix_db: str, gripper: str, obj: str) -> tuple[int | None, list[dict]]:
    """Grasps that held under every test condition in the latest finished grasp-matrix run of
    this pair (the run the report uses). Poses are the gripper frame in the matrix's world, with
    the object at its settled start pose; quaternions are (w, x, y, z)."""
    conn = sqlite3.connect(f"file:{matrix_db}?mode=ro", uri=True)
    row = conn.execute("SELECT max(run_id) FROM runs WHERE gripper=? AND object=? AND status='done'", (gripper, obj)).fetchone()
    run_id = row[0] if row else None
    if run_id is None:
        return None, []
    conditions = conn.execute("SELECT count(DISTINCT condition) FROM trials WHERE run_id=?", (run_id,)).fetchone()[0]
    rows = conn.execute(
        """SELECT g.grasp_index, g.pos_x, g.pos_y, g.pos_z, g.qw, g.qx, g.qy, g.qz, g.approach_z, g.width_m
           FROM grasps g WHERE g.run_id=? AND g.filter IS NULL AND
             (SELECT count(*) FROM trials t WHERE t.run_id=g.run_id AND t.grasp_index=g.grasp_index
              AND t.outcome='held') = ?
           ORDER BY g.grasp_index""",
        (run_id, conditions),
    ).fetchall()
    conn.close()
    return run_id, [{"index": r[0], "position": r[1:4], "quat_wxyz": r[4:8], "approach_z": r[8], "width_m": r[9]} for r in rows]
