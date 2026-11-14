"""SQLite storage for the grasp matrix. Standard library only, so both Isaac Sim's Python
and the project venv can import it.

Tables:
  objects   one row per catalogued object (from data/object_catalog.csv)
  grippers  one row per gripper, with its calibrated geometry
  runs      one row per gripper x object evaluation, with its settings and status
  grasps    every sampled grasp pose, the object's width along its closing line, the lowest
            fingertip-pad point open and closed, and the filter that rejected it (if any)
  trials    the physics result of each evaluated grasp under each test condition, including
            the pad gap the fingers stopped at (compare with grasps.width_m)
"""

import hashlib
import json
import sqlite3

SCHEMA = """
CREATE TABLE IF NOT EXISTS objects (
    asset_name TEXT PRIMARY KEY, category TEXT, description TEXT,
    min_dim_m REAL, mid_dim_m REAL, max_dim_m REAL, mass_kg REAL,
    wikidata_qcode TEXT, license TEXT, package_version TEXT,
    source_dataset TEXT, source_revision TEXT
);
CREATE TABLE IF NOT EXISTS grippers (
    name TEXT PRIMARY KEY, joint_type TEXT, max_aperture_m REAL, pads_touch_position REAL,
    fingertip_standoff_m REAL, usd TEXT, variants TEXT
);
CREATE TABLE IF NOT EXISTS runs (
    run_id INTEGER PRIMARY KEY AUTOINCREMENT,
    gripper TEXT NOT NULL, object TEXT NOT NULL, settings_hash TEXT NOT NULL, settings TEXT NOT NULL,
    status TEXT NOT NULL, started TEXT, finished TEXT, seconds REAL,
    isaac_sim_version TEXT, control_ok INTEGER, num_sampled INTEGER, notes TEXT
);
CREATE TABLE IF NOT EXISTS grasps (
    run_id INTEGER NOT NULL, grasp_index INTEGER NOT NULL,
    pos_x REAL, pos_y REAL, pos_z REAL, qw REAL, qx REAL, qy REAL, qz REAL,
    approach_z REAL, lowest_pad_z REAL, closed_pad_z REAL, width_m REAL, lift_m REAL, filter TEXT,
    PRIMARY KEY (run_id, grasp_index)
);
CREATE TABLE IF NOT EXISTS trials (
    run_id INTEGER NOT NULL, grasp_index INTEGER NOT NULL, condition TEXT NOT NULL,
    outcome TEXT NOT NULL, finger_position REAL, finger_fraction REAL, finger_gap_m REAL,
    placement_move_m REAL, lift_move_m REAL, late_move_m REAL, seconds REAL,
    PRIMARY KEY (run_id, grasp_index, condition)
);
CREATE INDEX IF NOT EXISTS runs_pair ON runs (gripper, object, settings_hash, status);
"""


def connect(path: str) -> sqlite3.Connection:
    conn = sqlite3.connect(path, timeout=60)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA journal_mode=WAL")  # readers never block the writer, for parallel pairs
    conn.executescript(SCHEMA)
    return conn


def settings_hash(settings: dict) -> str:
    """Stable short hash of the run settings; a change in settings makes a pair run again."""
    return hashlib.sha256(json.dumps(settings, sort_keys=True).encode()).hexdigest()[:12]


def pair_done(conn: sqlite3.Connection, gripper: str, obj: str, digest: str) -> bool:
    row = conn.execute(
        "SELECT 1 FROM runs WHERE gripper=? AND object=? AND settings_hash=? AND status='done' LIMIT 1",
        (gripper, obj, digest),
    ).fetchone()
    return row is not None


def clear_unfinished(conn: sqlite3.Connection, gripper: str, obj: str, digest: str) -> None:
    """Delete rows left by an interrupted or failed run of this pair, so it can run cleanly."""
    ids = [r[0] for r in conn.execute(
        "SELECT run_id FROM runs WHERE gripper=? AND object=? AND settings_hash=? AND status!='done'",
        (gripper, obj, digest),
    )]
    for run_id in ids:
        conn.execute("DELETE FROM trials WHERE run_id=?", (run_id,))
        conn.execute("DELETE FROM grasps WHERE run_id=?", (run_id,))
        conn.execute("DELETE FROM runs WHERE run_id=?", (run_id,))
    conn.commit()
