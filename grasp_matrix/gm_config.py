"""Grasp-matrix configuration and protocol constants. Standard library only, so the batch
driver (project venv) and the Isaac Sim scripts compute identical run settings."""

import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
PROTOCOL_VERSION = "v3"

# Physics stepping (seconds and step counts at DT).
DT = 1.0 / 60.0
SETTLE_OBJECT_STEPS = 90
PLACE_STEPS = 15
CLOSE_STEPS = 60
LIFT_STEPS = 60

# Outcome thresholds and pre-filters.
PLACEMENT_TOLERANCE_M = 0.01
# Held = the object has come to rest in the fingers: it moved less than LATE_MOVE_TOLERANCE_M
# during the second half of the lift and sagged less than MAX_SAG_M overall.
LATE_MOVE_TOLERANCE_M = 0.005
MAX_SAG_M = 0.05
# Every grasp gets the lift test. A failed one is labelled missed if the fingers closed this far
# towards where the pads meet, or blocked if they stopped more than BLOCKED_MARGIN_M wider than
# the object's width at the grasp (something else, such as the table, was in the way).
MISSED_FRACTION = 0.95
BLOCKED_MARGIN_M = 0.01
FROM_BELOW_MAX_APPROACH_Z = 0.2  # approach vectors pointing up more than ~11 degrees are rejected
TABLE_CLEARANCE_M = 0.002
# A grasp whose pads would cut into the table is backed off along its approach axis, by at most
# this fraction of the pad length, so the original contact point stays on the pad.
MAX_LIFT_PAD_FRACTION = 0.5
APERTURE_MARGIN = 0.95  # sample only features at most 95% of the open gap

CONDITIONS = {
    "baseline": {"friction_scale": 1.0, "mass_scale": 1.0},
    "low_friction": {"friction_scale": 0.5, "mass_scale": 1.0},
    "heavy": {"friction_scale": 1.0, "mass_scale": 2.0},
}


GRIPPER_CONFIG = ROOT / "grasp_matrix/config/grippers.json"
CALIBRATION = ROOT / "data/gripper_calibration.json"
# Keys in config/grippers.json that document a gripper but do not change any result. They are
# left out of load_grippers(), so adding or editing them never invalidates stored runs.
DOC_KEYS = ("label", "datasheet", "source", "notes")
_noted_uncalibrated: set[str] = set()


def gripper_config() -> dict:
    """config/grippers.json as written, documentation keys included."""
    cfg = json.loads(GRIPPER_CONFIG.read_text())
    cfg.pop("_comment", None)
    return cfg


def load_calibration() -> dict:
    return json.loads(CALIBRATION.read_text()) if CALIBRATION.exists() else {}


def is_fingertip(body_name: str, gripper: dict) -> bool:
    """Whether a rigid body is one of the gripper's two fingertip pads: the bodies named in the
    optional "fingertips" config key, otherwise any body with "fingertip" in its name."""
    names = gripper.get("fingertips")
    return body_name in names if names else "fingertip" in body_name


def load_grippers(include_uncalibrated: bool = False) -> dict:
    """Static gripper settings (config/grippers.json) merged with the calibrated geometry.

    A gripper that is configured but not calibrated yet is left out (with a note on stderr),
    unless include_uncalibrated is set, in which case it has no calibrated keys."""
    cfg = gripper_config()
    calib = load_calibration()
    out = {}
    for name, g in cfg.items():
        g = {k: v for k, v in g.items() if k not in DOC_KEYS}
        c = calib.get(name)
        if c is None:
            if include_uncalibrated:
                out[name] = g
            elif name not in _noted_uncalibrated:
                _noted_uncalibrated.add(name)
                print(f"note: gripper {name} is configured but not calibrated; run tools/calibrate_grippers.py "
                      f"--grippers {name}", file=sys.stderr)
            continue
        # Calibrations before open/closed detection always opened at the lower joint limit.
        open_target = c.get("open_target", c["lower_limit"])
        close_target = c.get("close_target", c["upper_limit"])
        closing = 1.0 if close_target >= open_target else -1.0
        g.update({
            "open_target": open_target,
            "close_target": close_target,
            "pads_touch_position": c["pads_touch_position"],
            "max_aperture_m": c["max_aperture_m"],
            "fingertip_standoff_m": round((c["pad_height_open_m"] + c["pad_height_closed_m"]) / 2, 4),
            # [joint position, pad gap, pad height] from fully open to where the pads meet. The 2F
            # fingertips move along the approach axis as they close; the table check needs that.
            "closing_curve": [[p["joint"], p["gap_m"], p["pad_height_m"]] for p in c["curve"]
                              if closing * (p["joint"] - c["pads_touch_position"]) <= 1e-6],
        })
        if "open_state" in c:
            # Only for assets not stored with the fingers open: every joint's open position and
            # the pads' open boxes in the gripper frame (see gm_core.apply_open_state).
            g["open_state"] = c["open_state"]
        out[name] = g
    return out


def load_objects() -> list[str]:
    cfg = json.loads((ROOT / "grasp_matrix/config/objects.json").read_text())
    return [o["name"] for o in cfg["objects"]]


def object_usd(name: str) -> pathlib.Path:
    return ROOT / f"assets/allsides/{name}/simready_usd/{name}.usd"


def run_settings(gripper: str, candidates: int, orientations: int, conditions: list[str], seed: int) -> dict:
    """Everything that determines a pair's results; its hash decides whether a stored run is reusable."""
    return {
        "protocol": PROTOCOL_VERSION,
        "steps": {"dt": DT, "settle": SETTLE_OBJECT_STEPS, "place": PLACE_STEPS, "close": CLOSE_STEPS, "lift": LIFT_STEPS},
        "thresholds": {
            "placement_m": PLACEMENT_TOLERANCE_M, "late_move_m": LATE_MOVE_TOLERANCE_M, "max_sag_m": MAX_SAG_M,
            "missed_fraction": MISSED_FRACTION, "blocked_margin_m": BLOCKED_MARGIN_M,
            "from_below_max_approach_z": FROM_BELOW_MAX_APPROACH_Z, "table_clearance_m": TABLE_CLEARANCE_M,
            "max_lift_pad_fraction": MAX_LIFT_PAD_FRACTION,
            "aperture_margin": APERTURE_MARGIN,
        },
        "candidates": candidates,
        "orientations": orientations,
        "conditions": {c: CONDITIONS[c] for c in conditions},
        "seed": seed,
        "gripper": load_grippers()[gripper],
    }
