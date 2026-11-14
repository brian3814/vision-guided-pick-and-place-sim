"""Measure each gripper's fingertip gap across its drive range, and check that the gripper is
configured correctly for the grasp matrix (Isaac Sim, PhysX).

The asset files alone do not say which joint value is open or where the pads meet, so this
sweeps the drive joint from its lower to its upper limit, lets the gripper settle at each
target, and measures the gap between the fingertip pads from their world bounding boxes
(PhysX writes the link poses back to USD). The end with the wider gap is "open". A negative gap
means the pads have passed through each other: fingertips do not collide with each other
inside one articulation.

Per gripper it records:
- the gap curve (joint value -> pad gap, pad height), ordered from open to closed
- open_target / close_target: the joint limits that open and close the fingers
- max_aperture_m: the gap when open
- pads_touch_position: the first joint value, going from open to closed, where the gap closes
  to <= 1 mm; a grasp whose fingers end past this point closed on nothing
- pad centre height (fingertip standoff) when open and when closed
- the result of each check below

Checks. A failed error check leaves the gripper's calibration unwritten, so it cannot be run:
- error: exactly two fingertip bodies are found
- error: the pads sit on the positive side of the approach axis (approach_axis points at them)
- error: the pads are separated along closing_axis, not along the other sideways axis
- error: the pads are centred on the approach axis (the grasp sampler assumes it)
- error: the gripper follows its base when moved, as the batch run moves it
- error: if the asset is not stored with the fingers open, the gripper starts open after a reset
  once every joint's open position is set (stored as "open_state" and applied by every trial)
- warning: the gap shrinks steadily from open to closed
- warning: the pads meet at some joint value
- warning: the measured opening is within 10% of the datasheet's (config key "datasheet")
- warning: the gripper has a physics material (otherwise friction is PhysX's default)
- warning: the fingertip's innermost collision shapes cover most of the pad (otherwise the pad
  centre, where the sampler puts the object, may be off; see contact_coverage)

Results are merged into data/gripper_calibration.json: grippers not calibrated in this run
keep their stored calibration, so adding a gripper never changes the others' settings.

Run: C:/issacsim-6-2/python.bat tools/calibrate_grippers.py [--grippers name1,name2] [--out path]
"""

import argparse
import asyncio
import json
import pathlib
import sys

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--grippers", help="comma-separated subset of config/grippers.json (default: all)")
parser.add_argument("--out", help="write here instead of data/gripper_calibration.json (to compare without changing it)")
parser.add_argument("--full-app", action="store_true", help="load the full Isaac Sim app instead of the physics-only one")
args, _ = parser.parse_known_args()

PHYSICS_ONLY_APP = pathlib.Path(__file__).resolve().parents[1] / "grasp_matrix/apps/grasp_physics.kit"
app = SimulationApp({"headless": True}, experience="" if args.full_app else str(PHYSICS_ONLY_APP))

import numpy as np  # noqa: E402
import omni.kit.app  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402

import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.replicator.grasping", True)
from isaacsim.replicator.grasping import grasping_utils, transform_utils  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "grasp_matrix"))
import gm_config  # noqa: E402
import gm_core  # noqa: E402

DT = 1.0 / 60.0
SWEEP_POINTS = 13
SETTLE_STEPS = 120
TOUCH_GAP_M = 0.001
MONOTONIC_TOLERANCE_M = 0.001
CENTRED_TOLERANCE_M = 0.002
FOLLOW_TOLERANCE_M = 0.002
DATASHEET_TOLERANCE = 0.10
STORED_OPEN_TOLERANCE_M = 0.0005  # pads this close to open in the stored pose count as stored open
START_OPEN_TOLERANCE_M = 0.001
CONTACT_DEPTH_M = 0.001  # collision shapes reaching within this of the innermost one count as touching
MIN_CONTACT_COVERAGE = 0.8  # of the pad's length along the approach axis
# Base pose for the follow check: moved and turned 90 degrees about z, so bounding boxes stay axis-aligned.
FOLLOW_LOCATION = Gf.Vec3d(0.3, -0.2, 0.6)
FOLLOW_ROTATION = Gf.Rotation(Gf.Vec3d(0, 0, 1), 90.0)


def log(msg: str) -> None:
    print(f"[calib] {msg}", flush=True)


def tip_boxes(tips: list[Usd.Prim], cfg: dict) -> list[tuple[np.ndarray, np.ndarray]]:
    """World boxes of the two pads, as the batch run defines them (gm_core.pad_boxes)."""
    return [(np.array(r.GetMin()), np.array(r.GetMax())) for r in gm_core.pad_boxes(tips, cfg)]


def pad_gap_and_height(tips: list[Usd.Prim], cfg: dict, closing: np.ndarray, approach: np.ndarray) -> tuple[float, float]:
    """Gap between the two pads along the closing axis, and their mean centre height along the
    approach axis. The gripper sits at the origin unrotated, so world axes are its own axes; both
    axes are coordinate axes, so projecting an aligned box's corners gives its exact extent."""
    spans = []
    for lo, hi in tip_boxes(tips, cfg):
        c = sorted((float(lo @ closing), float(hi @ closing)))
        spans.append((c, float((lo + hi) / 2 @ approach)))
    (left, h0), (right, h1) = sorted(spans, key=lambda s: s[0][0])
    return right[0] - left[1], (h0 + h1) / 2


def contact_coverage(tips: list[Usd.Prim], cfg: dict, boxes: list[tuple[np.ndarray, np.ndarray]],
                     closing: np.ndarray, approach: np.ndarray) -> list[str]:
    """Warn if the collision shapes that reach furthest in from a fingertip cover only part of its pad.

    The pad's centre along the approach axis is the fingertip standoff: where the sampler puts the
    object. A fingertip built from several collision shapes at different depths (a stepped finger)
    touches the object only with the shapes that reach furthest in, so a pad box over the whole
    fingertip puts the standoff in the wrong place. Which shapes should touch is a judgement (the
    Hand-E's innermost shape is a step near its base, not its pad), so this only warns and lists the
    shapes; name the right ones in "pad_shapes". With pad_shapes set, nothing is checked."""
    if cfg.get("pad_shapes"):
        return []
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"])
    tip, (lo, hi) = tips[0], boxes[0]  # the fingertips mirror each other; one is enough
    inward = -np.sign(((lo + hi) / 2) @ closing) * closing
    shapes = []
    for p in Usd.PrimRange(tip, Usd.TraverseInstanceProxies()):
        if p.HasAPI(UsdPhysics.CollisionAPI):
            r = cache.ComputeWorldBound(p).ComputeAlignedRange()
            a, b = np.array(r.GetMin()), np.array(r.GetMax())
            shapes.append((max(a @ inward, b @ inward), p.GetName(), sorted((a @ approach, b @ approach))))
    if len(shapes) < 2:
        return []
    reach = max(s[0] for s in shapes)
    touching = [s for s in shapes if s[0] >= reach - CONTACT_DEPTH_M]
    t_lo, t_hi = min(s[2][0] for s in touching), max(s[2][1] for s in touching)
    p_lo, p_hi = sorted((lo @ approach, hi @ approach))
    if t_hi - t_lo >= MIN_CONTACT_COVERAGE * (p_hi - p_lo):
        return []
    listing = ", ".join(f"{name} {(reach - face) * 1000:.1f} mm back at {span[0] * 1000:.0f}-{span[1] * 1000:.0f} mm"
                        for face, name, span in sorted(shapes, key=lambda s: s[2][0]))
    return [f"the innermost collision shape(s) of {tip.GetName()} cover only {t_lo * 1000:.0f}-{t_hi * 1000:.0f} mm "
            f"of its pad's {p_lo * 1000:.0f}-{p_hi * 1000:.0f} mm along the approach axis ({listing}), so the "
            f"fingertip standoff ({(p_lo + p_hi) / 2 * 1000:.0f} mm) may not be where the object is touched. "
            "Name the shapes that should touch the object in \"pad_shapes\"."]


async def settle_at(joints: list[Usd.Prim], target: float, steps: int = SETTLE_STEPS) -> None:
    grasping_utils.reset_physics_simulation()
    for joint in joints:
        grasping_utils.set_joint_drive_parameters(joint, target_value=float(target), target_type="position")
    await grasping_utils.simulate_physics_async(steps, DT)


async def calibrate(name: str, cfg: dict) -> tuple[dict | None, list[str], list[str]]:
    """Returns (calibration or None, errors, warnings)."""
    errors: list[str] = []
    warnings: list[str] = []
    stage = stage_utils.create_new_stage()
    stage_utils.set_stage_up_axis("Z")
    stage_utils.set_stage_units(meters_per_unit=1.0)
    UsdGeom.Xform.Define(stage, "/World")
    UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    gripper = "/World/Gripper"
    usd = ROOT / cfg["usd"]
    if not usd.exists():
        return None, [f"asset not found: {usd}"], []
    stage_utils.add_reference_to_stage(str(usd), gripper, variants=[tuple(v) for v in cfg["variants"]])
    while stage_utils.is_stage_loading():
        await omni.kit.app.get_app().next_update_async()
    gm_core.anchor_gripper(stage, gripper, cfg)  # the same set-up as the batch run
    gripper_prim = stage.GetPrimAtPath(gripper)
    all_joints = [p for p in Usd.PrimRange(gripper_prim, Usd.TraverseInstanceProxies())
                  if p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint)]

    search_root = stage.GetPrimAtPath(f"{gripper}/{cfg['articulation']}")
    tips = [p for p in Usd.PrimRange(search_root)
            if p.HasAPI(UsdPhysics.RigidBodyAPI) and gm_config.is_fingertip(p.GetName(), cfg)]
    if len(tips) != 2:
        found = [str(p.GetPath()) for p in tips]
        return None, [f"expected 2 fingertip bodies under {search_root.GetPath()}, found {len(tips)}: {found}. "
                      "Name them in the config key \"fingertips\"."], []
    stored_boxes = tip_boxes(tips, cfg)  # the pose stored in the asset, before any simulation
    joint = stage.GetPrimAtPath(f"{gripper}/{cfg['drive_joint']}")
    if not joint.IsValid():
        return None, [f"drive joint not found: {gripper}/{cfg['drive_joint']}"], []
    joints = [joint] + [stage.GetPrimAtPath(f"{gripper}/{j}") for j in cfg.get("coupled_joints", [])]
    jp = UsdPhysics.RevoluteJoint(joint) if cfg["joint_type"] == "revolute" else UsdPhysics.PrismaticJoint(joint)
    lower, upper = float(jp.GetLowerLimitAttr().Get()), float(jp.GetUpperLimitAttr().Get())
    closing, approach = np.array(cfg["closing_axis"], float), np.array(cfg["approach_axis"], float)
    side = np.cross(approach, closing)

    curve = []
    for target in np.linspace(lower, upper, SWEEP_POINTS):
        await settle_at(joints, target)
        position, _ = grasping_utils.get_joint_state(joint)
        gap, height = pad_gap_and_height(tips, cfg, closing, approach)
        curve.append({"target": round(float(target), 5), "joint": round(float(position), 5),
                      "gap_m": round(gap, 4), "pad_height_m": round(height, 4)})
    open_at_upper = curve[-1]["gap_m"] > curve[0]["gap_m"]
    if open_at_upper:
        curve.reverse()
    open_target, close_target = (upper, lower) if open_at_upper else (lower, upper)

    # Geometry checks with the fingers open (the pose every trial starts from).
    await settle_at(joints, open_target)
    boxes = tip_boxes(tips, cfg)
    centres = [(lo + hi) / 2 for lo, hi in boxes]
    warnings += contact_coverage(tips, cfg, boxes, closing, approach)
    if min(c @ approach for c in centres) <= 0:
        errors.append(f"pads are not on the positive side of approach_axis {cfg['approach_axis']}")
    apart = centres[1] - centres[0]
    if abs(apart @ closing) <= abs(apart @ side):
        errors.append(f"pads are {abs(apart @ closing) * 1000:.1f} mm apart along closing_axis {cfg['closing_axis']} "
                      f"but {abs(apart @ side) * 1000:.1f} mm along {side.round(3).tolist()}: wrong closing_axis?")
    mid = (centres[0] + centres[1]) / 2
    offset = np.array([mid @ closing, mid @ side])
    if np.abs(offset).max() > CENTRED_TOLERANCE_M:
        errors.append(f"pads are centred {(offset * 1000).round(1).tolist()} mm off the approach axis "
                      "(along closing, side); the grasp sampler assumes they are centred")

    # Open state: if the asset is not stored with the fingers open, every trial would open them during
    # placement, next to the object. Record each joint's open position and the open pad boxes, so trials
    # start open (gm_core.apply_open_state), and check that they then do.
    stored_offset = max(float(np.abs(a - b).max()) for s_box, o_box in zip(stored_boxes, boxes) for a, b in zip(s_box, o_box))
    open_state = None
    if stored_offset > STORED_OPEN_TOLERANCE_M:
        prefix = len(gripper) + 1
        open_state = {
            "joints": {str(j.GetPath())[prefix:]: round(float(grasping_utils.get_joint_state(j)[0]), 6) for j in all_joints},
            "pad_boxes_m": [[np.round(lo, 5).tolist(), np.round(hi, 5).tolist()] for lo, hi in boxes],
        }
        grasping_utils.reset_physics_simulation()
        gm_core.apply_open_state(stage, gripper, {"open_state": open_state})
        await settle_at(joints, open_target, steps=1)
        start_gap, _ = pad_gap_and_height(tips, cfg, closing, approach)
        if abs(start_gap - curve[0]["gap_m"]) > START_OPEN_TOLERANCE_M:
            errors.append(f"the asset is stored with its pads {stored_offset * 1000:.1f} mm from open, and setting the "
                          f"joints' open positions did not help: the gap one step after a reset is {start_gap * 1000:.1f} mm, "
                          f"not {curve[0]['gap_m'] * 1000:.1f} mm. Trials would open the fingers next to the object.")

    # Follow check: the batch run places the gripper by moving its root prim; the fingertips must follow.
    transform_utils.set_transform_attributes(stage.GetPrimAtPath(gripper), location=FOLLOW_LOCATION,
                                             orientation=FOLLOW_ROTATION.GetQuat())
    await settle_at(joints, open_target)
    moved = [(lo + hi) / 2 for lo, hi in tip_boxes(tips, cfg)]
    expected = [np.array(FOLLOW_ROTATION.TransformDir(Gf.Vec3d(*c.tolist())) + FOLLOW_LOCATION) for c in centres]
    follow_error = max(float(np.linalg.norm(m - e)) for m, e in zip(moved, expected))
    if follow_error > FOLLOW_TOLERANCE_M:
        errors.append(f"fingertips ended {follow_error * 1000:.1f} mm from where the moved base puts them: "
                      "the gripper does not follow its base. If its fixed root joint has no body0, name that "
                      "joint in the config key \"anchor_joint\".")

    materials = {str(p.GetPath()): UsdPhysics.MaterialAPI(p).GetStaticFrictionAttr().Get()
                 for p in Usd.PrimRange(stage.GetPrimAtPath(gripper)) if p.HasAPI(UsdPhysics.MaterialAPI)}
    if not materials:
        warnings.append("no physics material on the gripper: contacts use PhysX's default friction, and the "
                        "low_friction condition scales only the object's side, so it is milder than for other grippers")

    gaps = [p["gap_m"] for p in curve]
    if any(b > a + MONOTONIC_TOLERANCE_M for a, b in zip(gaps, gaps[1:])):
        warnings.append(f"gap does not shrink steadily from open to closed: {gaps}")
    touching = [p for p in curve if p["gap_m"] <= TOUCH_GAP_M]
    if not touching:
        warnings.append(f"pads never meet (smallest gap {min(gaps) * 1000:.1f} mm); a grasp that closes fully "
                        "is counted as missed only near the closed limit")
    touch_point = touching[0] if touching else curve[-1]
    sheet = cfg.get("datasheet", {}).get("max_opening_m")
    if sheet and abs(curve[0]["gap_m"] - sheet) > DATASHEET_TOLERANCE * sheet:
        warnings.append(f"measured opening {curve[0]['gap_m'] * 1000:.1f} mm differs from the datasheet's "
                        f"{sheet * 1000:.1f} mm by more than {DATASHEET_TOLERANCE:.0%}")

    result = {
        "joint_type": cfg["joint_type"],
        "lower_limit": lower,
        "upper_limit": upper,
        "open_target": open_target,
        "close_target": close_target,
        "max_aperture_m": curve[0]["gap_m"],
        "pads_touch_position": touch_point["joint"],
        "pad_height_open_m": curve[0]["pad_height_m"],
        "pad_height_closed_m": touch_point["pad_height_m"],
        "checks": {"pad_offset_mm": (offset * 1000).round(2).tolist(), "follow_error_mm": round(follow_error * 1000, 2),
                   "stored_pose_from_open_mm": round(stored_offset * 1000, 2), "static_friction": materials,
                   "warnings": warnings},
        "curve": curve,
    }
    if open_state is not None:
        result["open_state"] = open_state
    log(f"{name}: open at {'upper' if open_at_upper else 'lower'} limit ({open_target}), aperture "
        f"{result['max_aperture_m']:.4f} m; pads touch at joint {result['pads_touch_position']}; pad height "
        f"{result['pad_height_open_m']:.4f} -> {result['pad_height_closed_m']:.4f} m")
    log(f"{name}: gap curve {[(p['joint'], p['gap_m']) for p in curve]}")
    return (None if errors else result), errors, warnings


async def main() -> int:
    SimulationManager.switch_physics_engine("physx")
    log(f"engine: {SimulationManager.get_active_physics_engine()}")
    configured = gm_config.gripper_config()
    names = args.grippers.split(",") if args.grippers else list(configured)
    unknown = [n for n in names if n not in configured]
    if unknown:
        log(f"not in grasp_matrix/config/grippers.json: {unknown}")
        return 1
    out = pathlib.Path(args.out) if args.out else gm_config.CALIBRATION
    stored = gm_config.load_calibration()
    failed = []
    for name in names:
        result, errors, warnings = await calibrate(name, configured[name])
        for w in warnings:
            log(f"{name}: WARNING {w}")
        for e in errors:
            log(f"{name}: ERROR {e}")
        if result is None:
            failed.append(name)  # a stored calibration from an earlier run, if any, is kept as it was
        else:
            stored[name] = result
            log(f"{name}: OK" + (f" with {len(warnings)} warning(s)" if warnings else ""))
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(stored, indent=2))
    log(f"written to {out}" + (f"; NOT calibrated (fix the errors above): {failed}" if failed else ""))
    return 1 if failed else 0


task = asyncio.ensure_future(main())
while not task.done():
    app.update()
if (exc := task.exception()) is not None:
    raise exc
code = task.result()
app.close()
sys.exit(code)
