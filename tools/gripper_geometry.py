"""Inspect a gripper asset and draft its entry for grasp_matrix/config/grippers.json.

Prints the variant sets, rigid bodies (with bounding boxes in the gripper's own frame), driven
and mimic joints, root joints and physics materials, then a draft config entry. The draft
is a guess from names and geometry: check it, then run tools/calibrate_grippers.py, whose
checks catch wrong axes, wrong fingertips and a base that does not follow the gripper.

Runs with the project venv (usd-core), no Isaac Sim needed:
    .venv/Scripts/python.exe tools/gripper_geometry.py assets/onrobot/onrobot_rg2/simready_usd/onrobot_rg2.usda Physics=physx_parallel_grip
"""

import json
import pathlib
import sys

from pxr import Gf, Usd, UsdGeom, UsdPhysics

ROOT = pathlib.Path(__file__).resolve().parents[1]


def main(path: str, selections: list[str]) -> None:
    stage = Usd.Stage.Open(path)
    root = stage.GetDefaultPrim()
    print(f"== {path} (default prim {root.GetPath()})")
    for name in root.GetVariantSets().GetNames():
        vs = root.GetVariantSet(name)
        print(f"variant set {name}: {vs.GetVariantNames()} (default: {vs.GetVariantSelection()})")
    variants = []
    for spec in selections:
        vset, sel = spec.split("=")
        root.GetVariantSet(vset).SetVariantSelection(sel)
        variants.append([vset, sel])

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])

    def rel(prim: Usd.Prim) -> str:
        return str(prim.GetPath())[len(str(root.GetPath())) + 1:]

    def box(prim: Usd.Prim) -> tuple[Gf.Vec3d, Gf.Vec3d]:
        r = cache.ComputeRelativeBound(prim, root).ComputeAlignedRange()
        return r.GetMin(), r.GetMax()

    def fmt(v: Gf.Vec3d) -> list[float]:
        return [round(x, 4) for x in v]

    bodies, drives, joints = [], [], []
    print("\nrigid bodies (box in the gripper frame, m):")
    for p in Usd.PrimRange(root):
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            print(f"  articulation root: {rel(p) or '(default prim)'}")
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            lo, hi = box(p)
            bodies.append(p)
            print(f"  {p.GetName():34s} {fmt(lo)} .. {fmt(hi)}")
        if p.IsA(UsdPhysics.Joint):
            joints.append(p)
    print("\njoints:")
    for p in joints:
        j = UsdPhysics.Joint(p)
        kind = p.GetTypeName().replace("Physics", "")
        mimic = [a.GetName() for a in p.GetAttributes() if a.GetName().startswith("physxMimicJoint:") and a.GetName().endswith("gearing")]
        driven = [d for d in ("angular", "linear") if p.HasAPI(UsdPhysics.DriveAPI, d)]
        limits = (p.GetAttribute("physics:lowerLimit").Get(), p.GetAttribute("physics:upperLimit").Get())
        body0 = [str(t) for t in j.GetBody0Rel().GetTargets()]
        note = []
        if driven:
            d = UsdPhysics.DriveAPI(p, driven[0])
            note.append(f"DRIVEN stiffness {d.GetStiffnessAttr().Get()} damping {d.GetDampingAttr().Get()} "
                        f"maxForce {d.GetMaxForceAttr().Get()}")
            drives.append(p)
        if mimic:
            note.append("mimic")
        if kind == "FixedJoint" and not body0:
            note.append("fixed to the world (body0 empty)")
        lim = f" limits {limits[0]}..{limits[1]}" if limits[0] is not None else ""
        print(f"  {rel(p):50s} {kind}{lim} {'; '.join(note)}")
    print("\nphysics materials:")
    materials = [p for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.MaterialAPI)]
    for p in materials:
        m = UsdPhysics.MaterialAPI(p)
        print(f"  {rel(p)}: static {m.GetStaticFrictionAttr().Get()}, dynamic {m.GetDynamicFrictionAttr().Get()}")
    if not materials:
        print("  none: contacts would use PhysX's default friction")

    # Draft entry: fingertips by name, axes from where the two fingertips sit.
    tips = [b for b in bodies if "fingertip" in b.GetName()]
    named = len(tips) != 2
    if named:
        tips = [b for b in bodies if "tip" in b.GetName().lower()] or [b for b in bodies if "finger" in b.GetName().lower()]
    entry: dict = {"label": root.GetName().upper(), "usd": str(pathlib.Path(path).resolve().relative_to(ROOT)).replace("\\", "/"),
                   "variants": variants}
    if len(tips) == 2:
        centres = [(lo + hi) / 2 for lo, hi in (box(t) for t in tips)]
        apart, mid = [abs(x) for x in centres[1] - centres[0]], (centres[0] + centres[1]) / 2
        closing = [0, 0, 0]
        closing[apart.index(max(apart))] = 1
        approach = [0, 0, 0]
        axis = max(range(3), key=lambda i: abs(mid[i]))
        approach[axis] = 1 if mid[axis] > 0 else -1
        parents = [str(t.GetParent().GetPath()) for t in tips]
        common = parents[0] if parents[0] == parents[1] else str(root.GetPath())
        entry["articulation"] = common[len(str(root.GetPath())) + 1:] or "."
    else:
        closing, approach = "?", "?"
        entry["articulation"] = "?"
        print(f"\ncould not pick two fingertip bodies (candidates: {[t.GetName() for t in tips]}); set \"fingertips\" by hand")
    if len(tips) == 2:
        # Collision shapes of one fingertip, and how far each sits back from the one reaching
        # furthest towards the other finger: a stepped fingertip needs "pad_shapes".
        guide = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"])
        tip = tips[0]
        inward = -1 if centres[0][closing.index(1)] > 0 else 1
        shapes = []
        for p in Usd.PrimRange(tip, Usd.TraverseInstanceProxies()):
            if p.HasAPI(UsdPhysics.CollisionAPI):
                r = guide.ComputeRelativeBound(p, root).ComputeAlignedRange()
                lo, hi = r.GetMin(), r.GetMax()
                reach = (hi if inward > 0 else -lo)[closing.index(1)]
                along = sorted((lo[axis] * approach[axis], hi[axis] * approach[axis]))
                shapes.append((reach, p.GetName(), along))
        if shapes:
            top = max(s[0] for s in shapes)
            print(f"\ncollision shapes of {tip.GetName()} (set back from the innermost; span along the approach axis):")
            for reach, name, along in sorted(shapes, key=lambda s: s[2][0]):
                print(f"  {name:28s} {(top - reach) * 1000:5.1f} mm back   {along[0] * 1000:6.1f} .. {along[1] * 1000:6.1f} mm")
            if len({round(s[0], 3) for s in shapes}) > 1:
                print("  The shapes sit at different depths: set \"pad_shapes\" to those that should touch the object")
    if drives:
        entry["drive_joint"] = rel(drives[0])
        if len(drives) > 1:
            entry["coupled_joints"] = [rel(d) for d in drives[1:]]
        entry["joint_type"] = "revolute" if drives[0].IsA(UsdPhysics.RevoluteJoint) else "prismatic"
    if named and len(tips) == 2:
        entry["fingertips"] = [t.GetName() for t in tips]
    entry["approach_axis"], entry["closing_axis"] = approach, closing
    print("\ndraft config entry (check it; add \"datasheet\": {\"max_opening_m\": ...} if known):")
    print(json.dumps({root.GetName(): entry}, indent=2))
    print("If tools/calibrate_grippers.py then reports that the gripper does not follow its base, add "
          "\"anchor_joint\" with the root joint that is fixed to the world.")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2:])
