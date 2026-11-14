"""Scene and trial logic for the grasp matrix. Runs inside Isaac Sim; import it only after
SimulationApp has started.

Protocol for one gripper x object pair:
1. The object is dropped on a table and left to settle; its resting pose becomes the start
   pose for every trial.
2. Antipodal grasp candidates are sampled on the object's visual mesh, then filtered: no
   approach from below the table, and the fingertip pads must clear the tabletop both open and
   closed to the object's width at the grasp. (The 2F fingertips move up to 23 mm along the
   approach axis while closing, so a pad that clears the table when open can hit it when
   closing.) A grasp whose pads would cut into the table is backed off along its approach
   axis, by at most half the pad length, so the sampled contact point stays on the pad.
3. Each remaining grasp: place the open gripper, close the fingers, then switch the table's
   collision off, which is the same as lifting the object, and see whether the fingers hold it.
   Every grasp that survives placement gets this lift test; how far the fingers closed only
   labels the failures.

Outcomes:
  collided  placing the open gripper moved the object more than PLACEMENT_TOLERANCE_M (no lift test)
  held      with the table gone, the object came to rest in the fingers: it moved less than
            LATE_MOVE_TOLERANCE_M in the second half of the lift and sagged less than MAX_SAG_M
  missed    not held, and the fingers closed (almost) to where the pads meet: nothing between them
  blocked   not held, and the fingers stopped more than BLOCKED_MARGIN_M wider than the object's
            width at the grasp: something else (the table, another part of the object) was in the way
  dropped   not held otherwise: the fingers reached the object but it slipped out
"""

import time
from collections.abc import Awaitable, Callable

import numpy as np
import omni.kit.app
import omni.physx
import trimesh
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

import isaacsim.core.experimental.utils.stage as stage_utils

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.replicator.grasping", True)
from isaacsim.replicator.grasping import grasping_utils, sampler_utils  # noqa: E402
from isaacsim.replicator.grasping.grasping_manager import GraspingManager  # noqa: E402

from gm_config import (  # noqa: E402
    APERTURE_MARGIN, BLOCKED_MARGIN_M, CLOSE_STEPS, CONDITIONS, DT, FROM_BELOW_MAX_APPROACH_Z,
    LATE_MOVE_TOLERANCE_M, LIFT_STEPS, MAX_LIFT_PAD_FRACTION, MAX_SAG_M, MISSED_FRACTION, PLACE_STEPS,
    PLACEMENT_TOLERANCE_M, ROOT, SETTLE_OBJECT_STEPS, TABLE_CLEARANCE_M, is_fingertip, load_grippers, object_usd,
)

TABLE = "/World/Table"
OBJECT_MOUNT = "/World/ObjectMount"
OBJECT = f"{OBJECT_MOUNT}/Object"
GRIPPER = "/World/Gripper"
PARKED_LOCATION = Gf.Vec3d(0.0, 0.0, 1.5)

Step = Callable[[int], Awaitable[None]]


async def physics_steps(n: int) -> None:
    """Advance PhysX directly, without the timeline or rendering (fast path for batch runs)."""
    await grasping_utils.simulate_physics_async(n, DT)


def anchor_gripper(stage: Usd.Stage, gripper_root: str, gripper: dict) -> None:
    """Fix the gripper's base to its own root prim instead of the world origin, for assets whose
    fixed root joint has no body0 and so pins the base to the world origin (config key
    "anchor_joint"). Then the gripper can be placed by moving its root prim."""
    if "anchor_joint" in gripper:
        joint = UsdPhysics.Joint(stage.GetPrimAtPath(f"{gripper_root}/{gripper['anchor_joint']}"))
        joint.GetBody0Rel().SetTargets([Sdf.Path(gripper_root)])


def apply_open_state(stage: Usd.Stage, gripper_root: str, gripper: dict) -> None:
    """Make every simulation start with the fingers open, for assets stored with them closed or
    half open (calibration key "open_state"). Sets each joint's initial position, so after a reset
    the fingers are already open when the gripper is placed. Without it they would open during
    placement, next to the object."""
    for rel, position in gripper.get("open_state", {}).get("joints", {}).items():
        grasping_utils.set_joint_state(stage.GetPrimAtPath(f"{gripper_root}/{rel}"), position_value=position)


def pad_boxes(tips: list[Usd.Prim], gripper: dict, relative_to: Usd.Prim | None = None) -> list[Gf.Range3d]:
    """Each fingertip pad's axis-aligned box, in world coordinates or relative to a prim.

    The pad is the fingertip body's whole visible geometry, or with the config key "pad_shapes",
    the named shapes under it: for stepped fingertips, whose collision shapes sit at different
    depths and only some of them touch the object (Franka Hand). Those are usually collision
    shapes with 'guide' purpose, so guide geometry is included then."""
    names = gripper.get("pad_shapes")
    purposes = ["default", "render", "proxy"] + (["guide"] if names else [])
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), purposes)
    boxes = []
    for tip in tips:
        prims = [p for p in Usd.PrimRange(tip, Usd.TraverseInstanceProxies()) if p.GetName() in names] if names else [tip]
        if not prims:
            raise ValueError(f"none of pad_shapes {names} found under {tip.GetPath()}")
        ranges = [(cache.ComputeRelativeBound(p, relative_to) if relative_to else cache.ComputeWorldBound(p)).ComputeAlignedRange()
                  for p in prims]
        box = ranges[0]
        for r in ranges[1:]:
            box = Gf.Range3d.GetUnion(box, r)
        boxes.append(box)
    return boxes


def _pose_matrix(position, quat_xyzw) -> Gf.Matrix4d:
    m = Gf.Matrix4d()
    m.SetTransform(Gf.Rotation(Gf.Quatd(quat_xyzw[3], *quat_xyzw[:3])), Gf.Vec3d(*position))
    return m


class GraspScene:
    """One gripper and one object on a table, plus everything needed to run trials on them."""

    def __init__(self, gripper_name: str, gripper: dict, object_name: str):
        self.gripper_name, self.g, self.object_name = gripper_name, gripper, object_name
        self.stage = stage_utils.create_new_stage()
        stage_utils.set_stage_up_axis("Z")
        stage_utils.set_stage_units(meters_per_unit=1.0)
        stage = self.stage
        UsdGeom.Xform.Define(stage, "/World")
        scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        scene.CreateGravityMagnitudeAttr(9.81)

        table = UsdGeom.Cube.Define(stage, TABLE)
        table.CreateSizeAttr(1.0)
        table.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.02))
        table.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 0.04))  # top surface at z = 0
        self.table_collision = UsdPhysics.CollisionAPI.Apply(table.GetPrim())

        mount = UsdGeom.Xform.Define(stage, OBJECT_MOUNT)
        self.mount_op = mount.AddTransformOp()
        self.mount_op.Set(Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, 0, 0.005)))
        stage_utils.add_reference_to_stage(str(object_usd(object_name)), OBJECT, variants=[("PhysX", "Enabled"), ("lod", "1")])
        stage_utils.add_reference_to_stage(str(ROOT / gripper["usd"]), GRIPPER, variants=[tuple(v) for v in gripper["variants"]])
        anchor_gripper(stage, GRIPPER, gripper)
        apply_open_state(stage, GRIPPER, gripper)

        obj = stage.GetPrimAtPath(OBJECT)
        self.body = next(p for p in Usd.PrimRange(obj) if p.HasAPI(UsdPhysics.RigidBodyAPI))
        self.body_path = str(self.body.GetPath())
        self.mesh_path = str(next(p for p in Usd.PrimRange(self.body) if p.GetName() == "mesh_obj").GetPath())
        self.joint = stage.GetPrimAtPath(f"{GRIPPER}/{gripper['drive_joint']}")
        # Joints driven with the same target as the drive joint, for grippers whose fingers have
        # separate drives instead of a mimic linkage (Franka Hand).
        self.coupled_joints = [stage.GetPrimAtPath(f"{GRIPPER}/{j}") for j in gripper.get("coupled_joints", [])]

        # Originals for the test conditions: every physics material on the object and gripper, and the object's mass.
        self.materials = [
            (UsdPhysics.MaterialAPI(p), UsdPhysics.MaterialAPI(p).GetStaticFrictionAttr().Get(),
             UsdPhysics.MaterialAPI(p).GetDynamicFrictionAttr().Get())
            for root in (OBJECT, GRIPPER)
            for p in Usd.PrimRange(stage.GetPrimAtPath(root)) if p.HasAPI(UsdPhysics.MaterialAPI)
        ]
        self.mass_attr = UsdPhysics.MassAPI(self.body).GetMassAttr()
        self.mass = self.mass_attr.Get()

        # Fingertip pad boxes in the gripper's own frame with the fingers open, for the table check.
        gripper_prim = stage.GetPrimAtPath(GRIPPER)
        tips = [p for p in Usd.PrimRange(stage.GetPrimAtPath(f"{GRIPPER}/{gripper['articulation']}"))
                if p.HasAPI(UsdPhysics.RigidBodyAPI) and is_fingertip(p.GetName(), gripper)]
        assert len(tips) == 2, f"{gripper_name}: expected 2 fingertip bodies, found {len(tips)}"
        self.pad_corners = []
        if "open_state" in gripper:  # the asset is not stored open; use the calibrated open pads
            boxes = [Gf.Range3d(Gf.Vec3d(*lo), Gf.Vec3d(*hi)) for lo, hi in gripper["open_state"]["pad_boxes_m"]]
        else:
            boxes = pad_boxes(tips, gripper, relative_to=gripper_prim)
        for box in boxes:
            lo, hi = box.GetMin(), box.GetMax()
            self.pad_corners += [Gf.Vec3d(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
        approach = Gf.Vec3d(*gripper["approach_axis"])
        along = [Gf.Dot(c, approach) for c in self.pad_corners]
        self.max_lift = MAX_LIFT_PAD_FRACTION * (max(along) - min(along))

        # Closing curve sorted by pad gap, for pad gap -> pad height and joint -> pad gap lookups.
        curve = np.array(sorted(gripper["closing_curve"], key=lambda p: p[1]))
        self.curve_gap, self.curve_height = curve[:, 1], curve[:, 2]
        by_joint = curve[np.argsort(curve[:, 0])]
        self.curve_joint_sorted, self.curve_gap_by_joint = by_joint[:, 0], by_joint[:, 1]
        self._world_mesh: trimesh.Trimesh | None = None

        self.manager = GraspingManager()
        self.manager.set_gripper(GRIPPER)
        self.manager.set_object_prim_path(self.mesh_path)
        self.manager.set_gripper_pose(PARKED_LOCATION, Gf.Quatd(1, 0, 0, 0))

    # --- state helpers ---------------------------------------------------------------------
    def set_table(self, enabled: bool) -> None:
        self.table_collision.CreateCollisionEnabledAttr(enabled)

    def set_finger(self, target: float) -> None:
        for joint in (self.joint, *self.coupled_joints):
            grasping_utils.set_joint_drive_parameters(joint, target_value=target, target_type="position")

    def finger_position(self) -> float:
        return float(grasping_utils.get_joint_state(self.joint)[0])

    def object_position(self) -> np.ndarray:
        """Object position from PhysX while simulating, from USD otherwise."""
        result = omni.physx.get_physx_interface().get_rigidbody_transformation(self.body_path)
        if result and result.get("ret_val"):
            return np.array(result["position"], dtype=float)
        return self.object_position_usd()

    def object_position_usd(self) -> np.ndarray:
        m = UsdGeom.Xformable(self.body).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return np.array(m.ExtractTranslation(), dtype=float)

    def apply_condition(self, condition: str) -> None:
        c = CONDITIONS[condition]
        for api, static, dynamic in self.materials:
            api.GetStaticFrictionAttr().Set(static * c["friction_scale"])
            api.GetDynamicFrictionAttr().Set(dynamic * c["friction_scale"])
        self.mass_attr.Set(self.mass * c["mass_scale"])

    # --- setup steps -----------------------------------------------------------------------
    async def settle_object(self, step: Step) -> float:
        """Drop the object on the table and make its resting pose the start pose. Returns the drop distance."""
        grasping_utils.reset_physics_simulation()
        self.set_table(True)
        body_init = UsdGeom.Xformable(self.body).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        mount_init = UsdGeom.Xformable(self.stage.GetPrimAtPath(OBJECT_MOUNT)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        await step(SETTLE_OBJECT_STEPS)
        pose = omni.physx.get_physx_interface().get_rigidbody_transformation(self.body_path)
        body_settled = _pose_matrix(pose["position"], pose["rotation"])
        grasping_utils.reset_physics_simulation()
        body_in_mount = body_init * mount_init.GetInverse()
        self.mount_op.Set(body_in_mount.GetInverse() * body_settled)
        return float((body_init.ExtractTranslation() - body_settled.ExtractTranslation()).GetLength())

    def sample_grasps(self, candidates: int, orientations: int, seed: int) -> list[tuple[Gf.Vec3d, Gf.Quatd]]:
        g = self.g
        self.manager.clear_grasp_poses()
        self.manager.sampler_config = {
            "sampler_type": "antipodal",
            "num_candidates": candidates,
            "num_orientations": orientations,
            "orientation_sample_axis": g["closing_axis"],
            "grasp_align_axis": g["closing_axis"],
            "gripper_approach_direction": g["approach_axis"],
            "gripper_maximum_aperture": g["max_aperture_m"] * APERTURE_MARGIN,
            "gripper_standoff_fingertips": g["fingertip_standoff_m"],
            "lateral_sigma": 0.0,
            "random_seed": seed,
            "verbose": False,
        }
        if not self.manager.generate_grasp_poses():
            return []
        return self.manager.get_grasp_poses(in_world_frame=True)

    def _object_mesh_world(self) -> trimesh.Trimesh:
        """The object's visual mesh in world coordinates at its settled pose (built once, after settling)."""
        if self._world_mesh is not None:
            return self._world_mesh
        mesh_prim = UsdGeom.Mesh(self.stage.GetPrimAtPath(self.mesh_path))
        mesh = sampler_utils.usd_mesh_to_trimesh(mesh_prim, apply_scale=False)
        to_world = np.array(mesh_prim.ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
        mesh.apply_transform(to_world.T)  # Gf matrices act on row vectors; trimesh expects column vectors
        self._world_mesh = mesh
        return mesh

    def grasp_widths(self, poses: list[tuple[Gf.Vec3d, Gf.Quatd]]) -> list[float | None]:
        """The object's width along each grasp's closing line, through the grasp centre.

        The sampler puts the grasp centre midway between two contacts, fingertip_standoff_m along
        the approach axis from the gripper origin. A ray along the closing axis, spanning the
        open gap, finds the outermost surface hits; their distance is the width the fingers close
        to. None if the ray hits the mesh fewer than twice."""
        if not poses:
            return []
        g = self.g
        approach, closing = Gf.Vec3d(*g["approach_axis"]), Gf.Vec3d(*g["closing_axis"])
        span = g["max_aperture_m"]
        origins, dirs = [], []
        for loc, quat in poses:
            rot = Gf.Rotation(quat)
            centre = loc + rot.TransformDir(approach) * g["fingertip_standoff_m"]
            d = rot.TransformDir(closing).GetNormalized()
            origins.append(list(centre - d * (span / 2)))
            dirs.append(list(d))
        origins, dirs = np.array(origins), np.array(dirs)
        hits, ray_index, _ = self._object_mesh_world().ray.intersects_location(origins, dirs, multiple_hits=True)
        along = np.einsum("ij,ij->i", hits - origins[ray_index], dirs[ray_index]) if len(hits) else np.array([])
        widths: list[float | None] = []
        for i in range(len(poses)):
            t = along[(ray_index == i) & (along >= 0.0) & (along <= span)] if len(hits) else along
            widths.append(round(float(t.max() - t.min()), 4) if len(t) >= 2 else None)
        return widths

    def pad_descent(self, width: float | None) -> float:
        """How far the pads move along the approach axis (away from the palm) when closing from fully
        open to `width`. Unknown width -> fully closed, the worst case."""
        gap = self.g["max_aperture_m"]
        w = float(np.clip(width if width is not None else 0.0, self.curve_gap[0], gap))
        return float(np.interp(w, self.curve_gap, self.curve_height) - np.interp(gap, self.curve_gap, self.curve_height))

    def finger_gap(self, joint_position: float) -> float:
        """Pad gap at a drive-joint position, from the calibration curve."""
        return float(np.interp(joint_position, self.curve_joint_sorted, self.curve_gap_by_joint))

    def _lowest_pad_z(self, location: Gf.Vec3d, rot: Gf.Rotation) -> float:
        return min((location + rot.TransformDir(c))[2] for c in self.pad_corners)

    def pre_filter(self, location: Gf.Vec3d, orientation: Gf.Quatd, width: float | None) -> dict:
        """Check a sampled grasp before any physics.

        Returns the (possibly backed-off) location, the approach direction's world z, the lowest
        pad point open and closed to `width`, how far the grasp was backed off, and the rejection
        reason (None if it passed)."""
        rot = Gf.Rotation(orientation)
        approach = rot.TransformDir(Gf.Vec3d(*self.g["approach_axis"]))
        # Closing moves the pads by pad_descent along the approach axis; in world z that is * approach_z.
        closing_drop = self.pad_descent(width) * approach[2]
        out = {"location": location, "approach_z": float(approach[2]), "width_m": width, "lift_m": 0.0, "filter": None}

        def pad_heights(loc: Gf.Vec3d) -> None:
            out["lowest_pad_z"] = round(float(self._lowest_pad_z(loc, rot)), 5)
            out["closed_pad_z"] = round(out["lowest_pad_z"] + float(closing_drop), 5)

        pad_heights(location)
        if approach[2] > FROM_BELOW_MAX_APPROACH_Z:
            out["filter"] = "from_below"
            return out
        lowest = min(out["lowest_pad_z"], out["closed_pad_z"])
        if lowest < TABLE_CLEARANCE_M:
            # Backing off along the approach axis raises the pads by -approach_z per metre.
            lift = (TABLE_CLEARANCE_M - lowest) / -approach[2] if approach[2] < -1e-3 else float("inf")
            if lift > self.max_lift:
                out["filter"] = "hits_table"
                return out
            out["location"] = location - approach * lift
            out["lift_m"] = round(float(lift), 4)
            pad_heights(out["location"])
        return out

    # --- trials ----------------------------------------------------------------------------
    async def control_trial(self, step: Step) -> float:
        """Gripper parked away; switching the table off must drop the object. Returns how far it moved."""
        grasping_utils.reset_physics_simulation()
        self.apply_condition("baseline")
        self.set_table(True)
        self.manager.set_gripper_pose(PARKED_LOCATION, Gf.Quatd(1, 0, 0, 0))
        await step(PLACE_STEPS)
        before = self.object_position()
        self.set_table(False)
        await step(LIFT_STEPS)
        moved = float(np.linalg.norm(self.object_position() - before))
        self.set_table(True)
        # PhysX writes the fallen pose back to USD; reset so later steps (grasp sampling reads
        # the object's USD transform) see the object on the table again.
        grasping_utils.reset_physics_simulation()
        return moved

    async def run_trial(
        self, location: Gf.Vec3d, orientation: Gf.Quatd, width: float | None, condition: str, step: Step,
        on_phase: Callable[[str], Awaitable[None]] | None = None,
    ) -> dict:
        """Run one grasp. `width` is the object's width at the grasp (grasp_widths), used only to
        label a failure as blocked. on_phase, if given, is awaited after the 'placed', 'closed'
        and 'lifted' phases (used by render_grasp.py to take pictures)."""
        g = self.g
        t0 = time.perf_counter()

        async def phase(name: str) -> None:
            if on_phase is not None:
                await on_phase(name)

        grasping_utils.reset_physics_simulation()
        self.apply_condition(condition)
        self.set_table(True)
        self.set_finger(g["open_target"])
        self.manager.set_gripper_pose(location, orientation)
        start = self.object_position_usd()
        result: dict[str, float | str | None] = {
            "finger_position": None, "finger_fraction": None, "finger_gap_m": None, "placement_move_m": None,
            "lift_move_m": None, "late_move_m": None,
        }

        await step(PLACE_STEPS)
        await phase("placed")
        result["placement_move_m"] = round(float(np.linalg.norm(self.object_position() - start)), 4)
        if result["placement_move_m"] > PLACEMENT_TOLERANCE_M:
            return self._finish(result, "collided", t0)

        self.set_finger(g["close_target"])
        await step(CLOSE_STEPS)
        await phase("closed")
        finger = self.finger_position()
        fraction = (finger - g["open_target"]) / (g["pads_touch_position"] - g["open_target"])
        result["finger_position"], result["finger_fraction"] = round(finger, 5), round(fraction, 4)
        result["finger_gap_m"] = round(self.finger_gap(finger), 4)

        before_lift = self.object_position()
        self.set_table(False)
        await step(LIFT_STEPS // 2)
        midway = self.object_position()
        await step(LIFT_STEPS - LIFT_STEPS // 2)
        await phase("lifted")
        end = self.object_position()
        result["lift_move_m"] = round(float(np.linalg.norm(end - before_lift)), 4)
        result["late_move_m"] = round(float(np.linalg.norm(end - midway)), 4)
        self.set_table(True)
        if result["late_move_m"] < LATE_MOVE_TOLERANCE_M and result["lift_move_m"] < MAX_SAG_M:
            outcome = "held"
        elif fraction >= MISSED_FRACTION:
            outcome = "missed"
        elif width is not None and result["finger_gap_m"] - width > BLOCKED_MARGIN_M:
            outcome = "blocked"
        else:
            outcome = "dropped"
        return self._finish(result, outcome, t0)

    @staticmethod
    def _finish(result: dict, outcome: str, t0: float) -> dict:
        result["outcome"] = outcome
        result["seconds"] = round(time.perf_counter() - t0, 3)
        return result
