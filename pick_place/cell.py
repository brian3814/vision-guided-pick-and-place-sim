"""The pick-and-place cell: a UR10 on a table, a gripper from the grasp matrix on its flange, one
object at the pick spot and a marked place spot. Runs inside Isaac Sim; import it only after
SimulationApp has started.

Frames: the robot base is at the world origin and the tabletop is at z = 0, the same height as
the grasp matrix's table. An object resting at the pick spot is the matrix's settled object moved
sideways, so a grasp stored by the matrix moves into the cell with the same horizontal shift.
"""

import pathlib
import sys

from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

import isaacsim.core.experimental.utils.stage as stage_utils

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "grasp_matrix"))
import gm_config  # noqa: E402
import gm_core  # noqa: E402
from isaacsim.replicator.grasping import grasping_utils  # noqa: E402

ROBOT = "/World/UR10"
# The UR10 asset's wrist_3_link body frame is cuMotion's tool0 frame (check_tool_frame() in
# run_cell.py verifies it at start-up). The gripper is mounted so its own frame, the frame the
# grasp matrix's poses refer to, coincides with it.
WRIST = f"{ROBOT}/wrist_3_link"
GRIPPER = f"{ROBOT}/Gripper"
MOUNT_JOINT = f"{ROBOT}/GripperMount"
TABLE = "/World/Table"
OBJECT_MOUNT = "/World/ObjectMount"
OBJECT = f"{OBJECT_MOUNT}/Object"
PLACE_MARKER = "/World/PlaceMarker"

UR10_USD = ROOT / "assets/universal_robots/ur10/ur10.usd"
UR10_VARIANTS = [("Physics", "PhysX"), ("Sensor", "None"), ("Gripper", "None")]
ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
# Tool pointing straight down, about 0.5 m above the table and 0.8 m in front of the base.
HOME_Q = [0.0, -1.2, 1.4, -1.7708, -1.5708, 0.0]

PICK_XY = (0.75, -0.35)
PLACE_XY = (0.75, 0.35)
TABLE_X, TABLE_Y = (-0.35, 1.25), (-0.8, 0.8)
TABLE_THICKNESS = 0.04


def start_pose(seed: int, cycle: int, max_yaw_deg: float, jitter_m: float):
    """A cycle's start pose for the object, seeded by (seed, cycle): turned about the vertical by
    up to max_yaw_deg and shifted by up to jitter_m in x and y from the pick spot. Returns (yaw in
    degrees, pick point, the matrix-world -> cell transform as a numpy pose)."""
    import numpy as np

    rng = np.random.default_rng([seed, cycle])
    yaw = float(rng.uniform(-max_yaw_deg, max_yaw_deg))
    jitter = rng.uniform(-jitter_m, jitter_m, 2)
    pick = np.array([PICK_XY[0] + jitter[0], PICK_XY[1] + jitter[1], 0.0])
    c, s = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
    move = np.array([[c, -s, 0, pick[0]], [s, c, 0, pick[1]], [0, 0, 1, 0], [0, 0, 0, 1.0]])
    return yaw, pick, move


def shift(x: float, y: float) -> Gf.Matrix4d:
    """Horizontal translation, as a matrix to post-multiply a pose with (Gf row-vector convention)."""
    return Gf.Matrix4d().SetTranslate(Gf.Vec3d(x, y, 0.0))


def is_body(stage: Usd.Stage, targets: list) -> bool:
    return bool(targets) and stage.GetPrimAtPath(targets[0]).HasAPI(UsdPhysics.RigidBodyAPI)


def world_joints(root: Usd.Prim) -> list[Usd.Prim]:
    """Joints under root that tie a body to the world: one of body0/body1 is empty, or is not a
    rigid body (the OnRobot RG2 ties its base to the gripper's root Xform, which PhysX treats as
    the world)."""
    stage = root.GetStage()
    out = []
    for p in Usd.PrimRange(root):
        if p.IsA(UsdPhysics.Joint):
            j = UsdPhysics.Joint(p)
            if not (is_body(stage, j.GetBody0Rel().GetTargets()) and is_body(stage, j.GetBody1Rel().GetTargets())):
                out.append(p)
    return out


def mount_gripper(stage: Usd.Stage) -> Usd.Prim:
    """Fix the gripper's base body to the wrist, with the gripper's frame on the wrist frame, and
    make it part of the arm's articulation: its own articulation root and its joint to the world
    are removed. Returns the gripper's base body."""
    root = stage.GetPrimAtPath(GRIPPER)
    iterations = []
    for p in Usd.PrimRange(root):
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            iterations.append(PhysxSchema.PhysxArticulationAPI(p).GetSolverPositionIterationCountAttr().Get() or 0)
            p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            p.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
    # The merged articulation keeps the arm's solver settings; give it the gripper's position
    # iteration count if that is higher (the 2F-140 uses 64, the UR10 32), as the grasp matrix
    # simulated the gripper with it.
    arm_root = next(p for p in Usd.PrimRange(stage.GetPrimAtPath(ROBOT)) if p.HasAPI(UsdPhysics.ArticulationRootAPI))
    arm_api = PhysxSchema.PhysxArticulationAPI.Apply(arm_root)
    arm_iterations = arm_api.GetSolverPositionIterationCountAttr().Get() or 0
    if iterations and max(iterations) > arm_iterations:
        arm_api.CreateSolverPositionIterationCountAttr(int(max(iterations)))
    anchors = world_joints(root)
    if len(anchors) != 1:
        raise RuntimeError(f"expected one joint tying the gripper to the world, found {[str(p.GetPath()) for p in anchors]}")
    j = UsdPhysics.Joint(anchors[0])
    body1 = j.GetBody1Rel().GetTargets()
    base = stage.GetPrimAtPath((body1 if is_body(stage, body1) else j.GetBody0Rel().GetTargets())[0])
    anchors[0].SetActive(False)

    xc = UsdGeom.XformCache()
    base_in_gripper = xc.GetLocalToWorldTransform(base) * xc.GetLocalToWorldTransform(root).GetInverse()
    gripper_in_base = base_in_gripper.GetInverse()
    mount = UsdPhysics.FixedJoint.Define(stage, MOUNT_JOINT)
    mount.CreateBody0Rel().SetTargets([Sdf.Path(WRIST)])
    mount.CreateBody1Rel().SetTargets([base.GetPath()])
    mount.CreateLocalPos0Attr(Gf.Vec3f(0.0))
    mount.CreateLocalRot0Attr(Gf.Quatf(1.0))
    mount.CreateLocalPos1Attr(Gf.Vec3f(gripper_in_base.ExtractTranslation()))
    mount.CreateLocalRot1Attr(Gf.Quatf(gripper_in_base.ExtractRotationQuat()))
    return base


def set_arm_start(stage: Usd.Stage, q: list[float]) -> None:
    """Start the simulation with the arm at q (radians): joint state and drive target of each arm joint."""
    for name, value in zip(ARM_JOINTS, q):
        joint = stage.GetPrimAtPath(f"{ROBOT}/joints/{name}")
        degrees = float(Gf.RadiansToDegrees(value))
        grasping_utils.set_joint_state(joint, position_value=degrees)
        grasping_utils.set_joint_drive_parameters(joint, target_value=degrees, target_type="position")


class Cell:
    """Builds the cell on a new stage. `object_mount` is the object's mount transform in the cell
    (see rest_mount()); the gripper is open and the arm at HOME_Q when the simulation starts."""

    def __init__(self, gripper_name: str, gripper: dict, object_name: str, object_mount: Gf.Matrix4d):
        self.gripper_name, self.g, self.object_name = gripper_name, gripper, object_name
        self.stage = stage = stage_utils.create_new_stage()
        stage_utils.set_stage_up_axis("Z")
        stage_utils.set_stage_units(meters_per_unit=1.0)
        UsdGeom.Xform.Define(stage, "/World")
        # Same physics scene as the grasp matrix's (gm_core.GraspScene), so contacts behave the same.
        scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        scene.CreateGravityMagnitudeAttr(9.81)

        table = UsdGeom.Cube.Define(stage, TABLE)
        table.CreateSizeAttr(1.0)
        table.AddTranslateOp().Set(Gf.Vec3d(sum(TABLE_X) / 2, sum(TABLE_Y) / 2, -TABLE_THICKNESS / 2))
        table.AddScaleOp().Set(Gf.Vec3f(TABLE_X[1] - TABLE_X[0], TABLE_Y[1] - TABLE_Y[0], TABLE_THICKNESS))
        table.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.45, 0.35)])
        UsdPhysics.CollisionAPI.Apply(table.GetPrim())

        marker = UsdGeom.Cylinder.Define(stage, PLACE_MARKER)  # visual only, no collision
        marker.CreateRadiusAttr(0.07)
        marker.CreateHeightAttr(0.001)
        marker.CreateAxisAttr("Z")
        marker.AddTranslateOp().Set(Gf.Vec3d(*PLACE_XY, 0.0006))
        marker.CreateDisplayColorAttr([Gf.Vec3f(0.2, 0.7, 0.3)])

        UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(900.0)
        sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
        sun.CreateIntensityAttr(2500.0)
        sun.AddRotateXYZOp().Set(Gf.Vec3f(-45, 10, 35))

        stage_utils.add_reference_to_stage(str(UR10_USD), ROBOT, variants=UR10_VARIANTS)
        stage_utils.add_reference_to_stage(str(ROOT / gripper["usd"]), GRIPPER, variants=[tuple(v) for v in gripper["variants"]])
        # Start the gripper on the wrist (the arm's stored pose); the mount joint keeps it there.
        wrist = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(WRIST))
        UsdGeom.Xformable(stage.GetPrimAtPath(GRIPPER)).AddTransformOp().Set(wrist)
        self.gripper_base = mount_gripper(stage)
        gm_core.apply_open_state(stage, GRIPPER, gripper)
        set_arm_start(stage, HOME_Q)

        self.load_object(object_name, object_mount)

        self.finger_joint = gripper["drive_joint"].split("/")[-1]
        self.coupled_joints = [j.split("/")[-1] for j in gripper.get("coupled_joints", [])]

    def load_object(self, object_name: str, object_mount: Gf.Matrix4d) -> None:
        """Put the object in the cell at `object_mount`, in place of the one there (swap objects only
        while the simulation is stopped)."""
        stage = self.stage
        if stage.GetPrimAtPath(OBJECT_MOUNT):
            stage.RemovePrim(OBJECT_MOUNT)
        mount = UsdGeom.Xform.Define(stage, OBJECT_MOUNT)
        mount.AddTransformOp().Set(object_mount)
        stage_utils.add_reference_to_stage(str(gm_config.object_usd(object_name)), OBJECT,
                                           variants=[("PhysX", "Enabled"), ("lod", "1")])
        obj = stage.GetPrimAtPath(OBJECT)
        self.object_name = object_name
        self.object_body = next(p for p in Usd.PrimRange(obj) if p.HasAPI(UsdPhysics.RigidBodyAPI))

    def set_object_mount(self, mount: Gf.Matrix4d) -> None:
        """Move the object's start pose (only while the simulation is stopped)."""
        UsdGeom.Xformable(self.stage.GetPrimAtPath(OBJECT_MOUNT)).GetOrderedXformOps()[0].Set(mount)

    def object_box(self) -> tuple[Gf.Vec3d, Gf.Vec3d]:
        """The object's box in its body frame: (centre, size)."""
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
        box = cache.ComputeRelativeBound(self.object_body, self.object_body).ComputeAlignedRange()
        return box.GetMidpoint(), box.GetSize()

    def tool_spheres(self) -> list[tuple[list[float], float]]:
        """Spheres covering each of the gripper's rigid bodies as stored (open, or as stored for
        grippers kept half open), in the gripper frame: for the planner's collision model. Each
        body's box gets spheres spanning its cross-section, spaced along its longest side."""
        import math

        root = self.stage.GetPrimAtPath(GRIPPER)
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
        spheres = []
        for body in Usd.PrimRange(root):
            if not body.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            box = cache.ComputeRelativeBound(body, root).ComputeAlignedRange()
            if box.IsEmpty():
                continue
            size, lo = box.GetSize(), box.GetMin()
            axes = sorted(range(3), key=lambda i: size[i])
            long_axis = axes[2]
            r = 0.5 * math.hypot(size[axes[0]], size[axes[1]])
            n = max(1, math.ceil((size[long_axis] - 2 * r) / r) + 1) if size[long_axis] > 2 * r else 1
            for k in range(n):
                c = list(box.GetMidpoint())
                if n > 1:
                    c[long_axis] = lo[long_axis] + r + k * (size[long_axis] - 2 * r) / (n - 1)
                spheres.append((c, r))
        return spheres


async def rest_mount(gripper_name: str, gripper: dict, object_name: str) -> Gf.Matrix4d:
    """The object's mount transform after it settles on the grasp matrix's table: the start pose
    of every trial the matrix ran on this object, which its stored grasps refer to. Recomputed with
    the matrix's own code (same steps, same result); builds and discards a GraspScene stage."""
    scene = gm_core.GraspScene(gripper_name, gripper, object_name)
    await scene.settle_object(gm_core.physics_steps)
    return Gf.Matrix4d(scene.mount_op.Get())
