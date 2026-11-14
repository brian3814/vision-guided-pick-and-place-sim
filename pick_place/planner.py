"""Motion planning for the cell with cuMotion's trajectory optimizer.

Needs only numpy and cuMotion's Python module (no Isaac Sim stage), so it can be tried outside
Isaac Sim. Inside Isaac Sim, import it after the isaacsim.robot_motion.cumotion extension is on.

Frames: the robot base is the world origin (cell.py puts it there), so cuMotion's base frame and
the world frame are the same. Poses are 4x4 numpy matrices acting on column vectors.
"""

import pathlib
from dataclasses import dataclass

import numpy as np

import cumotion

ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
TOOL_FRAME = "tool0"


def supported_robot_dir(name: str = "ur10") -> pathlib.Path:
    import isaacsim.robot_motion.cumotion as cu  # the wheel that ships the robot configurations

    return pathlib.Path(str(cu.__file__)).resolve().parent / "robot_configurations" / name


def tool_to_wrist3(p: np.ndarray) -> np.ndarray:
    """A point in UR10 tool0 coordinates -> URDF wrist_3_link coordinates (tool0 is wrist_3_link
    shifted 0.0922 m along y and turned -90 degrees about x)."""
    x, y, z = p
    return np.array([x, 0.0922 + z, -y])


def ur10_xrdf_with_tool(xrdf_text: str, tool_spheres: list[tuple[np.ndarray, float]]) -> str:
    """The stock UR10 XRDF plus collision spheres for the gripper (centres in tool0 coordinates).

    The spheres are attached to wrist_3_link, the last arm link, and wrist_2_link is told to
    ignore it (the gripper's palm sits right against the wrist)."""
    anchor = "  ur10_collision_spheres:\n    spheres:\n"
    ignore_anchor = '    wrist_1_link: [ "wrist_2_link" ]\n'
    if anchor not in xrdf_text or ignore_anchor not in xrdf_text:
        raise RuntimeError("unexpected UR10 XRDF layout; update ur10_xrdf_with_tool()")
    block = "      wrist_3_link:\n" + "".join(
        "        - center: [ {:.4f}, {:.4f}, {:.4f} ]\n          radius: {:.4f}\n".format(*tool_to_wrist3(c), r)
        for c, r in tool_spheres
    )
    text = xrdf_text.replace(anchor, anchor + block, 1)
    return text.replace(ignore_anchor, ignore_anchor + '    wrist_2_link: [ "wrist_3_link" ]\n', 1)


def pose3(m: np.ndarray) -> cumotion.Pose3:
    return cumotion.Pose3(np.asarray(m, dtype=float))


def rotation3(m: np.ndarray) -> cumotion.Rotation3:
    return cumotion.Rotation3.from_matrix(np.asarray(m[:3, :3], dtype=float))


@dataclass
class Plan:
    """A planned motion: the trajectory and, for a goalset, which target was chosen."""
    trajectory: cumotion.Trajectory
    target_index: int
    keep: tuple = ()  # the world and optimizer it came from, kept alive as long as the plan

    @property
    def duration(self) -> float:
        return float(self.trajectory.domain().span())

    def final(self) -> np.ndarray:
        d = self.trajectory.domain()
        return np.asarray(self.trajectory.eval(d.upper), dtype=float).flatten()

    def at(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        """Joint positions and velocities at time t from the start (clamped to the end)."""
        d = self.trajectory.domain()
        t = min(max(d.lower + t, d.lower), d.upper)
        return (np.asarray(self.trajectory.eval(t, 0), dtype=float).flatten(),
                np.asarray(self.trajectory.eval(t, 1), dtype=float).flatten())


class Planner:
    """UR10 + gripper spheres in a world of cuboid obstacles."""

    def __init__(self, tool_spheres: list[tuple[np.ndarray, float]], robot_dir: pathlib.Path | None = None):
        robot_dir = robot_dir or supported_robot_dir("ur10")
        xrdf = ur10_xrdf_with_tool((robot_dir / "robot.xrdf").read_text(encoding="utf-8"), tool_spheres)
        urdf = (robot_dir / "robot.urdf").read_text(encoding="utf-8")
        self.robot = cumotion.load_robot_from_memory(xrdf, urdf)
        self.kinematics = self.robot.kinematics()
        names = [self.robot.cspace_coord_name(i) for i in range(self.robot.num_cspace_coords())]
        if names != ARM_JOINTS:
            raise RuntimeError(f"unexpected cuMotion joint order {names}")
        # name -> (side lengths, pose); and which are enabled
        self.obstacles: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self.enabled: dict[str, bool] = {}
        self._keep: tuple = ()

    # --- world ---------------------------------------------------------------------------
    def add_cuboid(self, name: str, size, centre=None, pose: np.ndarray | None = None) -> None:
        """Add a box obstacle, or move/resize the one with this name. Give its centre, or its pose."""
        if pose is None:
            pose = np.eye(4)
            pose[:3, 3] = centre
        self.obstacles[name] = (np.asarray(size, dtype=float), np.asarray(pose, dtype=float))
        self.enabled.setdefault(name, True)

    def set_enabled(self, name: str, enabled: bool) -> None:
        self.enabled[name] = enabled

    def _optimizer(self) -> cumotion.TrajectoryOptimizer:
        """A fresh cuMotion world holding the enabled obstacles, and an optimizer for it (about
        0.07 s). Changing a world after an optimizer config was made from its view (disabling,
        removing obstacles) crashed the process inside Isaac Sim, so worlds are never changed."""
        world = cumotion.create_world()
        view = world.add_world_view()
        # Inside Isaac Sim, creating an optimizer for a world with no enabled obstacle crashed the
        # process, so there is always one: a 1 cm cube 10 m below the robot.
        far = np.eye(4)
        far[2, 3] = -10.0
        obstacles = dict(self.obstacles, _placeholder=(np.full(3, 0.01), far))
        for name, (size, pose) in obstacles.items():
            if self.enabled.get(name, True):
                obstacle = cumotion.create_obstacle(cumotion.Obstacle.Type.CUBOID)
                obstacle.set_attribute(cumotion.Obstacle.Attribute.SIDE_LENGTHS, size)
                world.add_obstacle(obstacle, pose3(pose))
        view.update()
        config = cumotion.create_default_trajectory_optimizer_config(self.robot, TOOL_FRAME, view)
        optimizer = cumotion.create_trajectory_optimizer(config)
        self._keep = (world, view, config, optimizer)  # alive until the next plan
        return optimizer

    # --- kinematics ----------------------------------------------------------------------
    def tool_pose(self, q) -> np.ndarray:
        p = self.kinematics.pose(np.asarray(q, dtype=float), TOOL_FRAME)
        m = np.eye(4)
        m[:3, :3] = p.rotation.matrix()
        m[:3, 3] = p.translation
        return m

    # --- planning ------------------------------------------------------------------------
    def _result(self, result) -> Plan | None:
        if result.status() != cumotion.TrajectoryOptimizer.Results.Status.SUCCESS:
            return None
        return Plan(result.trajectory(), int(result.target_index()), self._keep)

    def plan_goalset(self, q0, poses: list[np.ndarray]) -> Plan | None:
        """Collision-free motion to whichever of the tool poses suits best."""
        TO = cumotion.TrajectoryOptimizer
        goal = TO.TaskSpaceTargetGoalset(
            translation_constraints=TO.TranslationConstraintGoalset.target([p[:3, 3] for p in poses]),
            orientation_constraints=TO.OrientationConstraintGoalset.terminal_target([rotation3(p) for p in poses]),
        )
        return self._result(self._optimizer().plan_to_task_space_goalset(np.asarray(q0, dtype=float), goal))

    def plan_pose(self, q0, pose: np.ndarray) -> Plan | None:
        TO = cumotion.TrajectoryOptimizer
        goal = TO.TaskSpaceTarget(
            translation_constraint=TO.TranslationConstraint.target(pose[:3, 3]),
            orientation_constraint=TO.OrientationConstraint.terminal_target(rotation3(pose)),
        )
        return self._result(self._optimizer().plan_to_task_space_target(np.asarray(q0, dtype=float), goal))

    def plan_line(self, q0, pose: np.ndarray, deviation_m: float = 0.005) -> Plan | None:
        """Straight-line tool motion to the pose, orientation held constant on the way."""
        TO = cumotion.TrajectoryOptimizer
        goal = TO.TaskSpaceTarget(
            translation_constraint=TO.TranslationConstraint.linear_path_constraint(pose[:3, 3], deviation_m),
            orientation_constraint=TO.OrientationConstraint.terminal_and_path_target(rotation3(pose)),
        )
        return self._result(self._optimizer().plan_to_task_space_target(np.asarray(q0, dtype=float), goal))

    def plan_joints(self, q0, q1) -> Plan | None:
        TO = cumotion.TrajectoryOptimizer
        goal = TO.CSpaceTarget(np.asarray(q1, dtype=float))
        return self._result(self._optimizer().plan_to_cspace_target(np.asarray(q0, dtype=float), goal))
