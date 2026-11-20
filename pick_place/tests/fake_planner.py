"""A fake planner for the tests: cumotion_planner.CuMotionPlanner's interface without cuMotion,
recording what it was asked. Plans are straight joint-space lines lasting 1/3 s (a duration with
every binary digit used, to show values cross the process boundary unchanged)."""

import numpy as np

DURATION = 1 / 3


class FakePlan:
    def __init__(self, q0, q1, target_index=0):
        self.q0, self.q1 = np.asarray(q0, dtype=float), np.asarray(q1, dtype=float)
        self.target_index = target_index
        self.duration = DURATION

    def final(self):
        return self.q1.copy()

    def at(self, t):
        s = min(max(t, 0.0), DURATION) / DURATION
        return self.q0 + s * (self.q1 - self.q0), (self.q1 - self.q0) / DURATION


class FakePlanner:
    made: list["FakePlanner"] = []  # every planner made, for the tests to look at

    def __init__(self, tool_spheres, robot_dir):
        self.tool_spheres, self.robot_dir = tool_spheres, robot_dir
        self.calls: list[tuple] = []
        FakePlanner.made.append(self)

    def add_cuboid(self, name, size, centre=None, pose=None):
        self.calls.append(("add_cuboid", name, size.tolist(), None if centre is None else centre.tolist(),
                           None if pose is None else pose.tolist()))

    def set_enabled(self, name, enabled):
        self.calls.append(("set_enabled", name, enabled))

    def tool_pose(self, q):
        m = np.eye(4)
        m[:3, 3] = q[:3] / 7
        return m

    def plan_goalset(self, q0, poses):
        return FakePlan(q0, q0 + poses[-1][:3, 3].sum(), target_index=len(poses) - 1)

    def plan_pose(self, q0, pose):
        return None  # no plan

    def plan_line(self, q0, pose, deviation_m=0.005):
        if pose[2, 3] < 0:
            raise ValueError("below the table")
        return FakePlan(q0, q0 + deviation_m)

    def plan_joints(self, q0, q1):
        return FakePlan(q0, q1)
