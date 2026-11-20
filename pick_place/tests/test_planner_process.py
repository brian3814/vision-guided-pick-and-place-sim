"""planner.Planner and the planning process (planner_process.py) with a fake planner: no cuMotion,
no Isaac Sim. They need numpy, so run them with Isaac Sim's Python; without numpy they are skipped.

    C:/issacsim-6-2/python.bat -m unittest discover -s pick_place/tests
"""

import pathlib
import sys
import threading
import unittest
from multiprocessing.connection import Pipe

HERE = pathlib.Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent), str(HERE)]
try:
    import numpy as np

    import fake_planner  # noqa: E402
    import planner  # noqa: E402
    import planner_process  # noqa: E402
except ImportError:  # no numpy
    np = None

SPHERES = [(np.array([0.0, 0.01, 0.1]), 0.05)] if np is not None else []
Q0 = [0.1, -1.2, 1.3, -1.4, -1.5, 0.6]


@unittest.skipIf(np is None, "needs numpy: run with Isaac Sim's python.bat")
class OverAPipe(unittest.TestCase):
    """Kit's side and the planning process's loop, over a pipe in one process."""

    def setUp(self):
        fake_planner.FakePlanner.made.clear()
        kit_end, process_end = Pipe()
        self.server = threading.Thread(target=planner_process.serve, args=(process_end, fake_planner.FakePlanner), daemon=True)
        self.server.start()
        self.process = planner.PlannerProcess(kit_end)
        self.pl = planner.Planner(SPHERES, robot_dir=pathlib.Path("robots/ur10"), process=self.process)
        self.fake = fake_planner.FakePlanner.made[-1]

    def tearDown(self):
        self.process.close()
        self.server.join(timeout=2)

    def test_the_planner_in_the_process_gets_the_robot_and_the_world(self):
        self.assertEqual(self.fake.robot_dir, pathlib.Path("robots/ur10"))
        self.assertEqual([(c.tolist(), r) for c, r in self.fake.tool_spheres], [([0.0, 0.01, 0.1], 0.05)])
        pose = np.eye(4)
        pose[:3, 3] = [0.5, 0.2, 0.1]
        self.pl.add_cuboid("table", size=[1.0, 0.8, 0.04], centre=[0.75, 0.0, -0.02])
        self.pl.add_cuboid("object", size=np.array([0.1, 0.1, 0.2]), pose=pose)
        self.pl.set_enabled("table", False)
        self.assertEqual(self.fake.calls, [
            ("add_cuboid", "table", [1.0, 0.8, 0.04], [0.75, 0.0, -0.02], None),
            ("add_cuboid", "object", [0.1, 0.1, 0.2], None, pose.tolist()),
            ("set_enabled", "table", False),
        ])

    def test_a_plan_gives_the_planners_own_numbers_to_the_last_digit(self):
        q1 = np.array(Q0) + np.array([0.3, 0.1, -0.2, 0.05, 0.0, 1 / 7])
        p = self.pl.plan_joints(Q0, q1)
        want = fake_planner.FakePlan(Q0, q1)
        self.assertEqual(p.duration, want.duration)
        self.assertEqual(p.final().tobytes(), want.final().tobytes())
        for t in (0.0, 0.1, 1 / 9, 0.25, want.duration, 5.0):
            for got, expected in zip(p.at(t), want.at(t)):
                self.assertEqual(got.tobytes(), expected.tobytes(), t)
        self.assertEqual(self.pl.tool_pose(q1).tobytes(), self.fake.tool_pose(q1).tobytes())

    def test_a_goalset_says_which_target_was_chosen(self):
        poses = [np.eye(4), np.eye(4) * 2]
        self.assertEqual(self.pl.plan_goalset(Q0, poses).target_index, 1)

    def test_no_plan_is_none(self):
        self.assertIsNone(self.pl.plan_pose(Q0, np.eye(4)))

    def test_an_error_in_the_process_is_raised_here_and_the_process_goes_on(self):
        below = np.eye(4)
        below[2, 3] = -0.1
        with self.assertRaises(RuntimeError) as raised:
            self.pl.plan_line(Q0, below)
        self.assertIn("ValueError: below the table", str(raised.exception))
        self.assertIsNotNone(self.pl.plan_line(Q0, np.eye(4), deviation_m=0.01))

    def test_plans_and_planners_dropped_here_are_let_go_there(self):
        p = self.pl.plan_joints(Q0, Q0)
        self.assertIsNotNone(p.at(0.1))
        k = p._k
        del p
        self.assertEqual(self.process._dropped, [("plan", k)])
        self.pl.tool_pose(Q0)  # the next request takes the drops along
        self.assertEqual(self.process._dropped, [])
        other = planner.Planner([], robot_dir=pathlib.Path("robots/ur10"), process=self.process)
        del other
        self.assertEqual([kind for kind, _ in self.process._dropped], ["planner"])

    def test_a_plan_let_go_there_cannot_be_followed(self):
        p = self.pl.plan_joints(Q0, Q0)
        self.process._dropped.append(("plan", p._k))
        with self.assertRaises(RuntimeError) as raised:
            p.at(0.1)
        self.assertIn("KeyError", str(raised.exception))


@unittest.skipIf(np is None, "needs numpy: run with Isaac Sim's python.bat")
class WhenTheProcessEnds(unittest.TestCase):
    def test_kit_is_told_the_process_ended(self):
        kit_end, process_end = Pipe()
        process_end.close()
        process = planner.PlannerProcess(kit_end)
        with self.assertRaises(planner.PlannerProcessError) as raised:
            planner.Planner(SPHERES, robot_dir=pathlib.Path("r"), process=process)
        self.assertIn("ended", str(raised.exception))
        self.assertFalse(process.alive)


@unittest.skipIf(np is None, "needs numpy: run with Isaac Sim's python.bat")
class AStartedProcess(unittest.TestCase):
    """The process as Kit starts it (this Python, -I, the key, READY), serving the fake planner."""

    def test_it_plans_and_ends_when_closed(self):
        numpy_path = str(pathlib.Path(np.__file__).resolve().parents[1])
        process = planner.PlannerProcess.start(backend="fake_planner:FakePlanner", paths=[str(HERE), numpy_path])
        try:
            pl = planner.Planner(SPHERES, robot_dir=pathlib.Path("r"), process=process)
            p = pl.plan_joints(Q0, np.array(Q0) + 1)
            self.assertEqual(p.final().tobytes(), (np.array(Q0) + 1).tobytes())
            self.assertTrue(process.alive)
        finally:
            process.close()
        self.assertEqual(process.proc.returncode, 0)
        self.assertFalse(process.alive)

    def test_a_process_that_cannot_start_says_why(self):
        with self.assertRaises(planner.PlannerProcessError) as raised:
            planner.PlannerProcess.start(backend="no_such_module:Planner", paths=[])
        self.assertIn("No module named 'no_such_module'", str(raised.exception))


if __name__ == "__main__":
    unittest.main()
