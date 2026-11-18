"""cell_control.CellControl with a fake cell: no Isaac Sim needed.

    python -m unittest discover -s pick_place/tests
"""

import asyncio
import contextlib
import io
import json
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import cell_control  # noqa: E402
from cell_control import BUSY, CellControl  # noqa: E402
from fake_cell import LABELS, MENU, FakeCell  # noqa: E402


class ControlTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.cell = FakeCell()
        self.control = CellControl(self.cell, MENU, LABELS, "camera")
        self.events: list[dict] = []
        self.control.subscribe(self.events.append)
        await self.control.start()
        self.events.clear()
        self.cell.calls.clear()

    def kinds(self):
        return [e["type"] for e in self.events if e["type"] != "state"]

    def last_state(self):
        return [e for e in self.events if e["type"] == "state"][-1]["state"]

    async def test_start_heads_the_results_and_shows_the_first_start(self):
        cell = FakeCell()
        control = CellControl(cell, MENU, LABELS, "camera", pose=3)
        events = []
        control.subscribe(events.append)
        await control.start()
        self.assertEqual(events[0], {"type": "heading", "gripper": "robotiq_2f_140", "object": "mug", "text": "2F-140 x mug"})
        self.assertEqual(cell.calls, [("show", 3)])
        self.assertEqual(control.status, "2F-140 x mug (12 robust grasps), start pose 3: turned +30 deg at (0.750, -0.350). Ready.")

    async def test_state_lists_the_choices_with_their_texts(self):
        self.cell.unusable["robotiq_hand_e"] = "no hand-eye calibration for robotiq_hand_e"
        state = self.control.state()
        self.assertEqual(state["grippers"][0], {"name": "robotiq_2f_140", "text": "2F-140   3 objects", "usable": True})
        self.assertEqual(state["grippers"][2], {"name": "robotiq_hand_e", "text": "HAND-E   1 objects   (no hand-eye calibration)",
                                                "usable": False})
        self.assertEqual([o["text"] for o in state["objects"]],
                         ["jar   3 grasps", "mug   12 grasps   (batch-tested)", "toy   40 grasps   (batch-tested)"])
        self.assertEqual(state["info"], "The object's pose from the wrist camera.")
        json.dumps(state)

    async def test_choosing_an_object_swaps_it_and_shows_its_start(self):
        self.assertIsNone(self.control.choose_object("toy"))
        await self.control.settled()
        self.assertEqual(self.cell.calls, [("load_object", "toy"), ("show", 0)])
        self.assertEqual(self.kinds(), ["heading"])
        self.assertEqual(self.last_state()["object"], "toy")

    async def test_choosing_the_same_object_does_nothing(self):
        self.control.choose_object("mug")
        await self.control.settled()
        self.assertEqual(self.cell.calls, [])

    async def test_an_object_not_offered_is_refused(self):
        self.assertEqual(self.control.choose_object("valve"), "valve is not offered with the 2F-140")

    async def test_a_new_gripper_keeps_the_object_it_has_grasps_for(self):
        self.control.choose_object("toy")
        await self.control.settled()
        self.cell.calls.clear()
        self.control.choose_gripper("onrobot_rg2")
        await self.control.settled()
        self.assertEqual(self.cell.calls, [("build", "onrobot_rg2", "toy"), ("show", 0)])

    async def test_a_new_gripper_without_grasps_for_the_object_takes_its_first(self):
        self.control.choose_gripper("onrobot_rg2")
        await self.control.settled()
        self.assertEqual(self.cell.calls[0], ("build", "onrobot_rg2", "toy"))
        self.assertEqual([o["name"] for o in self.last_state()["objects"]], ["toy", "valve"])
        statuses = [e["state"]["status"] for e in self.events if e["type"] == "state"]
        self.assertEqual(statuses[0], "Building the cell with the RG2...")
        busy = [e["state"]["busy"] for e in self.events if e["type"] == "state"]
        self.assertEqual(busy[:2], [True, False])

    async def test_a_gripper_without_calibration_is_refused_with_why(self):
        self.cell.unusable["robotiq_hand_e"] = "no hand-eye calibration for robotiq_hand_e"
        self.control.choose_gripper("robotiq_hand_e")
        await self.control.settled()
        self.assertEqual(self.cell.calls, [])
        self.assertEqual(self.last_state()["status"], "no hand-eye calibration for robotiq_hand_e")
        self.assertEqual(self.last_state()["gripper"], "robotiq_2f_140")

    async def test_with_ros_the_gripper_stays(self):
        control = CellControl(FakeCell(), MENU, LABELS, "ros")
        self.assertTrue(control.state()["fixed_gripper"])
        self.assertIn("stays as started", control.choose_gripper("onrobot_rg2"))
        self.assertIsNone(control.choose_gripper("robotiq_2f_140"))
        await control.settled()

    async def test_a_new_start_pose_is_shown_once(self):
        self.control.set_pose(4)
        self.control.set_pose(4)
        await self.control.settled()
        self.assertEqual(self.cell.calls, [("show", 4)])

    async def test_cycles_run_from_the_start_pose_on_and_are_reported(self):
        self.control.set_pose(2)
        await self.control.settled()
        self.cell.calls.clear()
        self.cell.outcomes = ["placed", "tracking"]
        self.assertIsNone(self.control.run(2))
        self.assertTrue(self.control.state()["busy"])
        await self.control.settled()
        self.assertEqual(self.cell.calls, [("begin_run",), ("cycle", 2, "start pose 2 (1/2)"), ("cycle", 3, "start pose 3 (2/2)"),
                                           ("show", 4)])
        results = [e for e in self.events if e["type"] == "result"]
        self.assertEqual([(r["pose"], r["outcome"], r["ok"]) for r in results], [(2, "placed", True), (3, "tracking", False)])
        self.assertEqual(results[0]["text"], "   2   placed, 4.2 mm from the target; found 0.6 mm / 0.8 deg off; grasp 116")
        self.assertEqual(results[1]["text"], "   3   tracking, 4.2 mm from the target; found 0.6 mm / 0.8 deg off; grasp 116"
                                             "  (approach: tool 7.0 mm off)")
        self.assertIs(type(results[0]["place_error_m"]), float)  # a plain number, not the cell's own type
        json.dumps(self.events)
        state = self.control.state()
        self.assertEqual((state["pose"], state["busy"]), (4, False))
        self.assertTrue(state["status"].endswith("Ready."))

    async def test_a_single_cycle_is_labelled_by_its_pose_alone(self):
        self.control.run(1)
        await self.control.settled()
        self.assertEqual(self.cell.calls[1], ("cycle", 0, "start pose 0"))

    async def test_the_cycle_log_becomes_the_status(self):
        self.control.run(1)
        await self.control.settled()
        statuses = [e["state"]["status"] for e in self.events if e["type"] == "state"]
        self.assertIn("cycle start pose 0: object turned", statuses)

    async def test_actions_are_refused_while_cycles_run(self):
        self.cell.gate = asyncio.Event()
        self.control.run(5)
        await asyncio.sleep(0)
        for problem in (self.control.run(1), self.control.choose_object("toy"), self.control.choose_gripper("onrobot_rg2"),
                        self.control.set_pose(9)):
            self.assertEqual(problem, BUSY)
        self.cell.gate.set()
        await self.control.settled()

    async def test_stop_ends_the_run_after_the_cycle(self):
        self.cell.gate = asyncio.Event()
        self.control.run(5)
        await asyncio.sleep(0)
        self.assertIsNone(self.control.stop())
        self.assertTrue(self.control.status.endswith("(stopping after this cycle)"))
        self.cell.gate.set()
        await self.control.settled()
        self.assertEqual([c[0] for c in self.cell.calls], ["begin_run", "cycle", "show"])
        self.assertEqual(self.control.stop(), "nothing runs")

    async def test_a_failing_cycle_says_why_and_leaves_the_window_usable(self):
        self.cell.fail_cycle = RuntimeError("the simulation was stopped")
        with contextlib.redirect_stderr(io.StringIO()) as printed:  # the traceback, for the log
            self.control.run(3)
            await self.control.settled()
        self.assertIn("RuntimeError: the simulation was stopped", printed.getvalue())
        self.assertEqual([c[0] for c in self.cell.calls], ["begin_run", "cycle"])  # no show after an error
        state = self.control.state()
        self.assertEqual((state["status"], state["busy"]), ("error: the simulation was stopped", False))
        self.cell.fail_cycle = None
        self.assertIsNone(self.control.run(1))
        await self.control.settled()

    async def test_what_guard_put_back_comes_before_the_result(self):
        self.cell.warnings = ["the wrist camera had been moved in the stage (40.0 mm); put back on its mount"]
        self.control.run(1)
        await self.control.settled()
        self.assertEqual(self.kinds(), ["warning", "result"])

    async def test_a_view_that_fails_does_not_stop_the_others(self):
        def broken(_event):
            raise ValueError("a broken view")
        seen = []
        self.control.subscribe(broken)
        self.control.subscribe(seen.append)
        with contextlib.redirect_stderr(io.StringIO()):
            self.control.choose_object("toy")
            await self.control.settled()
        self.assertTrue(seen)

    async def test_unsubscribed_listeners_get_nothing(self):
        seen = []
        unsubscribe = self.control.subscribe(seen.append)
        unsubscribe()
        self.control.set_pose(1)
        await self.control.settled()
        self.assertEqual(seen, [])


class ResultTextTest(unittest.TestCase):
    def test_a_failure_before_placing_says_why(self):
        self.assertEqual(cell_control.result_text(12, {"outcome": "dropped_lift", "detail": "object rose 0 mm"}),
                         "  12   dropped_lift  (object rose 0 mm)")


if __name__ == "__main__":
    unittest.main()
