"""web_bridge.WebBridge with the real controller, a fake cell and a fake viewport: no Isaac Sim needed.

    python -m unittest discover -s pick_place/tests
"""

import asyncio
import json
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import web_bridge  # noqa: E402
from cell_control import BUSY, CellControl  # noqa: E402
from fake_cell import LABELS, MENU, FakeCell  # noqa: E402
from fake_viewport import WRIST_CAMERA, FakeViewport  # noqa: E402
from viewport_cameras import PERSPECTIVE, ViewportCameras  # noqa: E402
from web_bridge import EVENT, NO_VIEW, PROTOCOL, STATE, WebBridge  # noqa: E402


class BridgeTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.cell = FakeCell()
        self.control = CellControl(self.cell, MENU, LABELS, "camera")
        self.sent: list[tuple[str, dict]] = []
        self.clock = iter(range(1000, 100000))
        self.viewport = FakeViewport()
        self.bridge = WebBridge(self.control, lambda kind, payload: self.sent.append((kind, payload)),
                                clock=lambda: float(next(self.clock)), view=ViewportCameras(self.viewport))
        await self.control.start()

    def of(self, kind):
        return [payload for k, payload in self.sent if k == kind]

    async def test_hello_answers_with_the_events_so_far_and_the_state(self):
        answer = self.bridge.handle("hello", {"request_id": "r1"})
        self.assertEqual((answer["ok"], answer["protocol"], answer["request_id"]), (True, PROTOCOL, "r1"))
        self.assertEqual(answer["log"], [{"type": "heading", "gripper": "robotiq_2f_140", "object": "mug",
                                          "text": "2F-140 x mug", "at": 1000.0}])
        self.assertTrue(answer["state"]["status"].endswith("Ready."))

    async def test_events_go_to_the_page_stamped_and_are_kept_for_the_next_hello(self):
        self.cell.outcomes = ["placed", "dropped_lift"]
        self.assertTrue(self.bridge.handle("run", {"n": 2})["ok"])
        await self.control.settled()
        events = [p["event"] for p in self.of(EVENT)]
        self.assertEqual([(e["type"], e.get("pose")) for e in events], [("heading", None), ("result", 0), ("result", 1)])
        self.assertTrue(all("at" in e for e in events))
        self.assertEqual(self.bridge.handle("hello")["log"], events)

    async def test_state_goes_to_the_page_only_when_it_changed(self):
        states = [p["state"] for p in self.of(STATE)]
        self.assertTrue(states)
        self.assertTrue(all(a != b for a, b in zip(states, states[1:])))
        self.control._changed()  # the same state again
        self.assertEqual(len(self.of(STATE)), len(states))

    async def test_state_is_not_kept_in_the_log(self):
        self.bridge.handle("pose", {"pose": 3})
        await self.control.settled()
        self.assertEqual([e["type"] for e in self.bridge.handle("hello")["log"]], ["heading"])

    async def test_each_action_reaches_the_controller(self):
        self.assertTrue(self.bridge.handle("object", {"name": "toy"})["ok"])
        await self.control.settled()
        self.assertTrue(self.bridge.handle("gripper", {"name": "onrobot_rg2"})["ok"])
        await self.control.settled()
        self.assertTrue(self.bridge.handle("pose", {"pose": "5"})["ok"])
        await self.control.settled()
        self.cell.gate = asyncio.Event()
        self.assertTrue(self.bridge.handle("run", {"n": 3})["ok"])
        await asyncio.sleep(0)
        answer = self.bridge.handle("stop")
        self.assertTrue(answer["ok"])
        self.assertTrue(answer["state"]["stopping"])
        self.cell.gate.set()
        await self.control.settled()
        self.assertEqual([c for c in self.cell.calls if c[0] != "show"],
                         [("load_object", "toy"), ("build", "onrobot_rg2", "toy"), ("begin_run",),
                          ("cycle", 5, "start pose 5 (1/3)")])

    async def test_an_action_refused_says_why_with_the_state(self):
        self.cell.gate = asyncio.Event()
        self.bridge.handle("run", {"n": 1})
        answer = self.bridge.handle("object", {"name": "toy"})
        self.assertEqual((answer["ok"], answer["error"]), (False, BUSY))
        self.assertTrue(answer["state"]["busy"])
        self.cell.gate.set()
        await self.control.settled()

    async def test_a_request_handed_over_twice_runs_once(self):
        first = self.bridge.handle("run", {"n": 1, "request_id": "abc"})
        again = self.bridge.handle("run", {"n": 1, "request_id": "abc"})
        await self.control.settled()
        self.assertIs(first, again)
        self.assertTrue(first["ok"])
        self.assertEqual([c[0] for c in self.cell.calls].count("cycle"), 1)

    async def test_only_the_last_answers_are_kept(self):
        for k in range(web_bridge.ANSWERS_KEPT + 5):
            self.bridge.handle("hello", {"request_id": f"r{k}"})
        self.assertEqual(len(self.bridge._answered), web_bridge.ANSWERS_KEPT)
        self.assertNotIn("r0", self.bridge._answered)

    async def test_the_state_carries_the_viewports_cameras(self):
        for state in [p["state"] for p in self.of(STATE)] + [self.bridge.handle("hello")["state"]]:
            self.assertEqual(state["view"]["camera"], PERSPECTIVE)
            self.assertEqual([c["name"] for c in state["view"]["cameras"]],
                             ["Perspective", "Top", "Front", "Right", "WristCamera"])
            self.assertEqual(state["view"]["unit"], "m")

    async def test_the_camera_action_switches_the_viewports_camera(self):
        answer = self.bridge.handle("camera", {"path": WRIST_CAMERA})
        self.assertEqual((answer["ok"], answer["state"]["view"]["camera"]), (True, WRIST_CAMERA))
        self.assertEqual(self.viewport.told, [WRIST_CAMERA])
        answer = self.bridge.handle("camera", {"path": "/World/nothing"})
        self.assertEqual((answer["ok"], answer["error"]), (False, "the stage has no camera /World/nothing"))
        self.assertEqual(answer["state"]["view"]["camera"], WRIST_CAMERA)

    async def test_the_camera_can_be_switched_while_cycles_run(self):
        self.cell.gate = asyncio.Event()
        self.bridge.handle("run", {"n": 1})
        answer = self.bridge.handle("camera", {"path": WRIST_CAMERA})
        self.assertTrue(answer["ok"])
        self.assertTrue(answer["state"]["busy"])
        self.cell.gate.set()
        await self.control.settled()

    async def test_a_state_sent_after_another_stage_opened_shows_its_cameras(self):
        self.bridge.handle("camera", {"path": WRIST_CAMERA})
        self.viewport.new_stage(extra=())  # the cell built again, on a stage without the wrist camera
        self.control.set_pose(4)
        await self.control.settled()
        view = self.of(STATE)[-1]["state"]["view"]
        self.assertEqual((view["camera"], len(view["cameras"])), (PERSPECTIVE, 4))

    async def test_without_a_viewport_there_are_no_cameras_to_switch(self):
        bridge = WebBridge(self.control)
        self.assertEqual(bridge.handle("hello")["state"]["view"], NO_VIEW)
        answer = bridge.handle("camera", {"path": PERSPECTIVE})
        self.assertEqual((answer["ok"], answer["error"]), (False, "no viewport to switch the camera of"))
        bridge.close()

    async def test_requests_that_do_not_fit_are_answered_with_an_error(self):
        for action, args, error in [("fly", {}, "unknown action: 'fly'"),
                                    ("camera", {}, "camera: "),
                                    ("run", {}, "run: "),
                                    ("run", {"n": 1, "extra": 2}, "run: "),
                                    ("pose", {"pose": "three"}, "pose: ")]:
            answer = self.bridge.handle(action, args)
            self.assertFalse(answer["ok"], action)
            self.assertTrue(answer["error"].startswith(error), answer["error"])
            self.assertIn("state", answer)
        await self.control.settled()
        self.assertEqual([c for c in self.cell.calls if c[0] == "cycle"], [])

    async def test_everything_sent_is_json(self):
        self.bridge.handle("run", {"n": 1})
        await self.control.settled()
        for kind, payload in self.sent:
            json.dumps(payload)
        json.dumps(self.bridge.handle("hello"))

    async def test_after_close_nothing_is_sent(self):
        self.bridge.close()
        self.sent.clear()
        self.control.set_pose(7)
        await self.control.settled()
        self.assertEqual(self.sent, [])

    async def test_a_page_that_connects_late_gets_the_last_events(self):
        for k in range(web_bridge.LOG_KEPT + 3):
            self.bridge._on_event({"type": "warning", "text": str(k)})
        log = self.bridge.handle("hello")["log"]
        self.assertEqual(len(log), web_bridge.LOG_KEPT)
        self.assertEqual(log[-1]["text"], str(web_bridge.LOG_KEPT + 2))


if __name__ == "__main__":
    unittest.main()
