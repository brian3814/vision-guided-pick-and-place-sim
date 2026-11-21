"""viewport_cameras.ViewportCameras with a fake viewport: no Isaac Sim needed.

    python -m unittest discover -s pick_place/tests
"""

import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from fake_viewport import WRIST_CAMERA, FakeViewport  # noqa: E402
from viewport_cameras import PERSPECTIVE, ViewportCameras  # noqa: E402

TOP = "/OmniverseKit_Top"


class ViewportCamerasTest(unittest.TestCase):
    def setUp(self):
        self.viewport = FakeViewport()
        self.cameras = ViewportCameras(self.viewport)

    def test_lists_kits_cameras_first_then_the_stages_by_name(self):
        self.assertEqual(self.cameras.state(), {
            "camera": PERSPECTIVE,
            "cameras": [{"path": PERSPECTIVE, "name": "Perspective"}, {"path": TOP, "name": "Top"},
                        {"path": "/OmniverseKit_Front", "name": "Front"}, {"path": "/OmniverseKit_Right", "name": "Right"},
                        {"path": WRIST_CAMERA, "name": "WristCamera"}]})

    def test_lists_only_the_cameras_the_stage_has(self):
        self.viewport.paths = [PERSPECTIVE]
        self.assertEqual(self.cameras.state()["cameras"], [{"path": PERSPECTIVE, "name": "Perspective"}])

    def test_switching_looks_through_the_camera(self):
        self.assertIsNone(self.cameras.set_camera(WRIST_CAMERA))
        self.assertEqual(self.viewport.told, [WRIST_CAMERA])
        self.assertEqual(self.cameras.state()["camera"], WRIST_CAMERA)

    def test_a_camera_the_stage_does_not_have_is_refused(self):
        self.assertEqual(self.cameras.set_camera("/World/nothing"), "the stage has no camera /World/nothing")
        self.assertEqual((self.viewport.told, self.viewport.looking), ([], PERSPECTIVE))

    def test_on_a_new_stage_with_the_camera_it_is_looked_through_again(self):
        self.cameras.set_camera(WRIST_CAMERA)
        self.viewport.new_stage()  # the viewport is back on Perspective
        self.assertEqual(self.cameras.state()["camera"], WRIST_CAMERA)
        self.assertEqual(self.viewport.told, [WRIST_CAMERA, WRIST_CAMERA])

    def test_on_a_new_stage_that_still_looks_through_it_nothing_is_done(self):
        self.cameras.set_camera(TOP)
        self.viewport.new_stage(looking=TOP)
        self.assertEqual(self.cameras.state()["camera"], TOP)
        self.assertEqual(self.viewport.told, [TOP])

    def test_on_a_new_stage_without_the_camera_perspective_is_looked_through(self):
        self.cameras.set_camera(WRIST_CAMERA)
        self.viewport.new_stage(extra=(), looking=WRIST_CAMERA)  # a path the new stage has no camera at
        self.assertEqual(self.cameras.state()["camera"], PERSPECTIVE)
        self.viewport.new_stage()  # and Perspective stays the one: the wrist camera is not taken up again
        self.assertEqual(self.cameras.state()["camera"], PERSPECTIVE)

    def test_a_new_stage_is_followed_when_switching_too(self):
        self.cameras.set_camera(TOP)
        self.viewport.new_stage(extra=())
        self.assertEqual(self.cameras.set_camera(WRIST_CAMERA), f"the stage has no camera {WRIST_CAMERA}")
        self.assertEqual(self.viewport.looking, TOP)

    def test_a_camera_chosen_in_kits_own_menu_is_the_one_kept_on_a_new_stage(self):
        self.cameras.set_camera(TOP)
        self.viewport.looking = WRIST_CAMERA  # Kit's camera menu, with Kit's UI shown
        self.assertEqual(self.cameras.state()["camera"], WRIST_CAMERA)
        self.viewport.new_stage()
        self.assertEqual(self.cameras.state()["camera"], WRIST_CAMERA)

    def test_without_a_viewport_no_camera_is_looked_through(self):
        self.viewport.looking = None
        self.assertIsNone(self.cameras.state()["camera"])


if __name__ == "__main__":
    unittest.main()
