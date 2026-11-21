"""A fake viewport for the tests: viewport_cameras.Viewport without Isaac Sim, recording what it was told."""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from viewport_cameras import KIT_CAMERAS, PERSPECTIVE  # noqa: E402

WRIST_CAMERA = "/World/UR10/wrist_3_link/WristCamera"


class FakeViewport:
    """A viewport looking through Perspective, on a stage with Kit's cameras and `extra` ones."""

    def __init__(self, extra=(WRIST_CAMERA,)):
        self.key = 1
        self.paths = [*extra, *KIT_CAMERAS]  # Kit's own last, as a stage lists its session layer's prims
        self.looking: str | None = PERSPECTIVE
        self.told: list[str] = []  # the paths it was told to look through

    def stage(self):
        return self.key

    def cameras(self):
        return list(self.paths)

    def camera(self):
        return self.looking

    def look_through(self, path):
        self.told.append(path)
        self.looking = path

    def new_stage(self, extra=(WRIST_CAMERA,), looking: str | None = PERSPECTIVE) -> None:
        """Another stage opened, with these cameras; the viewport looks through `looking` on it."""
        self.key += 1
        self.paths = [*extra, *KIT_CAMERAS]
        self.looking = looking
