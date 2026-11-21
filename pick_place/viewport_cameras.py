"""The viewport's cameras, for the web page: which cameras the stage has, which one the viewport
looks through, and switching to another, as Isaac Sim's camera menu does (the page has no Kit menus).

The cameras are Kit's own four (Perspective, Top, Front, Right) and every camera in the stage that
Kit shows (with --perception camera or ros, the wrist camera). Switching only changes what the
viewport, and so the stream, shows: the wrist camera's images for perception come from a render
product of their own.

Changing the gripper builds the cell on a new stage. The viewport then goes on looking through the
camera it looked through, if the new stage has it, and through Perspective if not.

The state also names the stage's unit of length (its metersPerUnit), for the page's legend: as
Kit's unit box at the viewport's bottom left names it (omni.usd.metrics.assembler.ui), or the
metres per unit when it is none of the units Kit names. The legend says it in words, so the stream
leaves the box out (KitViewport.hide_unit_box).

ViewportCameras knows nothing of Kit: the viewport comes in through `Viewport`. KitViewport is
Isaac Sim's active viewport; it imports Kit only when made.
"""

from __future__ import annotations

from typing import Protocol

PERSPECTIVE = "/OmniverseKit_Persp"
KIT_CAMERAS = {PERSPECTIVE: "Perspective", "/OmniverseKit_Top": "Top", "/OmniverseKit_Front": "Front",
               "/OmniverseKit_Right": "Right"}
# The units Kit's unit box names, with their metres per unit, and how near a stage's must be (a share of it).
UNITS = {"mm": 0.001, "cm": 0.01, "dm": 0.1, "m": 1.0, "km": 1000.0, "in": 0.0254, "ft": 0.3048, "mi": 1609.34}
UNIT_TOLERANCE = 0.02


def unit_name(meters_per_unit: float | None) -> str | None:
    """The name of the unit of length `meters_per_unit` metres long ("m"); None without a stage."""
    if meters_per_unit is None:
        return None
    for name, meters in UNITS.items():
        if abs(meters_per_unit - meters) < meters * UNIT_TOLERANCE:
            return name
    return f"{meters_per_unit:g} m"


class Viewport(Protocol):
    """What ViewportCameras needs of the viewport."""

    def stage(self) -> object: ...  # which stage it shows: changes when another stage opens

    def cameras(self) -> list[str]: ...  # the paths of the stage's cameras, Kit's own included

    def camera(self) -> str | None: ...  # the path of the camera it looks through (None: no viewport)

    def look_through(self, path: str) -> None: ...

    def meters_per_unit(self) -> float | None: ...  # the stage's metersPerUnit (None: no stage)


class ViewportCameras:
    """The viewport's cameras, and switching between them; and the unit of the stage it shows."""

    def __init__(self, viewport: Viewport):
        self.viewport = viewport
        self._stage = viewport.stage()
        self._looked_through = viewport.camera()  # on that stage, last seen

    def state(self) -> dict:
        """The cameras, each with its path and the name the page shows (Kit's four first, as Kit's
        menu lists them), the path of the camera the viewport looks through, and the stage's unit."""
        self._follow_stage()
        paths = self.viewport.cameras()
        cameras = [{"path": p, "name": name} for p, name in KIT_CAMERAS.items() if p in paths]
        cameras += [{"path": p, "name": p.rsplit("/", 1)[-1]} for p in paths if p not in KIT_CAMERAS]
        return {"camera": self.viewport.camera(), "cameras": cameras, "unit": unit_name(self.viewport.meters_per_unit())}

    def set_camera(self, path: str) -> str | None:
        """Look through the camera at `path`. None when done, or why not."""
        self._follow_stage()
        if path not in self.viewport.cameras():
            return f"the stage has no camera {path}"
        self.viewport.look_through(path)
        self._looked_through = path
        return None

    def _follow_stage(self) -> None:
        """On a new stage, look through the camera looked through on the one before, if this one has
        it, else Perspective. On the same stage, remember what the viewport looks through (also a
        camera chosen in Kit's own menu, when Kit's UI is shown)."""
        stage = self.viewport.stage()
        if stage == self._stage:
            self._looked_through = self.viewport.camera()
            return
        self._stage = stage
        path = self._looked_through if self._looked_through in self.viewport.cameras() else PERSPECTIVE
        if self.viewport.camera() != path:
            self.viewport.look_through(path)
        self._looked_through = path


class KitViewport:
    """Isaac Sim's active viewport and the stage it shows. Make it inside Isaac Sim."""

    def __init__(self):
        import omni.usd
        from omni.kit.viewport.utility import get_active_viewport

        self._context = omni.usd.get_context()
        self._active = get_active_viewport

    def stage(self) -> object:
        return self._context.get_stage_id()

    def cameras(self) -> list[str]:
        """Every camera prim, as Kit's camera menu lists them: not those hidden from the UI, but Kit's own."""
        import omni.usd
        from pxr import UsdGeom

        stage = self._context.get_stage()
        if stage is None:
            return []
        return [str(prim.GetPath()) for prim in stage.Traverse()
                if prim.IsA(UsdGeom.Camera) and (str(prim.GetPath()) in KIT_CAMERAS or not omni.usd.editor.is_hide_in_ui(prim))]

    def camera(self) -> str | None:
        viewport = self._active()
        return None if viewport is None else str(viewport.camera_path)

    def look_through(self, path: str) -> None:
        viewport = self._active()
        if viewport is not None:
            viewport.camera_path = path

    def meters_per_unit(self) -> float | None:
        from pxr import UsdGeom

        stage = self._context.get_stage()
        return None if stage is None else UsdGeom.GetStageMetersPerUnit(stage)

    def hide_unit_box(self) -> bool:
        """Hide Kit's unit box (the 'm' at the viewport's bottom left) through its viewport layer, as
        Kit hides the viewport's axis and menu bar when its UI is hidden. Not through the box's own
        setting: Kit keeps that for later starts, those with a window too. The layer stays hidden on
        a new stage. False when the viewport has no unit box."""
        from omni.kit.viewport.utility import get_active_viewport_window

        window = get_active_viewport_window()
        # Kit's own name for the layer lookup that its hidden UI uses; no public one.
        layer = None if window is None else window._find_viewport_layer("MetricsAssembler", "hud")
        if layer is None:
            return False
        layer.visible = False
        return True
