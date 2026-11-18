"""A fake cell for the tests: cell_control.Cell without Isaac Sim, recording what was asked of it."""

import asyncio

MENU = {
    "robotiq_2f_140": (["jar", "mug", "toy"], {"jar": 3, "mug": 12, "toy": 40}, {"mug", "toy"}),
    "onrobot_rg2": (["toy", "valve"], {"toy": 7, "valve": 2}, set()),
    "robotiq_hand_e": (["mug"], {"mug": 5}, set()),
}
LABELS = {"robotiq_2f_140": "2F-140", "onrobot_rg2": "RG2", "robotiq_hand_e": "HAND-E"}


class Scalar(float):
    """Stands in for a numpy scalar (numpy.float64 is a float with item())."""

    def item(self):
        return float(self)


class FakeCell:
    """The cell as cell_control.Cell, recording what was asked of it."""

    def __init__(self, gripper="robotiq_2f_140", object_name="mug"):
        self.gripper, self.object_name = gripper, object_name
        self.calls: list[tuple] = []
        self.unusable: dict[str, str] = {}
        self.outcomes: list[str] = []  # the next cycles' outcomes (default: placed)
        self.warnings: list[str] = []  # what the next guard() puts back
        self.fail_cycle: Exception | None = None  # an exception the next cycle raises
        self.gate: asyncio.Event | None = None  # a cycle waits for it

    @property
    def grasp_count(self):
        return MENU[self.gripper][1][self.object_name]

    def cannot_use(self, gripper):
        return self.unusable.get(gripper)

    async def load_object(self, name):
        self.calls.append(("load_object", name))
        self.object_name = name
        return True

    async def build(self, gripper, name):
        self.calls.append(("build", gripper, name))
        self.gripper, self.object_name = gripper, name
        return True

    async def show(self, pose):
        self.calls.append(("show", pose))
        return 10.0 * pose, (0.75, -0.35)

    async def begin_run(self):
        self.calls.append(("begin_run",))

    def guard(self):
        warnings, self.warnings = self.warnings, []
        return warnings

    async def run_cycle(self, pose, label, on_log):
        self.calls.append(("cycle", pose, label))
        on_log(f"[cell] cycle {label}: object turned")
        if self.gate is not None:
            await self.gate.wait()
        if self.fail_cycle is not None:
            raise self.fail_cycle
        outcome = self.outcomes.pop(0) if self.outcomes else "placed"
        return {"outcome": outcome, "place_error_m": Scalar(0.0042), "tilt_deg": 0.1, "est_centre_mm": 0.6,
                "est_rot_deg": 0.8, "grasp_index": 116, "detail": "approach: tool 7.0 mm off", "cycle_s": 15.2}
