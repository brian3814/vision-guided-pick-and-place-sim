"""The messages a web page gets in one session, kept in web/src/cell/contract.json for the page's tests
(web/src/cell/contract.test.ts): the real controller and bridge, with a fake cell and a fake viewport. This test fails
when what the bridge sends changes; then write the file again and see that the page's tests pass:

    python pick_place/tests/test_web_contract.py --write
    cd web && npm test

The session: hello; two cycles (one dropped, with a warning before it); another object; a start pose;
a cycle stopped, with a request refused while it runs and the camera switched; another gripper; a
cycle with nothing measured by the camera; a camera the stage does not have; an unknown action; hello
again, as a page that reloads.
"""

import asyncio
import itertools
import json
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from cell_control import CellControl  # noqa: E402
from fake_cell import LABELS, MENU, FakeCell  # noqa: E402
from fake_viewport import WRIST_CAMERA, FakeViewport  # noqa: E402
from viewport_cameras import ViewportCameras  # noqa: E402
from web_bridge import RESPONSE, WebBridge  # noqa: E402

CONTRACT = pathlib.Path(__file__).resolve().parents[2] / "web" / "src" / "cell" / "contract.json"


async def session() -> list[dict]:
    """What the page sends (`send`: a request's payload) and gets (`receive`: a message), in order."""
    steps: list[dict] = []
    cell = FakeCell()
    control = CellControl(cell, MENU, LABELS, "camera")
    clock = itertools.count(1_795_000_000)
    bridge = WebBridge(control, lambda kind, payload: steps.append({"receive": {"event_type": kind, "payload": payload}}),
                       clock=lambda: float(next(clock)), view=ViewportCameras(FakeViewport()))
    ids = itertools.count(1)

    def ask(action: str, **args) -> None:
        request = {"action": action, "request_id": f"r{next(ids)}", **args}
        steps.append({"send": request})
        answer = bridge.handle(action, {k: v for k, v in request.items() if k != "action"})
        steps.append({"receive": {"event_type": RESPONSE, "payload": answer}})

    async def done() -> None:
        await control.settled()

    await control.start()
    ask("hello")
    cell.outcomes, cell.warnings = ["placed", "dropped_lift"], ["the wrist camera had been moved in the stage (3.0 mm); put back on its mount"]
    ask("run", n=2)
    await done()
    ask("object", name="toy")
    await done()
    ask("pose", pose=7)
    await done()
    cell.gate = asyncio.Event()
    ask("run", n=5)
    await asyncio.sleep(0)
    ask("object", name="jar")  # refused: busy
    ask("camera", path=WRIST_CAMERA)  # taken: only what the stream shows
    ask("stop")
    cell.gate.set()
    await done()
    cell.gate = None
    ask("gripper", name="onrobot_rg2")
    await done()
    cell.overrides = {"est_centre_mm": None, "est_rot_deg": None, "detail": None}
    ask("run", n=1)
    await done()
    ask("camera", path="/World/nothing")
    ask("fly")
    ask("hello")
    bridge.close()
    return json.loads(json.dumps(steps))


def written() -> str:
    return json.dumps(asyncio.run(session()), indent=1) + "\n"


class ContractTest(unittest.TestCase):
    def test_the_page_tests_against_what_the_bridge_sends(self):
        self.assertTrue(CONTRACT.exists(), f"{CONTRACT} is missing: run this file with --write")
        self.assertEqual(json.loads(CONTRACT.read_text(encoding="utf-8")), json.loads(written()),
                         f"what the bridge sends changed: run {pathlib.Path(__file__).name} --write, then the page's tests")


if __name__ == "__main__":
    if "--write" in sys.argv:
        CONTRACT.parent.mkdir(parents=True, exist_ok=True)
        CONTRACT.write_text(written(), encoding="utf-8", newline="\n")
        print(f"wrote {CONTRACT}")
    else:
        unittest.main()
