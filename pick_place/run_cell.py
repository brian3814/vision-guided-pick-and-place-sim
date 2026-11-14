"""Run the pick-and-place cell: a UR10 with a gripper from the grasp matrix picks an object off
the table and puts it on the place spot, using that object's robust grasps from the grasp matrix.

Opens Isaac Sim's window so the cycles can be watched; --headless runs without it. Each cycle
starts with the object turned about the vertical and shifted a little (seeded), so the planner
may choose a different grasp each time. Results go to results/pick_place.sqlite.

--perception camera (step 4): the object's pose comes from the wrist camera instead of the
simulation, with the camera mount from the hand-eye calibration (handeye.py). --perception blind:
from neither; the robot takes the object to be at its nominal pose (the baseline for the camera).
To choose objects and run cycles from a window instead, see cell_app.py.

    C:/issacsim-6-2/python.bat pick_place/run_cell.py --object mug_A07RDPW843
    C:/issacsim-6-2/python.bat pick_place/run_cell.py --object mug_A07RDPW843 --cycles 10 --headless
    C:/issacsim-6-2/python.bat pick_place/run_cell.py --object mug_A07RDPW843 --perception camera
"""

import asyncio
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import cell_args  # noqa: E402

parser = cell_args.parser(__doc__, "results/pick_place.sqlite")
parser.add_argument("--cycles", type=int, default=3)
parser.add_argument("--start-cycle", type=int, default=0, help="first cycle index (each cycle's start pose is seeded by its index)")
parser.add_argument("--keep-open", action="store_true", help="with the window: keep it open after the last cycle")
parser.add_argument("--capture", help="with the window: save a picture of the viewport into this folder after each phase")
args, _ = parser.parse_known_args()
app = cell_args.start(args, headless=args.headless)

import cell_session  # noqa: E402
import gm_config  # noqa: E402
from cell_session import log  # noqa: E402


async def main() -> None:
    session = cell_session.CellSession(args)
    if not await session.build(args.object):
        return
    run_id = session.start_run(args.db)
    outcomes: dict[str, int] = {}
    try:
        for i in range(args.start_cycle, args.start_cycle + args.cycles):
            on_phase = None
            if args.capture and not args.headless:
                from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

                folder = gm_config.ROOT / args.capture
                folder.mkdir(parents=True, exist_ok=True)
                names = iter(range(100))

                def on_phase(name: str, cycle: int = i, names=names) -> None:
                    capture_viewport_to_file(get_active_viewport(), str(folder / f"cycle{cycle}_{next(names):02d}_{name}.png"))
            r = await session.run_cycle(i, f"{i + 1}/{args.start_cycle + args.cycles}", on_phase)
            outcomes[r["outcome"]] = outcomes.get(r["outcome"], 0) + 1
        session.finish_run()
    finally:
        if session.link is not None:
            session.link.close()
            session.link = None
    log(f"done: {outcomes} (cell run {run_id})")


task = asyncio.ensure_future(main())
while not task.done():
    app.update()
if task.exception():
    raise task.exception()
if args.keep_open and not args.headless:
    while app.is_running():
        app.update()
app.close()
