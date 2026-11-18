"""Command-line options and start-up shared by run_cell.py (a batch of cycles on one object) and
cell_app.py (a window to choose the object and run cycles one at a time). Plain Python: the options
are read before Isaac Sim starts, since one of them decides whether it opens a window."""

import argparse
import os
import sys


def parser(description: str, db_default: str, headless_option: bool = True) -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=description, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--gripper", default="robotiq_2f_140")
    p.add_argument("--object", default="toy_block_6MPL7NVFVU")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--max-yaw-deg", type=float, default=90.0, help="the object starts turned by up to this much")
    p.add_argument("--jitter-m", type=float, default=0.03, help="and shifted by up to this much in x and y")
    p.add_argument("--speed", type=float, default=0.5, help="play planned motions at this fraction of their planned speed")
    if headless_option:
        p.add_argument("--headless", action="store_true")
    p.add_argument("--matrix-db", default="results/grasp_matrix.sqlite")
    p.add_argument("--db", default=db_default)
    p.add_argument("--perception", choices=["truth", "camera", "ros", "blind"], default="truth",
                   help="where the object's pose comes from: the simulation, the wrist camera, or the wrist camera "
                        "through the ROS 2 object_locator node (step 5; start it first, see ros2_ws/README.md); "
                        "blind: nowhere, the robot takes the object to be at its nominal pose (not turned or shifted), "
                        "the baseline that shows what the camera adds (from the look pose, no slip correction, as with "
                        "the camera)")
    p.add_argument("--depth-noise", default="1",
                   help="camera: depth noise as a multiple of camera_model's defaults, or 'perfect'")
    p.add_argument("--handeye", default="auto",
                   help="camera: the hand-eye calibration to use: 'auto', the gripper's own (handeye.py --gripper writes "
                        "it; results/perception/handeye.json for the 2F-140); a file, for --gripper; or 'none' for the "
                        "mount as built")
    p.add_argument("--no-slip-correction", action="store_true",
                   help="lower as planned instead of correcting for slip in the hand (always so with the camera)")
    p.add_argument("--look-first", action="store_true",
                   help="truth: move to the camera's look pose before choosing a grasp, with the camera's housing in "
                        "the planner's model, as in the camera mode (to compare the two on equal terms)")
    return p


def start(args, headless: bool):
    """Start Isaac Sim with the extensions the cell needs. Returns the SimulationApp; the cell's
    modules (cell, controller, planner, cell_session, ...) can be imported after this."""
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": headless})
    from isaacsim.core.experimental.utils.app import enable_extension

    enable_extension("isaacsim.robot_motion.cumotion")
    if args.perception in ("camera", "ros"):
        enable_extension("isaacsim.sensors.experimental.rtx")
    if args.perception == "ros":
        # Isaac Sim's default middleware on Windows is Zenoh; ROS 2 Jazzy for Windows speaks Fast DDS.
        if os.environ.get("RMW_IMPLEMENTATION") != "rmw_fastrtps_cpp":
            app.close()
            sys.exit("--perception ros: set RMW_IMPLEMENTATION=rmw_fastrtps_cpp before starting Isaac Sim "
                     f"(it is {os.environ.get('RMW_IMPLEMENTATION')!r})")
        enable_extension("isaacsim.ros2.bridge")
    return app
