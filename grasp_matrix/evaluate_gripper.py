"""Evaluate a new or changed gripper against the object set, in one command.

1. Calibrate it with tools/calibrate_grippers.py, unless it is calibrated already (--recalibrate
   to redo it). Stops if a calibration check fails; warnings are printed and also appear in the
   report.
2. Run its gripper x object pairs with run_matrix.py. Pairs already done with the current
   settings are skipped, so a pilot on a few objects is never run twice.
3. Write the report with report.py, which compares it with every other gripper in the database.

Add the gripper to grasp_matrix/config/grippers.json first (see "Adding a gripper" in the README).
Options other than those below (--objects, --workers, --candidates, ...) go to run_matrix.py.

    .venv/Scripts/python.exe grasp_matrix/evaluate_gripper.py --gripper onrobot_rg2 --objects toy_block_RBP7LFVPZC
    .venv/Scripts/python.exe grasp_matrix/evaluate_gripper.py --gripper onrobot_rg2
"""

import argparse
import os
import pathlib
import subprocess
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import gm_config  # noqa: E402

ROOT = gm_config.ROOT


def calibrate(isaac_python: pathlib.Path, gripper: str) -> bool:
    """Run the calibration for one gripper; True if every error check passed. Isaac Sim's exit code
    is not reliable, so this reads the calibration's own verdict from its output."""
    log_path = ROOT / f"results/logs/calibrate_{gripper}.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    cmd = ["cmd", "/c", str(isaac_python), str(ROOT / "tools/calibrate_grippers.py"), "--grippers", gripper]
    with log_path.open("w", encoding="utf-8", errors="replace") as log:
        subprocess.run(cmd, cwd=ROOT, env=dict(os.environ, OMNI_KIT_ACCEPT_EULA="YES"), stdout=log, stderr=subprocess.STDOUT)
    lines = [ln for ln in log_path.read_text(encoding="utf-8", errors="replace").splitlines()
             if ln.startswith(f"[calib] {gripper}:") and "gap curve" not in ln]
    for ln in lines:
        print("  " + ln)
    ok = any(ln.startswith(f"[calib] {gripper}: OK") for ln in lines)
    if not ok:
        print(f"  calibration failed or did not finish; see {log_path}")
    return ok


def main() -> None:
    parser = argparse.ArgumentParser(description="Calibrate a gripper, run its grasp trials and update the report.")
    parser.add_argument("--gripper", required=True, help="a name from grasp_matrix/config/grippers.json")
    parser.add_argument("--recalibrate", action="store_true", help="calibrate again even if already calibrated")
    parser.add_argument("--isaac-sim", default=os.environ.get("ISAAC_SIM_DIR", r"C:\issacsim-6-2"))
    parser.add_argument("--db", default="results/grasp_matrix.sqlite")
    args, matrix_args = parser.parse_known_args()

    if args.gripper not in gm_config.gripper_config():
        sys.exit(f"{args.gripper} is not in grasp_matrix/config/grippers.json; add it first (see the README)")
    usd = ROOT / gm_config.gripper_config()[args.gripper]["usd"]
    if not usd.exists():
        sys.exit(f"asset not found: {usd}")
    isaac_python = pathlib.Path(args.isaac_sim) / "python.bat"

    print(f"1. calibration of {args.gripper}")
    if args.recalibrate or args.gripper not in gm_config.load_calibration():
        if not calibrate(isaac_python, args.gripper):
            sys.exit(1)
    else:
        warnings = gm_config.load_calibration()[args.gripper].get("checks", {}).get("warnings", [])
        print("  already calibrated (--recalibrate to redo)" + "".join(f"\n  WARNING {w}" for w in warnings))

    print("2. grasp trials")
    matrix = [sys.executable, str(ROOT / "grasp_matrix/run_matrix.py"), "--grippers", args.gripper, "--db", args.db,
              "--isaac-sim", args.isaac_sim, *matrix_args]
    if subprocess.run(matrix, cwd=ROOT).returncode != 0:
        sys.exit("run_matrix.py failed")

    print("3. report")
    report = subprocess.run([sys.executable, str(ROOT / "grasp_matrix/report.py"), "--db", args.db], cwd=ROOT,
                            capture_output=True, text=True, encoding="utf-8",
                            env=dict(os.environ, PYTHONIOENCODING="utf-8"))  # the report prints non-ASCII marks
    if report.returncode != 0:
        sys.exit(f"report.py failed:\n{report.stderr}")
    print("  written to results/summary.md and results/summary_pairs.csv")


if __name__ == "__main__":
    main()
