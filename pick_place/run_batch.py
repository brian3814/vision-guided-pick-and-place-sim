"""Run the pick-and-place cell headless on several objects, one Isaac Sim process per object, one
after another, then write the report.

    .venv/Scripts/python.exe pick_place/run_batch.py --objects toy_block_6MPL7NVFVU,mug_A07RDPW843 --cycles 6

Options other than those below (--gripper, --seed, --speed, ...) go to run_cell.py.
"""

import argparse
import os
import pathlib
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")  # the report has non-ASCII; stdout is often redirected to a log
    parser = argparse.ArgumentParser(description="Run the pick-and-place cell on several objects, then report.")
    parser.add_argument("--objects", required=True, help="comma-separated object names")
    parser.add_argument("--cycles", type=int, default=6)
    parser.add_argument("--isaac-sim", default=os.environ.get("ISAAC_SIM_DIR", r"C:\issacsim-6-2"))
    parser.add_argument("--db", default="results/pick_place.sqlite")
    parser.add_argument("--log-tag", default="", help="added to the log names (default: the --perception given, if any)")
    args, cell_args = parser.parse_known_args()
    tag = args.log_tag or (cell_args[cell_args.index("--perception") + 1] if "--perception" in cell_args else "")

    isaac_python = pathlib.Path(args.isaac_sim) / "python.bat"
    env = dict(os.environ, OMNI_KIT_ACCEPT_EULA="YES")
    failed = []
    for obj in args.objects.split(","):
        log_path = ROOT / f"results/logs/cell_{obj}{'_' + tag if tag else ''}.log"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        cmd = ["cmd", "/c", str(isaac_python), str(ROOT / "pick_place/run_cell.py"), "--object", obj,
               "--cycles", str(args.cycles), "--headless", "--db", args.db, *cell_args]
        t0 = time.perf_counter()
        with log_path.open("w", encoding="utf-8", errors="replace") as log:
            subprocess.run(cmd, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
        # Isaac Sim's exit code is not reliable; the run's own last line says whether it finished.
        lines = [ln for ln in log_path.read_text(encoding="utf-8", errors="replace").splitlines() if ln.startswith("[cell] ")]
        done = [ln for ln in lines if ln.startswith("[cell] done:") or "nothing to do" in ln]
        print(f"{obj}: {done[-1][7:] if done else 'DID NOT FINISH, see ' + str(log_path)} ({time.perf_counter() - t0:.0f} s)",
              flush=True)
        if not done:
            failed.append(obj)

    report = subprocess.run([sys.executable, str(ROOT / "pick_place/report.py"), "--db", args.db], cwd=ROOT,
                            capture_output=True, text=True, encoding="utf-8", env=dict(os.environ, PYTHONIOENCODING="utf-8"))
    print(report.stdout[-3000:] if report.returncode == 0 else f"report.py failed:\n{report.stderr}")
    if failed:
        sys.exit(f"did not finish: {', '.join(failed)}")


if __name__ == "__main__":
    main()
