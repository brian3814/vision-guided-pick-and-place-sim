"""Measure the perception offline, on captures from capture_views.py: for each capture, the depth
as the stereo camera would give it (camera_model.py), the pose perception.locate() finds in it, and
that pose's error against the true pose. No Isaac Sim needed beyond its Python and packages.

    C:/issacsim-6-2/python.bat pick_place/perception_eval.py
    C:/issacsim-6-2/python.bat pick_place/perception_eval.py --objects mug_A07RDPW843 --noise perfect 1 2 --write

--noise: depth noise levels, as multiples of camera_model's defaults ("perfect": the renderer's
depth untouched). --write saves the summary to results/perception_summary.md. --overlay DIR saves
each RGB image with the estimated model's visible points drawn on it in green.
"""

import argparse
import pathlib
import statistics
import sys
import time

# numpy and scipy: Isaac Sim's bundled copies (python.bat does not put them on the path)
_ISAAC = pathlib.Path(sys.executable).resolve().parents[2]
for _pattern in ("extscache/omni.kit.pip_archive-*/pip_prebundle", "exts/omni.pip.compute/pip_prebundle"):
    sys.path += [str(d) for d in sorted(_ISAAC.glob(_pattern))]
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import camera_model  # noqa: E402
import perception  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[1]
PICK_XY = (0.75, -0.35)  # cell.PICK_XY, for captures saved without it


def load_capture(path: pathlib.Path) -> dict:
    d = dict(np.load(path))
    fx, fy, cx, cy, w, h = d["intrinsics"]
    d["intr"] = perception.Intrinsics(float(fx), float(fy), float(cx), float(cy), int(w), int(h))
    d["object"] = path.stem.rsplit("_", 1)[0]
    return d


def camera_depth(cap: dict, noise: str) -> np.ndarray:
    if noise == "perfect":
        return cap["depth"].astype(float)
    rng = np.random.default_rng([int(cap["seed"]), int(cap["cycle"]), 7])
    return camera_model.simulate_depth(cap["depth"], cap["intr"].fx, rng, scale=float(noise))


def save_overlay(path: pathlib.Path, cap: dict, model: perception.ObjectModel, pose: np.ndarray | None) -> None:
    """The RGB image with the model's points drawn where the estimate puts them (visible side only)."""
    from PIL import Image

    img = cap["rgb"].copy()
    if pose is not None:
        intr = cap["intr"]
        T_cam = perception.invert(cap["T_world_cam"])
        pc = perception.transform(T_cam @ pose, model.points)
        normals = model.normals @ (T_cam @ pose)[:3, :3].T
        facing = np.einsum("ij,ij->i", normals, -pc) > 0
        pc = pc[facing & (pc[:, 2] > 0.05)]
        u = np.floor(pc[:, 0] / pc[:, 2] * intr.fx + intr.cx).astype(int)
        v = np.floor(pc[:, 1] / pc[:, 2] * intr.fy + intr.cy).astype(int)
        ok = (u >= 0) & (u < intr.width) & (v >= 0) & (v < intr.height)
        img[v[ok], u[ok]] = (0, 220, 60)
    Image.fromarray(img).save(path)


def believed_camera_pose(cap: dict, mount_usd: np.ndarray | None) -> np.ndarray:
    """The camera pose the robot would use: the capture's (true, from the mount as built), or the
    same arm pose with a hand-eye calibration's mount instead."""
    if mount_usd is None:
        return cap["T_world_cam"]
    gripper = str(cap.get("gripper", "robotiq_2f_140"))  # captures before other grippers name none
    tool = cap["T_world_cam"] @ np.linalg.inv(camera_model.mount(gripper) @ camera_model.USD_TO_CV)
    return camera_model.camera_pose(tool, mount_usd)


def evaluate(cap: dict, model: perception.ObjectModel, noise: str, overlay_dir: pathlib.Path | None = None,
             mount_usd: np.ndarray | None = None) -> dict:
    depth = camera_depth(cap, noise)
    T_wc = believed_camera_pose(cap, mount_usd)
    t0 = time.perf_counter()
    est = perception.locate(depth, cap["intr"], T_wc, model, cap.get("roi_centre", PICK_XY))
    if overlay_dir is not None:  # drawn with the true camera pose: where the estimate really is
        save_overlay(overlay_dir / f"{cap['object']}_{int(cap['cycle'])}_{noise}.png", cap, model, est.pose)
    row = {"object": cap["object"], "cycle": int(cap["cycle"]), "noise": noise, "wall_s": time.perf_counter() - t0,
           "n_points": est.n_points, "fitness": est.fitness, "violation": est.violation, "found": est.pose is not None,
           "detail": est.detail}
    if est.pose is not None:
        row.update(perception.pose_errors(model, est.pose, cap["obj_true"]))
    return row


def fmt(v, digits=1):
    return "–" if v is None or (isinstance(v, float) and np.isnan(v)) else f"{v:.{digits}f}"


def summarise(rows: list[dict]) -> list[str]:
    lines = ["| Object | Depth | Found | Centre mm (median / max) | Rotation deg (median) | ADD-S mm (median / max) "
             "| ADD mm (median) | Object points | Wall s |",
             "|---|---|---|---|---|---|---|---|---|"]
    keys = sorted({(r["object"], r["noise"]) for r in rows}, key=lambda k: (k[0], not k[1].startswith("perfect"), k[1]))
    for obj, noise in keys:
        rs = [r for r in rows if r["object"] == obj and r["noise"] == noise]
        ok = [r for r in rs if r["found"]]

        def med(k):
            return statistics.median([r[k] for r in ok]) if ok else None

        def mx(k):
            return max(r[k] for r in ok) if ok else None
        label = noise if noise.startswith("perfect") else f"stereo x{noise}"
        lines.append(f"| {obj} | {label} | {len(ok)}/{len(rs)} | {fmt(med('centre_mm'))} / {fmt(mx('centre_mm'))} "
                     f"| {fmt(med('rot_deg'))} | {fmt(med('adds_mm'))} / {fmt(mx('adds_mm'))} | {fmt(med('add_mm'))} "
                     f"| {fmt(med('n_points'), 0)} | {fmt(med('wall_s'), 2)} |")
    return lines


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--captures", default="results/perception/captures")
    parser.add_argument("--models", default="results/perception/models")
    parser.add_argument("--objects", help="comma-separated; default all captured")
    parser.add_argument("--noise", nargs="+", default=["1"])
    parser.add_argument("--write", action="store_true")
    parser.add_argument("--verbose", action="store_true", help="print every capture's result")
    parser.add_argument("--overlay", help="save each image with the estimated model drawn on it into this folder")
    parser.add_argument("--handeye", help="use this hand-eye calibration's camera mount (as the robot would) "
                                          "instead of the true one, e.g. results/perception/handeye.json")
    args = parser.parse_args()
    mount_usd = None
    if args.handeye:
        import json

        mount_usd = np.array(json.loads((ROOT / args.handeye).read_text(encoding="utf-8"))["mount_usd"])
    overlay_dir = None
    if args.overlay:
        overlay_dir = ROOT / args.overlay
        overlay_dir.mkdir(parents=True, exist_ok=True)

    paths = sorted((ROOT / args.captures).glob("*.npz"))
    if args.objects:
        wanted = set(args.objects.split(","))
        paths = [p for p in paths if p.stem.rsplit("_", 1)[0] in wanted]
    models: dict[str, perception.ObjectModel] = {}
    rows = []
    for path in paths:
        cap = load_capture(path)
        if cap["object"] not in models:
            m = np.load(ROOT / args.models / f"{cap['object']}.npz")
            models[cap["object"]] = perception.ObjectModel(m["points"], m["normals"])
        for noise in args.noise:
            runs = [evaluate(cap, models[cap["object"]], noise, overlay_dir)]
            if mount_usd is not None:
                runs.append(dict(evaluate(cap, models[cap["object"]], noise, None, mount_usd), noise=f"{noise}, calibrated mount"))
            for row in runs:
                rows.append(row)
                if not args.verbose:
                    continue
                print(f"{path.stem} {row['noise']}: " + (f"centre {row['centre_mm']:.1f} mm, rot {row['rot_deg']:.1f} deg, "
                                                  f"ADD-S {row['adds_mm']:.1f} mm, ADD {row['add_mm']:.1f} mm, "
                                                  f"{row['n_points']} pts, fit {row['fitness']:.2f}, viol {row['violation']:.3f}, "
                                                  f"{row['wall_s']:.2f}s" if row["found"] else f"not found: {row['detail']}"),
                      flush=True)
    lines = summarise(rows)
    print("\n".join(lines))
    if args.write:
        text = "\n".join(["# Perception results", "",
                          "The object's pose from one wrist-camera depth image (pick_place/perception.py), against the true pose,",
                          "on the captures of capture_views.py: the start poses of the pick-and-place cycles, seen from the look pose.",
                          "*Centre*: distance between the model's centroid as estimated and as true. *ADD-S*: mean distance from each",
                          "estimated model point to the nearest true one (near zero for a pose a symmetric object cannot tell from",
                          "the true one); *ADD*: mean distance between corresponding model points. *Depth*: the renderer's perfect",
                          "depth, or the simulated stereo depth at 1 or 2 times camera_model's noise. Unless marked *calibrated",
                          "mount*, the camera's pose is the true one; with it, the robot uses the hand-eye calibration's mount, as",
                          "it does in the cell. *Wall s*: time of perception.locate() (offline, on this PC).", "", *lines, ""])
        (ROOT / "results/perception_summary.md").write_text(text, encoding="utf-8")


if __name__ == "__main__":
    main()
