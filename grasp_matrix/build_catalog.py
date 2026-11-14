"""Build the object catalog from SimReady package metadata.

Reads only each package's entry layer (its SimReady_Metadata block holds extents, mass,
category, Wikidata Q-code and licence), so no geometry has to be downloaded. Writes
data/object_catalog.csv with one row per object and, for each gripper, whether the
object's smallest extent fits inside the open fingers. That flag is only a first filter:
an object can still be grasped by a narrower feature such as a handle or rim.

Run with the project venv (needs usd-core):
    .venv/Scripts/python.exe grasp_matrix/build_catalog.py assets/allsides_meta --revision <commit>
"""

import argparse
import csv
import json
import pathlib

from pxr import Sdf

ROOT = pathlib.Path(__file__).resolve().parents[1]
# Measured open gap of each gripper (tools/calibrate_grippers.py), not the datasheet value.
GRIPPERS = json.loads((ROOT / "data/gripper_calibration.json").read_text())
OUT = ROOT / "data/object_catalog.csv"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("meta_dir", type=pathlib.Path, help="folder holding <package>/simready_usd/<package>.usd")
    parser.add_argument("--dataset", default="ALLSIDES/simready-scanned-objects")
    parser.add_argument("--revision", required=True, help="dataset commit the entry files came from")
    args = parser.parse_args()

    rows = []
    for entry in sorted(args.meta_dir.glob("*/simready_usd/*.usd")):
        meta = dict(Sdf.Layer.FindOrOpen(str(entry)).customLayerData).get("SimReady_Metadata", {})
        if not meta:
            print(f"skipping {entry}: no SimReady_Metadata")
            continue
        dims = sorted(float(v) for v in meta["asset_extents"])
        row = {
            "asset_name": meta["asset_name"],
            "category": meta.get("category", ""),
            "description": meta.get("description", ""),
            "min_dim_m": round(dims[0], 4),
            "mid_dim_m": round(dims[1], 4),
            "max_dim_m": round(dims[2], 4),
            "mass_kg": round(float(meta.get("mass", 0.0)), 4),
            "wikidata_qcode": meta.get("qcode", ""),
            "license": meta.get("asset_license", ""),
            "package_version": meta.get("version", ""),
            "validation_profile": meta.get("validation", {}).get("profile", ""),
            "source_dataset": args.dataset,
            "source_revision": args.revision,
        }
        for name, gripper in GRIPPERS.items():
            row[f"fits_{name}"] = dims[0] <= gripper["max_aperture_m"]
        rows.append(row)

    OUT.parent.mkdir(parents=True, exist_ok=True)
    with OUT.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {len(rows)} objects to {OUT}")
    for name in GRIPPERS:
        print(f"  smallest extent fits {name}: {sum(r[f'fits_{name}'] for r in rows)}")


if __name__ == "__main__":
    main()
