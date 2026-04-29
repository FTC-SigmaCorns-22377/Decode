#!/usr/bin/env python3
"""
Generate FTC DECODE autonomous JSON files and invoke the trajopt solver.

Usage examples:
  python generate_autos.py --type 12far   --side red
  python generate_autos.py --type 15close --side blue

The script copies a template JSON (found in TeamCode/trajopt/),
updates the `mirrored` flag and optionally mirrors the waypoint coordinates,
then runs the Gradle `runTrajopt` task which starts the trajectory solver.
The generated `*_traj.json` file is moved into the assets directory used by the op‑modes.
"""

import argparse
import json
import pathlib
import shutil
import subprocess
import math

# Repository layout constants
REPO_ROOT = pathlib.Path(__file__).resolve().parent
TRAJOPT_DIR = REPO_ROOT / "TeamCode" / "trajopt"
ASSETS_DIR = REPO_ROOT / "TeamCode" / "src" / "main" / "assets" / "autos"

# Mapping from user‑visible auto type to the template JSON filename
TEMPLATE_MAP = {
    "12far": "12ballfar.json",
    "15close": "red-15-close-1.json",
    "15far": "15closeredfarbetter.json",
}

# FTC field width in inches – used when mirroring X coordinates
FIELD_WIDTH_IN = 144.0

def mirror_waypoint(pt: dict) -> dict:
    """Return a mirrored waypoint (X flipped, heading reversed).
    Expected keys: x, y, heading (radians).  Missing keys are left untouched.
    """
    mirrored = dict(pt)
    if "x" in pt:
        mirrored["x"] = FIELD_WIDTH_IN - pt["x"]
    if "heading" in pt:
        mirrored["heading"] = (math.pi - pt["heading"]) % (2 * math.pi)
    return mirrored

def load_template(auto_type: str) -> dict:
    tmpl_name = TEMPLATE_MAP.get(auto_type)
    if tmpl_name is None:
        raise ValueError(f"Unsupported auto type '{auto_type}'. Available: {list(TEMPLATE_MAP)}")
    tmpl_path = TRAJOPT_DIR / tmpl_name
    if not tmpl_path.is_file():
        raise FileNotFoundError(f"Template JSON not found: {tmpl_path}")
    with tmpl_path.open() as f:
        return json.load(f)

def write_output_json(data: dict, out_path: pathlib.Path):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w") as f:
        json.dump(data, f, indent=2)
    print(f"✅ Written JSON to {out_path}")

def run_trajopt_solver(json_path: pathlib.Path):
    cmd = ["./gradlew", ":TeamCode:runTrajopt", "--args", str(json_path)]
    print(f"▶️ Running trajopt solver: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=REPO_ROOT, check=True)
    print("✅ Trajopt solver completed")

def main():
    parser = argparse.ArgumentParser(description="Generate FTC DECODE autos using the existing trajopt solver")
    parser.add_argument("--type", required=True, choices=TEMPLATE_MAP.keys(), help="Auto type to generate")
    parser.add_argument("--side", required=True, choices=["red", "blue"], help="Alliance side (blue side is mirrored)")
    parser.add_argument("--no-mirror-waypoints", action="store_true", help="Do not automatically mirror waypoint coordinates; only set the mirrored flag")
    args = parser.parse_args()

    data = load_template(args.type)
    mirrored = args.side == "blue"
    data["mirrored"] = mirrored
    if not args.no_mirror_waypoints and "waypoints" in data:
        original = data["waypoints"]
        data["waypoints"] = [mirror_waypoint(pt) if mirrored else pt for pt in original]
    out_name = f"{args.type}_{args.side}.json"
    out_path = TRAJOPT_DIR / out_name
    write_output_json(data, out_path)
    run_trajopt_solver(out_path)
    generated = TRAJOPT_DIR / f"{out_name}_traj.json"
    if generated.is_file():
        ASSETS_DIR.mkdir(parents=True, exist_ok=True)
        shutil.move(str(generated), ASSETS_DIR / generated.name)
        print(f"✅ Moved generated trajectory to {ASSETS_DIR / generated.name}")
    else:
        print("⚠️ No generated trajectory file found – the solver may still be running or failed.")

if __name__ == "__main__":
    main()
