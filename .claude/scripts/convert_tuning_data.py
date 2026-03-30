#!/usr/bin/env python3
"""
Convert AdaptiveTuner shot_tuning_data.json (SpeedPoint format) to
FlywheelMap optimized_shot_tuning_data.json format.

SpeedPoint format:  {distance, speed (rad/s), hoodAngle (deg)}
FlywheelMap format: {phi (rad), vExit (m/s), omega (rad/s)}

Back-computes v_exit from projectile equations:
  v = speed * flywheelRadius * launchEfficiency  (linear exit velocity)
  This is the ball's initial speed. The hood angle determines the split
  between horizontal and vertical components.

Usage:
  python3 convert_tuning_data.py [input.json] [output.json]

  Defaults:
    input:  /sdcard/FIRST/shot_tuning_data.json
    output: /sdcard/FIRST/optimized_shot_tuning_data.json
"""

import json
import math
import sys

# Match HoodConfig constants
FLYWHEEL_RADIUS = 0.05      # m
LAUNCH_EFFICIENCY = 0.3
GRAVITY = 9.81              # m/s^2
GOAL_HEIGHT = 0.75          # m
LAUNCH_HEIGHT = 0.22        # m


def speed_point_to_flywheel_map(point: dict) -> dict:
    """Convert a SpeedPoint to a FlywheelMapPoint.

    Computes exit velocity from flywheel omega using the linear relationship:
    v_exit = omega * flywheelRadius * launchEfficiency
    """
    omega = point["speed"]  # flywheel angular velocity (rad/s)
    hood_angle_deg = point.get("hoodAngle", 45.0)
    phi = math.radians(hood_angle_deg)

    # Exit velocity from flywheel speed
    v_exit = omega * FLYWHEEL_RADIUS * LAUNCH_EFFICIENCY

    return {
        "phi": round(phi, 6),
        "vExit": round(v_exit, 6),
        "omega": round(omega, 4)
    }


def convert(input_path: str, output_path: str):
    with open(input_path, "r") as f:
        speed_points = json.load(f)

    flywheel_points = [speed_point_to_flywheel_map(sp) for sp in speed_points]

    # Sort by exit velocity
    flywheel_points.sort(key=lambda p: p["vExit"])

    with open(output_path, "w") as f:
        json.dump(flywheel_points, f, indent=2)

    print(f"Converted {len(flywheel_points)} points")
    print(f"  Input:  {input_path}")
    print(f"  Output: {output_path}")
    for p in flywheel_points:
        phi_deg = round(math.degrees(p["phi"]), 1)
        print(f"  phi={phi_deg}deg  v_exit={p['vExit']:.3f}m/s  omega={p['omega']:.1f}rad/s")


if __name__ == "__main__":
    input_file = sys.argv[1] if len(sys.argv) > 1 else "/sdcard/FIRST/shot_tuning_data.json"
    output_file = sys.argv[2] if len(sys.argv) > 2 else "/sdcard/FIRST/optimized_shot_tuning_data.json"
    convert(input_file, output_file)
