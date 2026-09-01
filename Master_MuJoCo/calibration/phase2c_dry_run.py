#!/usr/bin/env python3
"""Phase 2C offline command-plan printer.

This module is intentionally incapable of robot motion: it imports only Python
standard-library modules, reads a local snapshot, and prints a plan. It creates
no ROS node, publisher, service/action client, SDK session, or SSH connection.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


CALIBRATION_DIR = Path(__file__).resolve().parent
DEFAULT_SNAPSHOT = CALIBRATION_DIR / "evidence" / "phase2b2_latest_arm_snapshot.json"

# Operator-supplied FIELD_TEST_EVIDENCE (degrees), 2026-08-11. These are
# hardware control-coordinate limits, not evidence of a MuJoCo sign or axis.
FIELD_LIMITS_DEG: dict[str, tuple[float, float]] = {
    "left_shoulder_pitch_joint": (-176.471, 116.883),
    "left_shoulder_roll_joint": (-3.495, 171.486),
    "left_shoulder_yaw_joint": (-146.448, 146.448),
    "left_elbow_joint": (-134.965, 0.0),
    "left_wrist_yaw_joint": (-146.448, 146.448),
    "left_wrist_pitch_joint": (-31.971, 31.971),
    "left_wrist_roll_joint": (-90.012, 41.482),
    "right_shoulder_pitch_joint": (-176.471, 116.883),
    "right_shoulder_roll_joint": (-171.486, 3.495),
    "right_shoulder_yaw_joint": (-146.448, 146.448),
    "right_elbow_joint": (-134.965, 0.0),
    "right_wrist_yaw_joint": (-146.448, 146.448),
    "right_wrist_pitch_joint": (-31.971, 31.971),
    "right_wrist_roll_joint": (-41.482, 90.012),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Print an offline Phase 2C single-joint test plan; never move a robot."
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="required safety latch; the program has no execution mode",
    )
    parser.add_argument("--joint", required=True, choices=sorted(FIELD_LIMITS_DEG))
    parser.add_argument("--delta-deg", required=True, type=float)
    parser.add_argument("--duration-s", type=float, default=3.0)
    parser.add_argument("--reserve-deg", type=float, default=5.0)
    parser.add_argument("--minimum-useful-deg", type=float, default=1.0)
    parser.add_argument("--snapshot", type=Path, default=DEFAULT_SNAPSHOT)
    parser.add_argument(
        "--interface",
        default="NO_APPROVED_MC_SINGLE_JOINT_INTERFACE",
        help="descriptive evidence label only; it is never imported or called",
    )
    return parser.parse_args()


def fail(message: str) -> int:
    print("DRY_RUN_ONLY=true")
    print("MOTION_CAPABILITY_PRESENT=false")
    print("STATUS=REFUSED")
    print(f"REASON={message}")
    return 2


def main() -> int:
    args = parse_args()
    if not args.dry_run:
        return fail("--dry-run is mandatory; no execution mode exists")
    if not math.isfinite(args.delta_deg) or args.delta_deg == 0.0:
        return fail("--delta-deg must be finite and non-zero")
    if abs(args.delta_deg) < args.minimum_useful_deg:
        return fail("requested amplitude is below the minimum useful amplitude")
    if args.duration_s <= 0.0 or args.reserve_deg < 0.0:
        return fail("duration must be positive and reserve must be non-negative")

    snapshot_path = args.snapshot.resolve()
    with snapshot_path.open("r", encoding="utf-8") as stream:
        snapshot = json.load(stream)
    by_name = {entry["name"]: entry for entry in snapshot["joints"]}
    if args.joint not in by_name:
        return fail(f"joint is absent from snapshot: {args.joint}")

    current_deg = math.degrees(float(by_name[args.joint]["position"]))
    target_deg = current_deg + args.delta_deg
    lower_deg, upper_deg = FIELD_LIMITS_DEG[args.joint]
    target_lower_margin = target_deg - lower_deg
    target_upper_margin = upper_deg - target_deg
    within_limits = lower_deg <= target_deg <= upper_deg
    reserve_pass = min(target_lower_margin, target_upper_margin) >= args.reserve_deg
    status = "PLAN_GEOMETRIC_CHECK_PASS" if within_limits and reserve_pass else "SKIP"

    print("DRY_RUN_ONLY=true")
    print("MOTION_CAPABILITY_PRESENT=false")
    print("ROS_NODE_CREATED=false")
    print("PUBLISHER_OR_CLIENT_CREATED=false")
    print(f"selected_joint={args.joint}")
    print("live_current_position_source=/aima/hal/joint/arm/state")
    print(f"calculation_source={snapshot_path}")
    print(f"calculation_source_timestamp={snapshot.get('capture_host_time', 'UNKNOWN')}")
    print("calculation_source_fresh_for_motion=false")
    print(f"current_position_deg={current_deg:.9f}")
    print(f"target_delta_deg={args.delta_deg:+.9f}")
    print(f"target_position_deg={target_deg:.9f}")
    print(f"command_interface={args.interface}")
    print(f"expected_duration_s={args.duration_s:.6f}")
    print(
        "safety_limit_deg="
        f"[{lower_deg:.6f},{upper_deg:.6f}] FIELD_TEST_EVIDENCE"
    )
    print(f"required_limit_reserve_deg={args.reserve_deg:.6f}")
    print(f"target_lower_margin_deg={target_lower_margin:.9f}")
    print(f"target_upper_margin_deg={target_upper_margin:.9f}")
    print(f"return_target_deg={current_deg:.9f}")
    print("publisher_count_check=NOT_PERFORMED_OFFLINE")
    print("control_input_source_check=NOT_PERFORMED_OFFLINE")
    print("operator_confirmation=REQUIRED_BEFORE_ANY_FUTURE_MOTION")
    print(f"STATUS={status}")
    if status == "PLAN_GEOMETRIC_CHECK_PASS":
        print("NOTE=geometric offline check only; this is not a GO authorization")
        return 0
    print("NOTE=target fails hardware-limit/reserve calculation and must be skipped")
    return 3


if __name__ == "__main__":
    raise SystemExit(main())
