from __future__ import annotations

import csv
import json
from pathlib import Path
import subprocess
import sys
import unittest

from calibration.phase2b2_common import (
    FIELD_LIMITS_DEG,
    assessments,
    load_snapshot,
    neutral_arm_pose_deg,
    ranked_candidates,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CALIBRATION = PROJECT_ROOT / "calibration"
ACTIVE = CALIBRATION / "active_tests"
SCRIPT = ACTIVE / "phase2b2_active_test.py"


class Phase2B2OfflineTests(unittest.TestCase):
    def test_snapshot_and_directional_margins_cover_all_arm_joints(self) -> None:
        snapshot = load_snapshot()
        rows = assessments(snapshot)
        self.assertEqual(len(rows), 14)
        self.assertEqual({row.name for row in rows}, set(FIELD_LIMITS_DEG))
        self.assertTrue(all(row.lower_distance_deg >= 0 for row in rows))
        self.assertTrue(all(row.upper_distance_deg >= 0 for row in rows))

    def test_adaptive_screen_skips_both_shoulder_roll_joints(self) -> None:
        rows = {
            row["name"]: row
            for row in ranked_candidates(
                load_snapshot(),
                requested_deg=2.0,
                reserve_deg=5.0,
                minimum_useful_deg=1.0,
            )
        }
        self.assertIsNone(rows["left_shoulder_roll_joint"]["selected_amplitude_deg"])
        self.assertIsNone(rows["right_shoulder_roll_joint"]["selected_amplitude_deg"])
        selected = [row for row in rows.values() if row["selected_amplitude_deg"] is not None]
        self.assertEqual(len(selected), 12)
        self.assertTrue(all(row["selected_amplitude_deg"] == 2.0 for row in selected))

    def test_neutral_candidate_has_at_least_ten_degrees_j2_margin(self) -> None:
        pose = neutral_arm_pose_deg(load_snapshot())
        for name in ("left_shoulder_roll_joint", "right_shoulder_roll_joint"):
            lower, upper = FIELD_LIMITS_DEG[name]
            self.assertGreaterEqual(pose[name] - lower, 10.0)
            self.assertGreaterEqual(upper - pose[name], 10.0)

    def test_default_dry_run_cannot_import_ros_or_send(self) -> None:
        result = subprocess.run(
            [sys.executable, str(SCRIPT), "--dry-run"],
            cwd=PROJECT_ROOT,
            capture_output=True,
            text=True,
            timeout=30,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("PHASE2B2_MODE=DRY_RUN", result.stdout)
        self.assertIn("ROS_IMPORTED=0", result.stdout)
        self.assertIn("PUBLISHER_CREATED=0", result.stdout)
        self.assertIn("COMMAND_SENT=0", result.stdout)
        self.assertIn("GO_NO_GO=NO-GO", result.stdout)

    def test_supplied_motion_gate_fails_before_ros_import(self) -> None:
        result = subprocess.run(
            [sys.executable, str(SCRIPT), "--enable-motion"],
            cwd=PROJECT_ROOT,
            capture_output=True,
            text=True,
            timeout=30,
            check=False,
        )
        self.assertEqual(result.returncode, 3)
        combined = result.stdout + result.stderr
        self.assertIn("GO_NO_GO=NO-GO", combined)
        self.assertIn("ROS_IMPORTED=0", combined)
        self.assertIn("PUBLISHER_CREATED=0", combined)
        self.assertIn("COMMAND_SENT=0", combined)

    def test_sim_rehearsal_outputs_are_separate_and_complete(self) -> None:
        summary = json.loads(
            (CALIBRATION / "evidence" / "phase2b2_sim_rehearsal_summary.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(len(summary["fixed_base_results"]), 12)
        self.assertEqual(
            set(summary["skipped_joints"]),
            {"left_shoulder_roll_joint", "right_shoulder_roll_joint"},
        )
        required = {
            "timestamp",
            "joint_name",
            "command_position",
            "measured_position",
            "measured_velocity",
            "measured_torque",
            "imu_quaternion",
            "imu_gyro",
            "imu_accel",
            "phase",
        }
        for row in summary["fixed_base_results"]:
            self.assertEqual(row["maximum_self_collision_count"], 0)
            self.assertEqual(row["model_limit_violation_steps"], 0)
            self.assertEqual(row["tracking_status"], "TRACKING_NOT_SETTLED")
            csv_path = PROJECT_ROOT / row["csv"]
            plot_path = PROJECT_ROOT / row["plot"]
            self.assertTrue(csv_path.is_file(), csv_path)
            self.assertTrue(plot_path.is_file(), plot_path)
            with csv_path.open("r", encoding="utf-8", newline="") as stream:
                header = set(next(csv.reader(stream)))
            self.assertTrue(required.issubset(header))

        for name in ("j2_left.csv", "j2_right.csv", "j7_left.csv", "j7_right.csv"):
            self.assertFalse((ACTIVE / name).exists(), f"synthetic real log must not exist: {name}")


if __name__ == "__main__":
    unittest.main()
