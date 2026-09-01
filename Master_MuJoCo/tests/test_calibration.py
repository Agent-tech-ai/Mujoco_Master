from __future__ import annotations

import csv
import hashlib
from pathlib import Path
import unittest

from calibration.analysis import compare_joint
from calibration.inspect_model import DEFAULT_MODELS, inspect
from calibration.log_io import (
    canonicalize_rows,
    joint_series,
    load_log,
    load_mapping,
    relative_time,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CALIBRATION = PROJECT_ROOT / "calibration"
ORIGINAL_XML = PROJECT_ROOT / "assets" / "Master" / "ff_master_ultra.xml"
ORIGINAL_SHA256 = "89619295fcc372c57473224130865b2fe4f22e0741f72925fac243805f4353db"


class CalibrationTests(unittest.TestCase):
    def test_original_mjcf_is_preserved(self) -> None:
        digest = hashlib.sha256(ORIGINAL_XML.read_bytes()).hexdigest()
        self.assertEqual(digest, ORIGINAL_SHA256)

    def test_mapping_keeps_unverified_hardware_parameters_unknown(self) -> None:
        with (CALIBRATION / "joint_mapping.csv").open(
            "r", encoding="utf-8", newline=""
        ) as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 31)
        self.assertTrue(all(row["hardware_joint_id"] == "UNKNOWN" for row in rows))
        self.assertTrue(all(row["hardware_zero"] == "UNKNOWN" for row in rows))
        self.assertTrue(all(row["sign"] == "UNKNOWN" for row in rows))
        self.assertTrue(all(row["encoder_offset"] == "UNKNOWN" for row in rows))
        head_pitch = next(row for row in rows if row["hardware_joint_name"] == "head_pitch")
        self.assertEqual(head_pitch["mujoco_joint_name"], "NOT_PRESENT_FIXED_0_DEG")

    def test_inspector_loads_all_four_models(self) -> None:
        reports = [inspect(path) for path in DEFAULT_MODELS]
        self.assertEqual([report["counts"]["joints"] for report in reports], [32, 31, 31, 31])
        self.assertEqual([report["counts"]["actuators"] for report in reports], [31, 30, 30, 30])
        for report in reports:
            for joint in report["joints"]:
                self.assertIn("body_link", joint)
                self.assertIn("damping", joint)
                self.assertIn("armature", joint)
                self.assertIn("frictionloss", joint)
                self.assertIn("joint_actuator_force_range", joint)

    def test_synthetic_logs_align_and_raise_expected_candidates(self) -> None:
        aliases, limits = load_mapping(CALIBRATION / "joint_mapping.csv")
        real = relative_time(
            canonicalize_rows(load_log(CALIBRATION / "logs" / "real" / "test.csv"), aliases)
        )
        sim = relative_time(
            canonicalize_rows(load_log(CALIBRATION / "logs" / "sim" / "test.csv"), aliases)
        )
        head, _, _ = compare_joint(
            "head_yaw_joint",
            joint_series(real, "head_yaw_joint"),
            joint_series(sim, "head_yaw_joint"),
            limits["head_yaw_joint"],
        )
        knee, _, _ = compare_joint(
            "left_knee_joint",
            joint_series(real, "left_knee_joint"),
            joint_series(sim, "left_knee_joint"),
            limits["left_knee_joint"],
        )
        self.assertTrue(any("SIGN_MISMATCH" in flag for flag in head.flags))
        self.assertTrue(any("ZERO_OFFSET" in flag for flag in knee.flags))
        self.assertTrue(any("POSITION_SCALE" in flag for flag in knee.flags))


if __name__ == "__main__":
    unittest.main()

