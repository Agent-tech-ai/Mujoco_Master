from __future__ import annotations

import json
import math
from pathlib import Path
import unittest

import mujoco
import numpy as np

from master_sim.controller import SimulationStabilityController
from master_sim.model import load_model


PROJECT_ROOT = Path(__file__).resolve().parents[1]
STABILITY = PROJECT_ROOT / "calibration" / "standing_stability"


class StandingStabilityTests(unittest.TestCase):
    def test_simulation_cleanup_stands_for_ten_seconds(self) -> None:
        model = load_model(free_base=True)
        data = mujoco.MjData(model)
        controller = SimulationStabilityController(model)
        controller.initialize_data(data)
        pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        maximum_tilt_deg = 0.0
        maximum_saturation = 0.0
        while data.time < 10.0:
            controller.apply(data)
            mujoco.mj_step(model, data)
            rotation = data.xmat[pelvis_id].reshape(3, 3)
            roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
            pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
            maximum_tilt_deg = max(
                maximum_tilt_deg, math.degrees(math.hypot(roll, pitch))
            )
            limit = np.maximum(
                np.abs(model.actuator_ctrlrange[:, 0]),
                np.abs(model.actuator_ctrlrange[:, 1]),
            )
            maximum_saturation = max(
                maximum_saturation, float(np.max(np.abs(data.ctrl) / limit))
            )
        self.assertGreater(float(data.xpos[pelvis_id, 2]), 0.60)
        self.assertLess(maximum_tilt_deg, 3.0)
        self.assertLess(maximum_saturation, 0.5)

    def test_cleanup_evidence_passes_and_preserves_model_boundary(self) -> None:
        summary = json.loads((STABILITY / "summary.json").read_text(encoding="utf-8"))
        cleanup = summary["runs"]["free_cleanup"]
        self.assertEqual(summary["safety"], "LOCAL_SIMULATION_ONLY_NO_ROBOT_ACCESS")
        self.assertEqual(summary["model_files_modified"], [])
        self.assertIsNone(cleanup["fall_time_s"])
        self.assertEqual(cleanup["saturation_ratio"], 0.0)
        self.assertGreater(cleanup["both_feet_contact_fraction"], 0.99)
        self.assertLess(cleanup["left_foot_slip_m"], 0.001)
        self.assertLess(cleanup["right_foot_slip_m"], 0.001)
        self.assertTrue(summary["com"]["inside"])

    def test_all_rehearsals_settle_without_collision_or_limit_violation(self) -> None:
        summary = json.loads(
            (STABILITY / "rehearsal_summary.json").read_text(encoding="utf-8")
        )
        self.assertEqual(len(summary["after"]), 12)
        self.assertEqual(summary["settled_after"], 12)
        for result in summary["after"]:
            self.assertEqual(result["tracking_status"], "SETTLED")
            self.assertTrue(result["target_generator_complete"])
            self.assertEqual(result["actuator_saturation_ratio"], 0.0)
            self.assertEqual(result["model_limit_violation_steps"], 0)
            self.assertEqual(result["maximum_self_collision_count"], 0)

    def test_requested_reports_and_plots_exist(self) -> None:
        reports = (
            "baseline_report.md",
            "initial_pose_report.md",
            "com_support_polygon_report.md",
            "mass_inertia_audit.md",
            "foot_collision_report.md",
            "contact_report.md",
            "actuator_sanity_report.md",
            "solver_sensitivity_report.md",
            "fixed_vs_free_report.md",
            "rehearsal_before_after_report.md",
            "standing_stability_experiments.csv",
        )
        for name in reports:
            self.assertTrue((STABILITY / name).is_file(), name)
        self.assertGreaterEqual(len(list((STABILITY / "plots").glob("*.png"))), 5)


if __name__ == "__main__":
    unittest.main()
