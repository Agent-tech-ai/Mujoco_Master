from __future__ import annotations

import unittest

import mujoco
import numpy as np

from master_sim.controller import JointPositionController, POSES
from master_sim.model import EXPECTED_LIMITS_DEG, load_model, validate_model


class MasterModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.model = load_model()

    def test_documented_model_structure_and_limits(self) -> None:
        self.assertEqual(validate_model(self.model), [])
        self.assertEqual(self.model.nu, len(EXPECTED_LIMITS_DEG))
        self.assertEqual(self.model.nu, 30)

    def test_head_pitch_is_fixed(self) -> None:
        joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "head_pitch_joint"
        )
        body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "head_pitch_link"
        )
        self.assertEqual(joint_id, -1)
        self.assertGreaterEqual(body_id, 0)

    def test_all_presets_are_inside_joint_limits(self) -> None:
        controller = JointPositionController(self.model)
        for pose_name in POSES:
            controller.set_pose(pose_name)
            for joint in controller.joints:
                value = controller.target[joint.qpos_adr]
                self.assertGreaterEqual(value, joint.lower)
                self.assertLessEqual(value, joint.upper)

    def test_fixed_base_controller_is_numerically_stable(self) -> None:
        data = mujoco.MjData(self.model)
        controller = JointPositionController(self.model)
        controller.set_pose("wave")
        controller.initialize_data(data)
        while data.time < 0.5:
            controller.apply(data)
            mujoco.mj_step(self.model, data)
        self.assertTrue(np.all(np.isfinite(data.qpos)))
        self.assertTrue(np.all(np.isfinite(data.qvel)))
        self.assertLess(float(np.max(np.abs(data.qvel))), 20.0)

    def test_free_base_variant_loads(self) -> None:
        model = load_model(free_base=True)
        self.assertEqual(validate_model(model), [])
        self.assertEqual(model.neq, 0)


if __name__ == "__main__":
    unittest.main()

