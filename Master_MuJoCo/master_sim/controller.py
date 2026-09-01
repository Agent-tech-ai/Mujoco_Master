"""Joint-space position controller and safe demonstration poses."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping

import mujoco
import numpy as np

from .model import actuated_joint_names


POSES: dict[str, dict[str, float]] = {
    "home": {
        "left_shoulder_roll_joint": 0.10,
        "right_shoulder_roll_joint": -0.10,
        "left_elbow_joint": -0.35,
        "right_elbow_joint": -0.35,
    },
    "crouch": {
        "left_hip_pitch_joint": -0.42,
        "right_hip_pitch_joint": -0.42,
        "left_knee_joint": 0.82,
        "right_knee_joint": 0.82,
        "left_ankle_pitch_joint": -0.38,
        "right_ankle_pitch_joint": -0.38,
        "left_shoulder_roll_joint": 0.10,
        "right_shoulder_roll_joint": -0.10,
        "left_elbow_joint": -0.35,
        "right_elbow_joint": -0.35,
    },
    "tpose": {
        "left_shoulder_roll_joint": 1.45,
        "right_shoulder_roll_joint": -1.45,
        "left_elbow_joint": -0.08,
        "right_elbow_joint": -0.08,
    },
    "wave": {
        "left_shoulder_roll_joint": 0.10,
        "left_elbow_joint": -0.35,
        "right_shoulder_pitch_joint": -0.25,
        "right_shoulder_roll_joint": -1.20,
        "right_elbow_joint": -1.55,
        "right_wrist_pitch_joint": 0.20,
    },
}


@dataclass(frozen=True)
class DrivenJoint:
    actuator_id: int
    joint_id: int
    qpos_adr: int
    dof_adr: int
    name: str
    lower: float
    upper: float
    kp: float
    kd: float


def _gains(name: str) -> tuple[float, float]:
    if any(token in name for token in ("hip", "knee")):
        kp = 115.0
    elif "ankle" in name:
        kp = 55.0
    elif "waist" in name:
        kp = 85.0
    elif "shoulder" in name or "elbow" in name:
        kp = 38.0
    elif "wrist" in name:
        kp = 12.0
    else:
        kp = 16.0
    return kp, 1.6 * math.sqrt(kp)


class JointPositionController:
    """PD plus bias-force compensation for all direct-drive joints."""

    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self.joints: list[DrivenJoint] = []
        names = actuated_joint_names(model)
        for actuator_id, name in enumerate(names):
            joint_id = int(model.actuator_trnid[actuator_id, 0])
            kp, kd = _gains(name)
            lower, upper = model.jnt_range[joint_id]
            self.joints.append(
                DrivenJoint(
                    actuator_id=actuator_id,
                    joint_id=joint_id,
                    qpos_adr=int(model.jnt_qposadr[joint_id]),
                    dof_adr=int(model.jnt_dofadr[joint_id]),
                    name=name,
                    lower=float(lower),
                    upper=float(upper),
                    kp=kp,
                    kd=kd,
                )
            )
        self.target = np.zeros(model.nq, dtype=float)
        self.pose_name = "home"
        self.set_pose("home")

    def set_pose(self, name: str) -> None:
        if name not in POSES:
            raise KeyError(f"Unknown pose {name!r}; choose from {sorted(POSES)}")
        self.pose_name = name
        for joint in self.joints:
            self.target[joint.qpos_adr] = 0.0
        self.set_targets(POSES[name])

    def set_targets(self, targets: Mapping[str, float]) -> None:
        by_name = {joint.name: joint for joint in self.joints}
        unknown = sorted(set(targets) - set(by_name))
        if unknown:
            raise KeyError(f"Unknown or non-actuated joints: {unknown}")
        for name, value in targets.items():
            joint = by_name[name]
            self.target[joint.qpos_adr] = float(
                np.clip(value, joint.lower, joint.upper)
            )

    def set_targets_degrees(self, targets: Mapping[str, float]) -> None:
        self.set_targets({name: math.radians(value) for name, value in targets.items()})

    def initialize_data(self, data: mujoco.MjData) -> None:
        """Start the actuated joints exactly at the selected target pose."""

        for joint in self.joints:
            data.qpos[joint.qpos_adr] = self.target[joint.qpos_adr]
            data.qvel[joint.dof_adr] = 0.0
        mujoco.mj_forward(self.model, data)

    def target_for_time(self, sim_time: float) -> np.ndarray:
        target = self.target.copy()
        if self.pose_name == "wave":
            by_name = {joint.name: joint for joint in self.joints}
            wrist = by_name["right_wrist_yaw_joint"]
            target[wrist.qpos_adr] = 0.75 * math.sin(3.5 * sim_time)
        return target

    def apply(self, data: mujoco.MjData) -> None:
        target = self.target_for_time(data.time)
        for joint in self.joints:
            error = target[joint.qpos_adr] - data.qpos[joint.qpos_adr]
            velocity = data.qvel[joint.dof_adr]
            bias = data.qfrc_bias[joint.dof_adr]
            torque = joint.kp * error - joint.kd * velocity + bias
            if bool(self.model.actuator_ctrllimited[joint.actuator_id]):
                lower, upper = self.model.actuator_ctrlrange[joint.actuator_id]
                torque = float(np.clip(torque, lower, upper))
            data.ctrl[joint.actuator_id] = torque


@dataclass(frozen=True)
class SimulationStabilityConfig:
    """Simulation-only feedback used to make free-base infrastructure testable.

    These gains are not X2 hardware gains. They act on simulated pelvis attitude
    and on the friction term already present in the MJCF.
    """

    pitch_kp: float = 200.0
    pitch_kd: float = 30.0
    roll_kp: float = 100.0
    roll_kd: float = 20.0
    friction_compensation_scale: float = 1.5
    friction_error_width: float = 0.005


class SimulationStabilityController(JointPositionController):
    """PD controller plus explicitly simulation-only stance/tracking cleanup.

    The base-attitude loop is enabled only for the unwelded free-base scene.
    Smooth compensation of the MJCF's own ``frictionloss`` is used in both
    fixed and free scenes to remove the pure-PD static deadband during small
    rehearsal motions. No hardware parameter is represented or inferred.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        config: SimulationStabilityConfig | None = None,
    ) -> None:
        super().__init__(model)
        self.config = config or SimulationStabilityConfig()
        self._by_name = {joint.name: joint for joint in self.joints}
        self._pelvis_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "pelvis"
        )
        if self._pelvis_id < 0:
            raise ValueError("SimulationStabilityController requires body 'pelvis'")

    @staticmethod
    def _roll_pitch(rotation: np.ndarray) -> tuple[float, float]:
        matrix = rotation.reshape(3, 3)
        roll = math.atan2(float(matrix[2, 1]), float(matrix[2, 2]))
        pitch = math.asin(float(np.clip(-matrix[2, 0], -1.0, 1.0)))
        return roll, pitch

    def apply(self, data: mujoco.MjData) -> None:
        target = self.target_for_time(data.time)
        additions = {joint.name: 0.0 for joint in self.joints}

        # scene_x2_fixed.xml retains the free joint but welds the pelvis. The
        # attitude loop belongs only to the actual free-base case (neq == 0).
        if self.model.neq == 0:
            roll, pitch = self._roll_pitch(data.xmat[self._pelvis_id])
            roll_rate = float(data.qvel[3])
            pitch_rate = float(data.qvel[4])
            pitch_feedback = (
                self.config.pitch_kp * pitch
                + self.config.pitch_kd * pitch_rate
            )
            roll_feedback = -(
                self.config.roll_kp * roll
                + self.config.roll_kd * roll_rate
            )
            for name in ("left_ankle_pitch_joint", "right_ankle_pitch_joint"):
                additions[name] += pitch_feedback
            for name in ("left_ankle_roll_joint", "right_ankle_roll_joint"):
                additions[name] += roll_feedback

        for joint in self.joints:
            error = target[joint.qpos_adr] - data.qpos[joint.qpos_adr]
            velocity = data.qvel[joint.dof_adr]
            bias = data.qfrc_bias[joint.dof_adr]
            frictionloss = float(self.model.dof_frictionloss[joint.dof_adr])
            friction_compensation = (
                self.config.friction_compensation_scale
                * frictionloss
                * math.tanh(error / self.config.friction_error_width)
            )
            torque = (
                joint.kp * error
                - joint.kd * velocity
                + bias
                + friction_compensation
                + additions[joint.name]
            )
            if bool(self.model.actuator_ctrllimited[joint.actuator_id]):
                lower, upper = self.model.actuator_ctrlrange[joint.actuator_id]
                torque = float(np.clip(torque, lower, upper))
            data.ctrl[joint.actuator_id] = torque
