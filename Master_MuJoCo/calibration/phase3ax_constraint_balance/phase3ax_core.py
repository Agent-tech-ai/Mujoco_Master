#!/usr/bin/env python3
"""Constraint-aware simulation balance controller and replay infrastructure.

This module is offline-only.  It never connects to X2, never reads reported
effort, and never modifies MJCF, hardware mapping, or physical parameters.
The independently validated Phase 3A arm gains remain frozen.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import json
import math
from pathlib import Path
import sys
from typing import Iterable

import mujoco
import numpy as np
import pandas as pd


HERE = Path(__file__).resolve().parent
CALIBRATION = HERE.parent
PROJECT = CALIBRATION.parent
P3AR_DIR = CALIBRATION / "phase3ar_controller_redesign"
RUNS = HERE / "runs"
if str(P3AR_DIR) not in sys.path:
    sys.path.insert(0, str(P3AR_DIR))
import phase3ar_core as P3AR  # noqa: E402


SAMPLE_RATE_HZ = 50.0
NUMERICAL_CONTACT_TOLERANCE_M = 0.0005
BALANCE_NAMES = (
    "left_ankle_pitch_joint", "right_ankle_pitch_joint",
    "left_ankle_roll_joint", "right_ankle_roll_joint",
    "left_hip_pitch_joint", "right_hip_pitch_joint",
    "left_hip_roll_joint", "right_hip_roll_joint",
    "left_knee_joint", "right_knee_joint",
    "waist_pitch_joint", "waist_roll_joint",
)
PITCH_GROUPS = ("ankle_pitch", "hip_pitch", "knee", "waist_pitch")
ROLL_GROUPS = ("ankle_roll", "hip_roll", "waist_roll")
GROUP_RATES_NM_S = {
    "ankle_pitch": 40.0,
    "ankle_roll": 25.0,
    "hip_pitch": 30.0,
    "hip_roll": 20.0,
    "knee": 35.0,
    "waist_pitch": 22.0,
    "waist_roll": 16.0,
}


@dataclass(frozen=True)
class AXDesign:
    experiment_id: str
    family: str
    hypothesis: str
    limit_aware: bool = False
    contact_aware: bool = False
    rate_aware: bool = False
    saturation_aware: bool = False
    split_pitch_roll: bool = False
    dynamic_allocation: bool = False
    tracking_gate: bool = False
    safe_standing_reference: bool = False
    pitch_kp: float = 140.0
    pitch_kd: float = 21.0
    roll_kp: float = 70.0
    roll_kd: float = 14.0
    ankle_pitch_weight: float = 0.70
    hip_pitch_weight: float = 0.10
    knee_pitch_weight: float = 0.15
    waist_pitch_weight: float = 0.00
    ankle_roll_weight: float = 0.70
    hip_roll_weight: float = 0.00
    waist_roll_weight: float = 0.00
    standing_reference_scale: float = 1.0
    left_hip_roll_standing_offset_rad: float = 0.0
    right_hip_roll_standing_offset_rad: float = 0.0
    joint_limit_safety_margin_rad: float = 0.050
    joint_limit_warning_width_rad: float = 0.120
    contact_warning_m: float = 0.0025
    contact_hard_m: float = 0.00075
    contact_avoidance_cap_rad: float = 0.035
    contact_avoidance_gain: float = 1.15
    saturation_warning_fraction: float = 0.75
    saturation_hard_fraction: float = 0.95
    reference_slew_rad_s: float = 0.35
    tracking_error_warning_rad: float = 0.060
    tracking_error_hard_rad: float = 0.180
    shoulder_gain_scale: float = 8.0
    wrist_gain_scale: float = 8.0
    warning: str = "SIMULATION CONTROLLER DESIGN; NOT HARDWARE CALIBRATION"


@dataclass
class BalanceSafetyState:
    sim_time: float
    joint_lower_margin_rad: dict[str, float]
    joint_upper_margin_rad: dict[str, float]
    joint_velocity_rad_s: dict[str, float]
    joint_tracking_error_rad: dict[str, float]
    actuator_saturation_fraction: dict[str, float]
    actuator_saturation_margin: dict[str, float]
    pelvis_left_hip_distance_m: float
    pelvis_right_hip_distance_m: float
    left_pelvis_hip_contact: bool
    right_pelvis_hip_contact: bool
    left_foot_contact: bool
    right_foot_contact: bool
    left_foot_slip_m: float
    right_foot_slip_m: float
    base_roll_rad: float
    base_pitch_rad: float
    com_support_margin_m: float


def datasets():
    return P3AR.datasets()


def standing_offsets(design: AXDesign) -> dict[str, float]:
    base = P3AR.Design("phase3ax_offset", "FROZEN", "frozen", "1", "1", standing_reference_scale=design.standing_reference_scale)
    offsets = P3AR.standing_offsets(base)
    offsets["left_hip_roll_joint"] = design.left_hip_roll_standing_offset_rad
    offsets["right_hip_roll_joint"] = design.right_hip_roll_standing_offset_rad
    return offsets


def _group(name: str) -> str | None:
    for token in ("ankle_pitch", "ankle_roll", "hip_pitch", "hip_roll", "knee", "waist_pitch", "waist_roll"):
        if token in name:
            return token
    return None


def _smooth_scale(value: float, hard: float, warning: float) -> float:
    if warning <= hard:
        return float(value > hard)
    x = float(np.clip((value - hard) / (warning - hard), 0.0, 1.0))
    return x * x * (3.0 - 2.0 * x)


class ConstraintAwareBalanceController:
    """Frozen arm PD plus optional constraint-aware balance architecture."""

    def __init__(self, model: mujoco.MjModel, design: AXDesign):
        from master_sim.controller import SimulationStabilityController

        base = SimulationStabilityController(model)
        self.model = model
        self.design = design
        self.joints = []
        for joint in base.joints:
            scale = 1.0
            if joint.name in P3AR.P3A.SHOULDER_JOINTS:
                scale = design.shoulder_gain_scale
            elif joint.name in P3AR.P3A.WRIST_JOINTS:
                scale = design.wrist_gain_scale
            self.joints.append(replace(joint, kp=joint.kp * scale, kd=joint.kd * math.sqrt(scale)))
        self.by_name = {joint.name: joint for joint in self.joints}
        self.target = base.target.copy()
        self.reference_target = base.target.copy()
        self.standing_offset = np.zeros(model.nq, dtype=float)
        self.pelvis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        self.geoms = P3AR.geom_info(model)
        self.feet = {
            "left": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_ankle_roll_link"),
            "right": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_ankle_roll_link"),
        }
        self.initial_foot_xy: dict[str, np.ndarray] | None = None
        self._last_time: float | None = None
        self._last_additions = {joint.name: 0.0 for joint in self.joints}
        self._last_gradient_time = -math.inf
        self._contact_gradients: dict[str, dict[str, float]] = {"left": {}, "right": {}}
        self.last_safety_state: BalanceSafetyState | None = None
        self.last_raw_pitch = 0.0
        self.last_raw_roll = 0.0
        self.last_applied_pitch = 0.0
        self.last_applied_roll = 0.0
        self.last_additions = {joint.name: 0.0 for joint in self.joints}
        self.last_decomposition = {joint.name: {} for joint in self.joints}
        self.last_allocation_weights = {joint.name: 0.0 for joint in self.joints}

    def set_initial_foot_positions(self, data: mujoco.MjData) -> None:
        self.initial_foot_xy = {side: data.xpos[body, :2].copy() for side, body in self.feet.items()}

    @staticmethod
    def _roll_pitch(rotation: np.ndarray) -> tuple[float, float]:
        matrix = rotation.reshape(3, 3)
        roll = math.atan2(float(matrix[2, 1]), float(matrix[2, 2]))
        pitch = math.asin(float(np.clip(-matrix[2, 0], -1.0, 1.0)))
        return roll, pitch

    def _pair_state(self, data: mujoco.MjData, side: str) -> tuple[float, bool]:
        first, second = self.geoms["pelvis"], self.geoms[f"{side}_hip"]
        distance, _ = P3AR.pair_distance(self.model, data, first, second)
        active = bool(P3AR.actual_pair_contacts(self.model, data, first, second))
        return distance, active

    def _support_margin(self, data: mujoco.MjData) -> float:
        # Axis-aligned sole-center support proxy; simulation diagnostic only.
        points = np.array([data.xpos[self.feet[side], :2] for side in ("left", "right")])
        lower = points.min(axis=0) - np.array([0.075, 0.035])
        upper = points.max(axis=0) + np.array([0.075, 0.035])
        com = data.subtree_com[self.pelvis, :2]
        return float(min(com[0] - lower[0], upper[0] - com[0], com[1] - lower[1], upper[1] - com[1]))

    def safety_state(self, data: mujoco.MjData) -> BalanceSafetyState:
        roll, pitch = self._roll_pitch(data.xmat[self.pelvis])
        left_distance, left_active = self._pair_state(data, "left")
        right_distance, right_active = self._pair_state(data, "right")
        _self, _nonfoot, left_foot, right_foot, _penetration = P3AR.contact_counts(self.model, data)
        lower: dict[str, float] = {}
        upper: dict[str, float] = {}
        velocity: dict[str, float] = {}
        error: dict[str, float] = {}
        sat_fraction: dict[str, float] = {}
        sat_margin: dict[str, float] = {}
        for joint in self.joints:
            q = float(data.qpos[joint.qpos_adr])
            lower[joint.name] = q - joint.lower
            upper[joint.name] = joint.upper - q
            velocity[joint.name] = float(data.qvel[joint.dof_adr])
            error[joint.name] = float(self.target[joint.qpos_adr] - q)
            ctrl_limit = max(abs(float(x)) for x in self.model.actuator_ctrlrange[joint.actuator_id])
            fraction = abs(float(data.ctrl[joint.actuator_id])) / ctrl_limit if ctrl_limit else 0.0
            sat_fraction[joint.name] = fraction
            sat_margin[joint.name] = 1.0 - fraction
        if self.initial_foot_xy is None:
            slip = {"left": 0.0, "right": 0.0}
        else:
            slip = {side: float(np.linalg.norm(data.xpos[body, :2] - self.initial_foot_xy[side])) for side, body in self.feet.items()}
        return BalanceSafetyState(
            sim_time=float(data.time),
            joint_lower_margin_rad=lower,
            joint_upper_margin_rad=upper,
            joint_velocity_rad_s=velocity,
            joint_tracking_error_rad=error,
            actuator_saturation_fraction=sat_fraction,
            actuator_saturation_margin=sat_margin,
            pelvis_left_hip_distance_m=left_distance,
            pelvis_right_hip_distance_m=right_distance,
            left_pelvis_hip_contact=left_active,
            right_pelvis_hip_contact=right_active,
            left_foot_contact=left_foot,
            right_foot_contact=right_foot,
            left_foot_slip_m=slip["left"],
            right_foot_slip_m=slip["right"],
            base_roll_rad=roll,
            base_pitch_rad=pitch,
            com_support_margin_m=self._support_margin(data),
        )

    def _contact_scale(self, distance: float, active: bool) -> float:
        if active:
            return 0.0
        # mj_geomDistance may return an exact non-negative query boundary for
        # separated mesh pairs. Do not interpret an inactive zero as contact.
        if distance <= 1e-10:
            return 1.0
        return _smooth_scale(distance, self.design.contact_hard_m, self.design.contact_warning_m)

    def _saturation_scale(self, fraction: float) -> float:
        return 1.0 - _smooth_scale(
            fraction,
            self.design.saturation_warning_fraction,
            self.design.saturation_hard_fraction,
        )

    def _directional_limit_scale(self, joint, state: BalanceSafetyState, addition_nm: float) -> float:
        if addition_nm == 0.0:
            return 1.0
        margin = state.joint_upper_margin_rad[joint.name] if addition_nm > 0.0 else state.joint_lower_margin_rad[joint.name]
        # Velocity buffer prevents a target just inside the envelope from
        # ignoring momentum toward the same mechanical limit.
        velocity = state.joint_velocity_rad_s[joint.name]
        toward = max(0.0, velocity if addition_nm > 0.0 else -velocity)
        effective = margin - self.design.joint_limit_safety_margin_rad - 0.05 * toward
        return _smooth_scale(effective, 0.0, self.design.joint_limit_warning_width_rad)

    def _refresh_contact_gradients(self, data: mujoco.MjData) -> None:
        if float(data.time) - self._last_gradient_time < 0.05:
            return
        epsilon = 2e-4
        original = data.qpos.copy()
        for side in ("left", "right"):
            names = (f"{side}_hip_roll_joint", f"{side}_hip_pitch_joint", "waist_roll_joint", "waist_pitch_joint")
            gradients: dict[str, float] = {}
            for name in names:
                joint = self.by_name.get(name)
                if joint is None:
                    continue
                data.qpos[joint.qpos_adr] = original[joint.qpos_adr] + epsilon
                mujoco.mj_forward(self.model, data)
                plus, _ = self._pair_state(data, side)
                data.qpos[joint.qpos_adr] = original[joint.qpos_adr] - epsilon
                mujoco.mj_forward(self.model, data)
                minus, _ = self._pair_state(data, side)
                gradients[name] = float((plus - minus) / (2.0 * epsilon))
                data.qpos[joint.qpos_adr] = original[joint.qpos_adr]
            self._contact_gradients[side] = gradients
        data.qpos[:] = original
        mujoco.mj_forward(self.model, data)
        self._last_gradient_time = float(data.time)

    def _contact_avoidance_additions(self, data: mujoco.MjData, state: BalanceSafetyState) -> dict[str, float]:
        result = {joint.name: 0.0 for joint in self.joints}
        if not self.design.contact_aware:
            return result
        if (
            (state.pelvis_left_hip_distance_m >= self.design.contact_warning_m or (state.pelvis_left_hip_distance_m <= 1e-10 and not state.left_pelvis_hip_contact))
            and (state.pelvis_right_hip_distance_m >= self.design.contact_warning_m or (state.pelvis_right_hip_distance_m <= 1e-10 and not state.right_pelvis_hip_contact))
        ):
            return result
        self._refresh_contact_gradients(data)
        for side, distance, active in (
            ("left", state.pelvis_left_hip_distance_m, state.left_pelvis_hip_contact),
            ("right", state.pelvis_right_hip_distance_m, state.right_pelvis_hip_contact),
        ):
            if distance <= 1e-10 and not active:
                continue
            deficit = self.design.contact_warning_m - distance
            if deficit <= 0.0:
                continue
            gradients = self._contact_gradients[side]
            norm2 = sum(value * value for value in gradients.values())
            if norm2 < 1e-10:
                continue
            for name, gradient in gradients.items():
                joint = self.by_name[name]
                q_correction = float(np.clip(self.design.contact_avoidance_gain * deficit * gradient / norm2, -self.design.contact_avoidance_cap_rad, self.design.contact_avoidance_cap_rad))
                result[name] += joint.kp * q_correction
        return result

    def _base_weights(self) -> dict[str, float]:
        d = self.design
        weights: dict[str, float] = {}
        for side in ("left", "right"):
            weights[f"{side}_ankle_pitch_joint"] = d.ankle_pitch_weight
            weights[f"{side}_hip_pitch_joint"] = d.hip_pitch_weight
            weights[f"{side}_knee_joint"] = d.knee_pitch_weight
            weights[f"{side}_ankle_roll_joint"] = d.ankle_roll_weight
            weights[f"{side}_hip_roll_joint"] = d.hip_roll_weight
        weights["waist_pitch_joint"] = d.waist_pitch_weight
        weights["waist_roll_joint"] = d.waist_roll_weight
        return weights

    def _dynamic_weights(self, state: BalanceSafetyState, pitch_nm: float, roll_nm: float) -> dict[str, float]:
        base = self._base_weights()
        if not self.design.dynamic_allocation:
            return base
        priorities = {
            "ankle_pitch": 1.00, "ankle_roll": 1.00,
            "hip_pitch": 0.80, "hip_roll": 0.80,
            "knee": 0.60, "waist_pitch": 0.45, "waist_roll": 0.45,
        }
        result = {name: 0.0 for name in base}
        for channel, demand, names in (
            ("pitch", pitch_nm, [name for name in base if _group(name) in PITCH_GROUPS]),
            ("roll", roll_nm, [name for name in base if _group(name) in ROLL_GROUPS]),
        ):
            target_total = sum(base[name] for name in names)
            scores: dict[str, float] = {}
            for name in names:
                joint = self.by_name[name]
                limit = self._directional_limit_scale(joint, state, demand)
                saturation = self._saturation_scale(state.actuator_saturation_fraction[name])
                contact = 1.0
                if channel == "roll" or "hip" in name or "waist" in name:
                    side_scales = []
                    if name.startswith("left_") or name.startswith("right_"):
                        side = name.split("_", 1)[0]
                        distance = getattr(state, f"pelvis_{side}_hip_distance_m")
                        active = getattr(state, f"{side}_pelvis_hip_contact")
                        side_scales.append(self._contact_scale(distance, active))
                    else:
                        side_scales = [
                            self._contact_scale(state.pelvis_left_hip_distance_m, state.left_pelvis_hip_contact),
                            self._contact_scale(state.pelvis_right_hip_distance_m, state.right_pelvis_hip_contact),
                        ]
                    contact = min(side_scales)
                # A zero design weight is ineligible. Constraint awareness may
                # redistribute an established contribution, but must not
                # silently activate a previously unused joint channel.
                scores[name] = base[name] * priorities[_group(name) or "waist_roll"] * limit * saturation * contact
            total = sum(scores.values())
            if total > 1e-12:
                # Keep total channel authority bounded while redistributing
                # away from constrained joints.
                for name in names:
                    result[name] = target_total * scores[name] / total
        return result

    def update_reference_target(self, joint, requested: float, data: mujoco.MjData, dt: float, mode: str) -> float:
        requested = float(np.clip(requested, joint.lower, joint.upper))
        if not self.design.tracking_gate or mode != "whole_body" or joint.name not in BALANCE_NAMES:
            self.target[joint.qpos_adr] = requested
            return requested
        current_target = float(self.target[joint.qpos_adr])
        q = float(data.qpos[joint.qpos_adr])
        error = abs(current_target - q)
        scale = 1.0 - _smooth_scale(error, self.design.tracking_error_warning_rad, self.design.tracking_error_hard_rad)
        max_step = self.design.reference_slew_rad_s * max(dt, 1e-6) * max(0.10, scale)
        step = float(np.clip(requested - current_target, -max_step, max_step))
        # A large existing error may recover, but a new reference cannot drive
        # it farther away at full authority.
        if abs(current_target + step - q) > abs(current_target - q) and error >= self.design.tracking_error_hard_rad:
            step *= 0.10
        safe_lower = joint.lower + self.design.joint_limit_safety_margin_rad
        safe_upper = joint.upper - self.design.joint_limit_safety_margin_rad
        filtered = float(np.clip(current_target + step, safe_lower, safe_upper))
        self.target[joint.qpos_adr] = filtered
        return filtered

    def apply(self, data: mujoco.MjData) -> None:
        d = self.design
        state = self.safety_state(data)
        dt = max(float(data.time - self._last_time), 1e-6) if self._last_time is not None else 0.001
        roll, pitch = state.base_roll_rad, state.base_pitch_rad
        pitch_rate = float(data.qvel[4])
        roll_rate = float(data.qvel[3])
        raw_pitch = d.pitch_kp * pitch + d.pitch_kd * pitch_rate
        raw_roll = -(d.roll_kp * roll + d.roll_kd * roll_rate)
        if d.split_pitch_roll:
            # Independent deadbands prevent the other plane's small noise from
            # consuming allocation authority.
            pitch_nm = 0.0 if abs(pitch) < math.radians(0.10) and abs(pitch_rate) < 0.01 else raw_pitch
            roll_nm = 0.0 if abs(roll) < math.radians(0.08) and abs(roll_rate) < 0.01 else raw_roll
        else:
            pitch_nm, roll_nm = raw_pitch, raw_roll
        weights = self._dynamic_weights(state, pitch_nm, roll_nm)
        avoidance = self._contact_avoidance_additions(data, state)
        additions = {joint.name: 0.0 for joint in self.joints}
        decomposition: dict[str, dict[str, float]] = {}
        for joint in self.joints:
            group = _group(joint.name)
            channel = pitch_nm if group in PITCH_GROUPS else roll_nm if group in ROLL_GROUPS else 0.0
            allocated = weights.get(joint.name, 0.0) * channel
            contact_scale = 1.0
            if d.contact_aware and (group in ROLL_GROUPS or group in ("hip_pitch", "waist_pitch")):
                if joint.name.startswith("left_"):
                    contact_scale = self._contact_scale(state.pelvis_left_hip_distance_m, state.left_pelvis_hip_contact)
                elif joint.name.startswith("right_"):
                    contact_scale = self._contact_scale(state.pelvis_right_hip_distance_m, state.right_pelvis_hip_contact)
                else:
                    contact_scale = min(
                        self._contact_scale(state.pelvis_left_hip_distance_m, state.left_pelvis_hip_contact),
                        self._contact_scale(state.pelvis_right_hip_distance_m, state.right_pelvis_hip_contact),
                    )
            after_contact = allocated * contact_scale + avoidance[joint.name]
            limit_scale = self._directional_limit_scale(joint, state, after_contact) if d.limit_aware else 1.0
            after_limit = after_contact * limit_scale
            sat_scale = self._saturation_scale(state.actuator_saturation_fraction[joint.name]) if d.saturation_aware else 1.0
            desired = after_limit * sat_scale
            rate_scale = 1.0
            if d.rate_aware and group is not None:
                maximum = GROUP_RATES_NM_S[group] * dt
                previous = self._last_additions[joint.name]
                limited = float(np.clip(desired, previous - maximum, previous + maximum))
                rate_scale = limited / desired if abs(desired) > 1e-12 else 1.0
                desired = limited
            # Final hard target envelope is applied after slew limiting so a
            # stale correction cannot survive into an unsafe state.
            final_limit_scale = self._directional_limit_scale(joint, state, desired) if d.limit_aware else 1.0
            desired *= final_limit_scale
            hard_envelope_scale = 1.0
            if d.limit_aware and abs(desired) > 1e-12:
                safe_lower = joint.lower + d.joint_limit_safety_margin_rad
                safe_upper = joint.upper - d.joint_limit_safety_margin_rad
                nominal = float(self.target[joint.qpos_adr])
                lower_addition = (safe_lower - nominal) * joint.kp
                upper_addition = (safe_upper - nominal) * joint.kp
                before_envelope = desired
                desired = float(np.clip(desired, lower_addition, upper_addition))
                hard_envelope_scale = desired / before_envelope
            additions[joint.name] = desired
            equivalent_q = desired / joint.kp
            final_target = float(self.target[joint.qpos_adr] + equivalent_q)
            decomposition[joint.name] = {
                "reference_target_rad": float(self.reference_target[joint.qpos_adr]),
                "standing_reference_offset_rad": float(self.standing_offset[joint.qpos_adr]),
                "pitch_balance_correction_nm": allocated if group in PITCH_GROUPS else 0.0,
                "roll_balance_correction_nm": allocated if group in ROLL_GROUPS else 0.0,
                "contact_avoidance_correction_nm": avoidance[joint.name],
                "contact_avoidance_scaling": contact_scale,
                "limit_scaling": limit_scale * final_limit_scale * hard_envelope_scale,
                "saturation_scaling": sat_scale,
                "rate_scaling": rate_scale,
                "allocation_weight": weights.get(joint.name, 0.0),
                "final_balance_addition_nm": desired,
                "equivalent_balance_target_offset_rad": equivalent_q,
                "final_joint_target_rad": final_target,
            }

            error = float(self.target[joint.qpos_adr] - data.qpos[joint.qpos_adr])
            velocity = float(data.qvel[joint.dof_adr])
            bias = float(data.qfrc_bias[joint.dof_adr])
            frictionloss = float(self.model.dof_frictionloss[joint.dof_adr])
            friction = 1.5 * frictionloss * math.tanh(error / 0.005)
            torque = joint.kp * error - joint.kd * velocity + bias + friction + desired
            if bool(self.model.actuator_ctrllimited[joint.actuator_id]):
                lower, upper = self.model.actuator_ctrlrange[joint.actuator_id]
                torque = float(np.clip(torque, lower, upper))
            data.ctrl[joint.actuator_id] = torque

        self._last_additions = additions.copy()
        self._last_time = float(data.time)
        self.last_safety_state = state
        self.last_raw_pitch, self.last_raw_roll = raw_pitch, raw_roll
        self.last_applied_pitch = sum(additions.get(f"{side}_ankle_pitch_joint", 0.0) for side in ("left", "right")) / 2.0
        self.last_applied_roll = sum(additions.get(f"{side}_ankle_roll_joint", 0.0) for side in ("left", "right")) / 2.0
        self.last_additions = additions
        self.last_decomposition = decomposition
        self.last_allocation_weights = weights


def _initial_perturbation(data: mujoco.MjData, controller: ConstraintAwareBalanceController, perturbation: dict[str, object] | None) -> None:
    if not perturbation:
        return
    roll = math.radians(float(perturbation.get("base_roll_deg", 0.0)))
    pitch = math.radians(float(perturbation.get("base_pitch_deg", 0.0)))
    if roll or pitch:
        quat = np.zeros(4, dtype=np.float64)
        mujoco.mju_euler2Quat(quat, np.array([roll, pitch, 0.0], dtype=np.float64), "xyz")
        data.qpos[3:7] = quat
    name = str(perturbation.get("joint_name", ""))
    if name:
        joint = controller.by_name[name]
        data.qpos[joint.qpos_adr] = float(np.clip(
            data.qpos[joint.qpos_adr] + math.radians(float(perturbation.get("joint_delta_deg", 0.0))),
            joint.lower,
            joint.upper,
        ))


def run_replay(
    design: AXDesign,
    dataset,
    mode: str,
    *,
    pre_s: float = 5.0,
    post_s: float = 5.0,
    save_detail: bool = False,
    perturbation: dict[str, object] | None = None,
) -> dict[str, object]:
    RUNS.mkdir(parents=True, exist_ok=True)
    reference_frame, real = P3AR.load_frames(dataset)
    if mode == "standing":
        t_start, t_end = -3.0, 7.0
    else:
        t_start = max(float(reference_frame.t.min()), -pre_s)
        t_end = min(float(reference_frame.t.max()), dataset.motion_end_s + post_s)
    reference_frame = reference_frame[reference_frame.t.between(t_start, t_end)].copy()
    reference = P3AR.P3A.Reference(reference_frame, "linear", 50.0)
    model = P3AR.load_model(free_base=True)
    model.opt.timestep = 0.001
    errors = P3AR.validate_model(model)
    if errors:
        raise RuntimeError(errors)
    controller = ConstraintAwareBalanceController(model, design)
    data = mujoco.MjData(model)
    mapped = sorted(set(controller.by_name) & set(reference.data))
    active = P3AR.active_joints(reference_frame, mode) & set(mapped)
    initial = {name: reference.at(name, t_start, "position") for name in mapped}
    offsets = standing_offsets(design)
    for name in mapped:
        joint = controller.by_name[name]
        reference_value = initial[name]
        offset = offsets.get(name, 0.0)
        target = reference_value + offset
        controller.reference_target[joint.qpos_adr] = reference_value
        controller.standing_offset[joint.qpos_adr] = offset
        controller.target[joint.qpos_adr] = float(np.clip(target, joint.lower, joint.upper))
        data.qpos[joint.qpos_adr] = initial[name]
        data.qvel[joint.dof_adr] = 0.0
    _initial_perturbation(data, controller, perturbation)
    mujoco.mj_forward(model, data)
    data.qpos[2] -= P3AR.P3A.foot_surface_minimum(model, data)
    mujoco.mj_forward(model, data)
    controller.set_initial_foot_positions(data)

    joint_rows: list[dict[str, object]] = []
    safety_rows: list[dict[str, object]] = []
    contact_rows: list[dict[str, object]] = []
    next_sample = next_control = 0.0
    duration = t_end - t_start
    fall_time: float | None = None
    target_clip_count = 0
    pelvis = controller.pelvis
    while data.time < duration - 1e-12:
        sim_time = float(data.time)
        t = t_start + sim_time
        if sim_time + 1e-12 >= next_control:
            dt_control = 0.001
            for name in mapped:
                joint = controller.by_name[name]
                reference_value = reference.at(name, t, "position")
                active_safety_offset = (
                    offsets.get(name, 0.0)
                    if design.safe_standing_reference and name in active and name in BALANCE_NAMES
                    else 0.0
                )
                requested = reference_value + active_safety_offset if name in active else initial[name] + offsets.get(name, 0.0)
                controller.reference_target[joint.qpos_adr] = reference_value
                controller.standing_offset[joint.qpos_adr] = active_safety_offset if name in active else offsets.get(name, 0.0)
                clipped = float(np.clip(requested, joint.lower, joint.upper))
                target_clip_count += int(abs(clipped - requested) > 1e-10)
                controller.update_reference_target(joint, clipped, data, dt_control, mode)
            controller.apply(data)
            next_control += dt_control
        mujoco.mj_step(model, data)
        roll, pitch, yaw = P3AR.P3A.rpy(data.xmat[pelvis])
        if fall_time is None and (float(data.xpos[pelvis, 2]) < 0.30 or max(abs(roll), abs(pitch)) > math.radians(45.0)):
            fall_time = float(data.time)
        if data.time + 1e-12 < next_sample:
            continue
        sim_time = float(data.time)
        t = t_start + sim_time
        state = controller.safety_state(data)
        self_count, nonfoot, left_foot, right_foot, max_penetration = P3AR.contact_counts(model, data)
        left_contacts = P3AR.actual_pair_contacts(model, data, controller.geoms["pelvis"], controller.geoms["left_hip"])
        right_contacts = P3AR.actual_pair_contacts(model, data, controller.geoms["pelvis"], controller.geoms["right_hip"])
        safety_rows.append({
            "sim_time": sim_time, "t": t,
            "base_x": float(data.xpos[pelvis, 0]), "base_y": float(data.xpos[pelvis, 1]), "base_z": float(data.xpos[pelvis, 2]),
            "base_roll_rad": roll, "base_pitch_rad": pitch, "base_yaw_rad": yaw,
            "com_x": float(data.subtree_com[pelvis, 0]), "com_y": float(data.subtree_com[pelvis, 1]), "com_z": float(data.subtree_com[pelvis, 2]),
            "com_support_margin_m": state.com_support_margin_m,
            "raw_pitch_feedback_nm": controller.last_raw_pitch, "applied_pitch_feedback_nm": controller.last_applied_pitch,
            "raw_roll_feedback_nm": controller.last_raw_roll, "applied_roll_feedback_nm": controller.last_applied_roll,
            "left_pelvis_hip_distance_m": state.pelvis_left_hip_distance_m,
            "right_pelvis_hip_distance_m": state.pelvis_right_hip_distance_m,
            "left_pelvis_hip_contact": int(state.left_pelvis_hip_contact),
            "right_pelvis_hip_contact": int(state.right_pelvis_hip_contact),
            "self_collision_contact_count": self_count, "nonfoot_ground_contact_count": nonfoot,
            "left_foot_contact": int(left_foot), "right_foot_contact": int(right_foot),
            "left_foot_slip_m": state.left_foot_slip_m, "right_foot_slip_m": state.right_foot_slip_m,
            "max_contact_penetration_m": max_penetration,
            "minimum_joint_margin_rad": min(min(state.joint_lower_margin_rad.values()), min(state.joint_upper_margin_rad.values())),
            "minimum_actuator_margin": min(state.actuator_saturation_margin.values()),
        })
        for side, distance, contacts in (
            ("left", state.pelvis_left_hip_distance_m, left_contacts),
            ("right", state.pelvis_right_hip_distance_m, right_contacts),
        ):
            contact_rows.append({
                "sim_time": sim_time, "t": t, "side": side,
                "signed_geom_distance_m": distance,
                "contact_active": int(bool(contacts)),
                "contact_dist_m": float(contacts[0].dist) if contacts else None,
            })
        for name in mapped:
            joint = controller.by_name[name]
            ctrl = float(data.ctrl[joint.actuator_id])
            ctrl_limit = max(abs(float(x)) for x in model.actuator_ctrlrange[joint.actuator_id])
            dec = controller.last_decomposition[name]
            joint_rows.append({
                "sim_time": sim_time, "t": t, "joint_name": name,
                "input_mode": "MEASURED_REAL_TRAJECTORY" if name in active else "STANDING_TARGET",
                "reference_position": reference.at(name, t, "position"),
                "target_position": float(controller.target[joint.qpos_adr]),
                "position": float(data.qpos[joint.qpos_adr]), "velocity": float(data.qvel[joint.dof_adr]),
                "tracking_error_rad": float(controller.target[joint.qpos_adr] - data.qpos[joint.qpos_adr]),
                "ctrl_nm": ctrl, "ctrl_saturation_fraction": abs(ctrl) / ctrl_limit if ctrl_limit else 0.0,
                "lower_margin_rad": state.joint_lower_margin_rad[name], "upper_margin_rad": state.joint_upper_margin_rad[name],
                **dec,
            })
        next_sample += 1.0 / SAMPLE_RATE_HZ

    joints = pd.DataFrame(joint_rows)
    safety = pd.DataFrame(safety_rows)
    contacts = pd.DataFrame(contact_rows)
    tracking, balance = P3AR._metrics(dataset, real, joints) if mode != "standing" else ([], [])
    active_contacts = contacts[contacts.contact_active == 1]
    positive = contacts[contacts.signed_geom_distance_m > 1e-8].signed_geom_distance_m
    time_saturated = joints.groupby("sim_time").ctrl_saturation_fraction.max() >= 0.98
    consecutive = 0
    max_consecutive = 0
    for value in time_saturated:
        consecutive = consecutive + 1 if value else 0
        max_consecutive = max(max_consecutive, consecutive)
    hip_by_time = contacts.groupby("sim_time").contact_active.sum()
    other_self = 0
    for row in safety.itertuples(index=False):
        other_self += int(int(row.self_collision_contact_count) > int(hip_by_time.get(row.sim_time, 0)))
    minimum_limit_margin = float(min(joints.lower_margin_rad.min(), joints.upper_margin_rad.min()))
    summary: dict[str, object] = {
        "experiment_id": design.experiment_id, "family": design.family,
        "dataset": dataset.name, "mode": mode, "design": asdict(design),
        "perturbation": perturbation or {}, "time_window_s": [t_start, t_end],
        "reported_effort_loaded": False, "robot_connected": False,
        "physical_parameters_modified": False, "mjcf_modified": False, "hardware_mapping_modified": False,
        "stable_no_fall": fall_time is None, "fall_time_s": fall_time,
        "self_collision_samples": int((safety.self_collision_contact_count > 0).sum()),
        "pelvis_hip_contact_samples": int(active_contacts.sim_time.nunique()),
        "pelvis_hip_contact_duration_s": float(active_contacts.sim_time.nunique() / SAMPLE_RATE_HZ),
        "other_self_collision_samples": other_self,
        "maximum_pelvis_hip_penetration_m": float(max(0.0, -active_contacts.contact_dist_m.min())) if not active_contacts.empty else 0.0,
        "minimum_positive_precontact_distance_m": float(positive.min()) if not positive.empty else 0.0,
        "distance_to_numerical_tolerance_m": float((positive.min() if not positive.empty else 0.0) - NUMERICAL_CONTACT_TOLERANCE_M),
        "nonfoot_ground_contact_samples": int((safety.nonfoot_ground_contact_count > 0).sum()),
        "minimum_limit_margin_rad": minimum_limit_margin,
        "limit_violation_samples": int(((joints.lower_margin_rad < 0.0) | (joints.upper_margin_rad < 0.0)).sum()),
        "target_clip_samples": int(target_clip_count),
        "persistent_saturation_fraction": float(time_saturated.mean()),
        "max_consecutive_saturation_s": float(max_consecutive / SAMPLE_RATE_HZ),
        "maximum_saturation_fraction": float(joints.ctrl_saturation_fraction.max()),
        "both_feet_contact_fraction": float(((safety.left_foot_contact == 1) & (safety.right_foot_contact == 1)).mean()),
        "minimum_com_support_margin_m": float(safety.com_support_margin_m.min()),
        "maximum_left_foot_slip_proxy_m": float(safety.left_foot_slip_m.max()),
        "maximum_right_foot_slip_proxy_m": float(safety.right_foot_slip_m.max()),
        "maximum_abs_tilt_deg": float(np.degrees(max(safety.base_roll_rad.abs().max(), safety.base_pitch_rad.abs().max()))),
        "tracking_metrics": tracking, "balance_metrics": balance,
    }
    summary["contact_safety_pass"] = bool(
        summary["pelvis_hip_contact_samples"] == 0
        and summary["minimum_positive_precontact_distance_m"] >= design.contact_hard_m
        and summary["other_self_collision_samples"] == 0
    )
    summary["limit_management_pass"] = bool(summary["limit_violation_samples"] == 0 and minimum_limit_margin >= 0.0)
    summary["saturation_management_pass"] = bool(summary["persistent_saturation_fraction"] <= 0.01 and summary["max_consecutive_saturation_s"] <= 0.20)
    summary["safety_pass"] = bool(
        summary["stable_no_fall"] and summary["contact_safety_pass"]
        and summary["limit_management_pass"] and summary["saturation_management_pass"]
        and summary["nonfoot_ground_contact_samples"] == 0
    )
    suffix = ""
    if perturbation:
        suffix = "__perturb_" + str(perturbation.get("id", "unnamed"))
    stem = f"{design.experiment_id}__{dataset.name}__{mode}{suffix}"
    (RUNS / f"{stem}_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    if save_detail:
        joints.to_csv(RUNS / f"{stem}_joint_log.csv", index=False)
        safety.to_csv(RUNS / f"{stem}_safety_log.csv", index=False)
        contacts.to_csv(RUNS / f"{stem}_contact_log.csv", index=False)
        decomposition_columns = [
            "sim_time", "t", "joint_name", "reference_target_rad", "standing_reference_offset_rad",
            "pitch_balance_correction_nm", "roll_balance_correction_nm", "contact_avoidance_correction_nm",
            "contact_avoidance_scaling", "limit_scaling", "saturation_scaling", "rate_scaling",
            "allocation_weight", "final_balance_addition_nm", "equivalent_balance_target_offset_rad",
            "final_joint_target_rad",
        ]
        joints[decomposition_columns].to_csv(RUNS / f"{stem}_command_decomposition.csv", index=False)
    return summary


def run_standing(design: AXDesign, dataset, **kwargs) -> dict[str, object]:
    return run_replay(design, dataset, "standing", **kwargs)


def arm_tracking_retained(summary: dict[str, object]) -> bool:
    active = {row["joint_name"]: row for row in summary["tracking_metrics"] if float(row["real_excursion_rad"]) >= 0.02}
    shoulder = [row for name, row in active.items() if "shoulder_roll" in name]
    wrist = [row for name, row in active.items() if "wrist_yaw" in name]
    shoulder_ok = all((row["lag_s"] is None or float(row["lag_s"]) <= 0.14) and float(row["rmse_rad"]) <= 0.16 for row in shoulder)
    wrist_ok = all((row["lag_s"] is None or float(row["lag_s"]) <= 0.22) and float(row["rmse_rad"]) <= 0.19 for row in wrist)
    return bool(shoulder_ok and wrist_ok)


def compact_row(summary: dict[str, object]) -> dict[str, object]:
    balance_ratios = [abs(math.log(max(float(row["excursion_ratio"]), 1e-6))) for row in summary["balance_metrics"] if row["excursion_ratio"] is not None]
    return {
        "experiment_id": summary["experiment_id"], "family": summary["family"],
        "dataset": summary["dataset"], "mode": summary["mode"],
        "stable_no_fall": summary["stable_no_fall"], "fall_time_s": summary["fall_time_s"],
        "contact_safety_pass": summary["contact_safety_pass"],
        "pelvis_hip_contact_samples": summary["pelvis_hip_contact_samples"],
        "minimum_positive_precontact_distance_m": summary["minimum_positive_precontact_distance_m"],
        "maximum_pelvis_hip_penetration_m": summary["maximum_pelvis_hip_penetration_m"],
        "limit_management_pass": summary["limit_management_pass"],
        "minimum_limit_margin_rad": summary["minimum_limit_margin_rad"],
        "saturation_management_pass": summary["saturation_management_pass"],
        "persistent_saturation_fraction": summary["persistent_saturation_fraction"],
        "max_consecutive_saturation_s": summary["max_consecutive_saturation_s"],
        "minimum_com_support_margin_m": summary["minimum_com_support_margin_m"],
        "arm_tracking_retained": arm_tracking_retained(summary),
        "balance_shape_score": float(np.mean(balance_ratios)) if balance_ratios else None,
        "safety_pass": summary["safety_pass"],
    }
