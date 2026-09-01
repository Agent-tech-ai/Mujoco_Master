#!/usr/bin/env python3
"""Offline Phase 3A-Y motion-conditioned balance experiments.

The controller in this module extends, but does not edit, the frozen Phase
3A-X controller.  Its response model executes before the inherited contact,
joint-limit, saturation, slew-rate, and target-envelope safety mechanisms.
It never connects to a robot and never loads reported effort.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import sys

import numpy as np


HERE = Path(__file__).resolve().parent
CALIBRATION = HERE.parent
P3AX_DIR = CALIBRATION / "phase3ax_constraint_balance"
RUNS = HERE / "runs"
if str(P3AX_DIR) not in sys.path:
    sys.path.insert(0, str(P3AX_DIR))
import phase3ax_core as AX  # noqa: E402


@dataclass(frozen=True)
class AYDesign(AX.AXDesign):
    """Explainable, state-scheduled distribution before the 3A-X safety layer."""

    response_model: str = "CONTINUOUS_ASYMMETRY_GAIN_SCHEDULE"
    pitch_symmetric: tuple[float, float, float, float] = (0.70, 0.10, 0.15, 0.00)
    pitch_asymmetric: tuple[float, float, float, float] = (0.70, 0.10, 0.15, 0.00)
    roll_symmetric: tuple[float, float, float] = (0.70, 0.00, 0.00)
    roll_asymmetric: tuple[float, float, float] = (0.70, 0.00, 0.00)
    pitch_authority: float = 1.90
    roll_authority: float = 1.40
    pitch_total_scale_symmetric: float = 1.0
    pitch_total_scale_asymmetric: float = 1.0
    roll_total_scale_symmetric: float = 1.0
    roll_total_scale_asymmetric: float = 1.0
    asymmetry_midpoint: float = 0.50
    asymmetry_width: float = 0.35
    feature_filter_tau_s: float = 0.12
    motion_energy_threshold_rad_s: float = 0.10
    feature_source: str = "LIVE_SIM_ARM_AND_BASE_STATE_NO_MOTION_ID"


def _smooth01(value: float) -> float:
    x = float(np.clip(value, 0.0, 1.0))
    return x * x * (3.0 - 2.0 * x)


def _normalise_signed(values: list[float], authority: float) -> list[float]:
    denominator = sum(abs(value) for value in values)
    if denominator <= 1e-12:
        return [0.0 for _ in values]
    return [authority * value / denominator for value in values]


class MotionConditionedBalanceController(AX.ConstraintAwareBalanceController):
    """Continuous response distribution followed by the frozen 3A-X safety layer."""

    def __init__(self, model, design: AYDesign):
        super().__init__(model, design)
        self.design: AYDesign = design
        self.filtered_asymmetry = 0.0
        self.filtered_motion_energy = 0.0
        self.filtered_sagittal_fraction = 0.5
        self.last_motion_activity = 0.0
        self.last_asymmetry_gate = 0.0
        self.last_response_gate = 0.0
        self.last_response_features: dict[str, float] = {}
        self.last_raw_desired_weights = {name: 0.0 for name in self.by_name}
        self.last_constraint_scales = {name: 1.0 for name in self.by_name}
        self.last_redistributed_weights = {name: 0.0 for name in self.by_name}

    def _arm_features(self, data) -> None:
        dt = max(float(data.time - self._last_time), 1e-6) if self._last_time is not None else 0.001

        def speed(side: str, tokens: tuple[str, ...] | None = None) -> float:
            values = []
            for name, joint in self.by_name.items():
                if not name.startswith(f"{side}_") or not any(token in name for token in ("shoulder", "elbow", "wrist")):
                    continue
                if tokens is not None and not any(token in name for token in tokens):
                    continue
                values.append(float(data.qvel[joint.dof_adr]))
            return float(np.linalg.norm(values))

        left = speed("left")
        right = speed("right")
        total = left + right
        asymmetry = abs(right - left) / max(total, 1e-6)
        sagittal = speed("left", ("shoulder_pitch", "elbow", "wrist_pitch")) + speed(
            "right", ("shoulder_pitch", "elbow", "wrist_pitch")
        )
        lateral = speed("left", ("shoulder_roll", "shoulder_yaw", "wrist_yaw", "wrist_roll")) + speed(
            "right", ("shoulder_roll", "shoulder_yaw", "wrist_yaw", "wrist_roll")
        )
        sagittal_fraction = sagittal / max(sagittal + lateral, 1e-6)
        alpha = 1.0 - math.exp(-dt / max(self.design.feature_filter_tau_s, 1e-6))
        self.filtered_asymmetry += alpha * (asymmetry - self.filtered_asymmetry)
        self.filtered_motion_energy += alpha * (total - self.filtered_motion_energy)
        self.filtered_sagittal_fraction += alpha * (sagittal_fraction - self.filtered_sagittal_fraction)
        threshold = max(self.design.motion_energy_threshold_rad_s, 1e-6)
        # A fixed deadband prevents landing/settling noise and small base
        # perturbations from masquerading as a deliberate arm gesture.  The
        # schedule reaches full authority at three times the threshold.
        energy_gate = _smooth01((self.filtered_motion_energy - threshold) / (2.0 * threshold))
        # Allocation selection uses current asymmetry so a unilateral onset
        # cannot briefly receive the aggressive symmetric profile while the
        # reporting/filter state catches up.  Motion activity remains filtered.
        coordinate = (asymmetry - self.design.asymmetry_midpoint) / max(self.design.asymmetry_width, 1e-6) + 0.5
        self.last_motion_activity = energy_gate
        self.last_asymmetry_gate = _smooth01(coordinate)
        self.last_response_gate = energy_gate * self.last_asymmetry_gate
        self.last_response_features = {
            "left_arm_speed_norm_rad_s": left,
            "right_arm_speed_norm_rad_s": right,
            "arm_motion_energy_rad_s": total,
            "arm_asymmetry": asymmetry,
            "filtered_arm_motion_energy_rad_s": self.filtered_motion_energy,
            "filtered_arm_asymmetry": self.filtered_asymmetry,
            "sagittal_fraction": self.filtered_sagittal_fraction,
            "motion_activity": self.last_motion_activity,
            "asymmetry_gate": self.last_asymmetry_gate,
            "motion_condition_gate": self.last_response_gate,
        }

    def _response_weights(self) -> dict[str, float]:
        d = self.design
        activity = self.last_motion_activity
        gate = self.last_asymmetry_gate

        def blend(first: tuple[float, ...], second: tuple[float, ...]) -> list[float]:
            return [(1.0 - gate) * a + gate * b for a, b in zip(first, second)]

        active_pitch = blend(d.pitch_symmetric, d.pitch_asymmetric)
        active_roll = blend(d.roll_symmetric, d.roll_asymmetric)
        # With no arm motion the exact frozen 3A-X distribution and authority
        # are restored.  This prevents a gesture-conditioned profile from
        # leaking into pre-roll, post-roll, or standing perturbation recovery.
        baseline_pitch = [d.ankle_pitch_weight, d.hip_pitch_weight, d.knee_pitch_weight, d.waist_pitch_weight]
        baseline_roll = [d.ankle_roll_weight, d.hip_roll_weight, d.waist_roll_weight]
        pitch = [(1.0 - activity) * a + activity * b for a, b in zip(baseline_pitch, active_pitch)]
        roll = [(1.0 - activity) * a + activity * b for a, b in zip(baseline_roll, active_roll)]
        active_pitch_scale = (1.0 - gate) * d.pitch_total_scale_symmetric + gate * d.pitch_total_scale_asymmetric
        active_roll_scale = (1.0 - gate) * d.roll_total_scale_symmetric + gate * d.roll_total_scale_asymmetric
        pitch_scale = (1.0 - activity) + activity * active_pitch_scale
        roll_scale = (1.0 - activity) + activity * active_roll_scale
        # Per-side pitch response is represented by ankle, hip, and knee.  The
        # single waist term is included once in the total absolute authority.
        p_vector = [pitch[0], pitch[1], pitch[2], pitch[0], pitch[1], pitch[2], pitch[3]]
        p_vector = _normalise_signed(p_vector, d.pitch_authority * pitch_scale)
        r_vector = [roll[0], roll[1], roll[0], roll[1], roll[2]]
        r_vector = _normalise_signed(r_vector, d.roll_authority * roll_scale)
        names = (
            "left_ankle_pitch_joint", "left_hip_pitch_joint", "left_knee_joint",
            "right_ankle_pitch_joint", "right_hip_pitch_joint", "right_knee_joint", "waist_pitch_joint",
            "left_ankle_roll_joint", "left_hip_roll_joint",
            "right_ankle_roll_joint", "right_hip_roll_joint", "waist_roll_joint",
        )
        values = p_vector + r_vector
        return dict(zip(names, values))

    def _base_weights(self) -> dict[str, float]:
        # Parent safety allocation calls this method; the returned distribution
        # has already been normalised separately from total feedback request.
        weights = {name: 0.0 for name in super()._base_weights()}
        weights.update(self._response_weights())
        return weights

    def _dynamic_weights(self, state, pitch_nm: float, roll_nm: float) -> dict[str, float]:
        raw = self._base_weights()
        if not self.design.dynamic_allocation:
            self.last_raw_desired_weights = raw.copy()
            self.last_constraint_scales = {name: 1.0 for name in raw}
            self.last_redistributed_weights = raw.copy()
            return raw

        result = {name: 0.0 for name in raw}
        scales = {name: 1.0 for name in raw}
        priorities = {
            "ankle_pitch": 1.00, "ankle_roll": 1.00,
            "hip_pitch": 0.80, "hip_roll": 0.80,
            "knee": 0.60, "waist_pitch": 0.45, "waist_roll": 0.45,
        }
        for channel, demand, groups in (
            ("pitch", pitch_nm, AX.PITCH_GROUPS),
            ("roll", roll_nm, AX.ROLL_GROUPS),
        ):
            names = [name for name in raw if AX._group(name) in groups]
            target_authority = sum(abs(raw[name]) for name in names)
            constrained: dict[str, float] = {}
            for name in names:
                joint = self.by_name[name]
                signed_demand = demand * (1.0 if raw[name] >= 0.0 else -1.0)
                limit = self._directional_limit_scale(joint, state, signed_demand)
                saturation = self._saturation_scale(state.actuator_saturation_fraction[name])
                contact = 1.0
                if channel == "roll" or "hip" in name or "waist" in name:
                    if name.startswith(("left_", "right_")):
                        side = name.split("_", 1)[0]
                        contact = self._contact_scale(
                            getattr(state, f"pelvis_{side}_hip_distance_m"),
                            getattr(state, f"{side}_pelvis_hip_contact"),
                        )
                    else:
                        contact = min(
                            self._contact_scale(state.pelvis_left_hip_distance_m, state.left_pelvis_hip_contact),
                            self._contact_scale(state.pelvis_right_hip_distance_m, state.right_pelvis_hip_contact),
                        )
                # Frozen Phase 3A-X channel priorities are part of the locked
                # safety/allocation baseline and must precede redistribution.
                scale = priorities[AX._group(name) or "waist_roll"] * limit * saturation * contact
                scales[name] = scale
                constrained[name] = raw[name] * scale
            available = sum(abs(value) for value in constrained.values())
            if available > 1e-12:
                for name in names:
                    result[name] = constrained[name] * target_authority / available
        self.last_raw_desired_weights = raw.copy()
        self.last_constraint_scales = scales
        self.last_redistributed_weights = result.copy()
        return result

    def apply(self, data) -> None:
        self._arm_features(data)
        super().apply(data)
        for name, decomposition in self.last_decomposition.items():
            features = self.last_response_features
            decomposition.update({
                "raw_desired_allocation_weight": self.last_raw_desired_weights.get(name, 0.0),
                "preallocation_constraint_scaling": self.last_constraint_scales.get(name, 1.0),
                "redistributed_allocation_weight": self.last_redistributed_weights.get(name, 0.0),
                "response_model_gate": self.last_response_gate,
                "response_motion_activity": self.last_motion_activity,
                "response_asymmetry_gate": self.last_asymmetry_gate,
                "arm_motion_energy_rad_s": features.get("filtered_arm_motion_energy_rad_s", 0.0),
                "arm_asymmetry": features.get("filtered_arm_asymmetry", 0.0),
                "sagittal_fraction": features.get("sagittal_fraction", 0.5),
            })


def run_replay(design: AYDesign, dataset, mode: str, **kwargs):
    """Reuse the frozen runner while substituting only the controller class/output path."""
    RUNS.mkdir(parents=True, exist_ok=True)
    old_controller = AX.ConstraintAwareBalanceController
    old_runs = AX.RUNS
    AX.ConstraintAwareBalanceController = MotionConditionedBalanceController
    AX.RUNS = RUNS
    try:
        return AX.run_replay(design, dataset, mode, **kwargs)
    finally:
        AX.ConstraintAwareBalanceController = old_controller
        AX.RUNS = old_runs


def run_standing(design: AYDesign, dataset, **kwargs):
    return run_replay(design, dataset, "standing", **kwargs)


def datasets():
    return AX.datasets()


def compact_row(summary):
    return AX.compact_row(summary)
