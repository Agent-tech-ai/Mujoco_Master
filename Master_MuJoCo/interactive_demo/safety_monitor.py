"""Outer interactive-demo monitor around the frozen Phase 3A-X/Y safety layer."""

from __future__ import annotations

from dataclasses import dataclass
import math

import mujoco


EXPECTED_CLAP_BODY_PAIR = frozenset(("left_wrist_roll_link", "right_wrist_roll_link"))
# Evidence envelope: min/max across original/+8%, arm-only/whole-body episodes,
# expanded only by the documented <=1 ms cross-condition onset variation.
EXPECTED_CLAP_WINDOWS_S = ((1.601, 1.761), (2.777, 2.962), (3.976, 4.163))


@dataclass
class SafetySnapshot:
    safe: bool
    reasons: tuple[str, ...]
    base_roll_deg: float
    base_pitch_deg: float
    pelvis_height_m: float
    minimum_joint_margin_rad: float
    maximum_saturation_fraction: float
    maximum_saturation_duration_s: float
    unexpected_self_contacts: int
    expected_clap_contacts: int
    nonfoot_ground_contacts: int
    maximum_contact_penetration_m: float
    left_foot_slip_m: float
    right_foot_slip_m: float


class SafetyMonitor:
    """Observe existing hard gates; never changes model or controller parameters."""

    FALL_TILT_RAD = math.radians(45.0)  # Phase 3A-X replay fall gate.
    FALL_PELVIS_HEIGHT_M = 0.30         # Phase 3A-X replay fall gate.
    MAX_SATURATION_DURATION_S = 0.20    # Phase 3A-X acceptance gate.

    def __init__(self, model: mujoco.MjModel, controller):
        self.model = model
        self.controller = controller
        self.floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self.foot_bodies = {"left_ankle_roll_link", "right_ankle_roll_link"}
        self._saturation_duration = 0.0
        self.maximum_saturation_duration = 0.0
        self.total_expected_clap_contacts = 0
        self.maximum_left_slip = 0.0
        self.maximum_right_slip = 0.0
        self.evaluations = 0

    def reset(self) -> None:
        self._saturation_duration = 0.0
        self.maximum_saturation_duration = 0.0
        self.total_expected_clap_contacts = 0
        self.maximum_left_slip = 0.0
        self.maximum_right_slip = 0.0
        self.evaluations = 0

    @staticmethod
    def _within_clap_window(action: str | None, motion_time_s: float | None) -> bool:
        epsilon = 1e-9  # Float comparison only; does not widen the evidence window.
        return bool(
            action == "clap"
            and motion_time_s is not None
            and any(start - epsilon <= motion_time_s <= end + epsilon for start, end in EXPECTED_CLAP_WINDOWS_S)
        )

    def evaluate(
        self,
        data: mujoco.MjData,
        dt: float,
        action: str | None,
        motion_time_s: float | None,
    ) -> SafetySnapshot:
        self.evaluations += 1
        state = self.controller.safety_state(data)
        minimum_margin = min(
            min(state.joint_lower_margin_rad.values()),
            min(state.joint_upper_margin_rad.values()),
        )
        max_saturation = max(state.actuator_saturation_fraction.values())
        if max_saturation >= self.controller.design.saturation_hard_fraction:
            self._saturation_duration += dt
        else:
            self._saturation_duration = 0.0
        self.maximum_saturation_duration = max(self.maximum_saturation_duration, self._saturation_duration)

        unexpected_self = expected_clap = nonfoot = 0
        max_penetration = 0.0
        for index in range(data.ncon):
            contact = data.contact[index]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            body1 = int(self.model.geom_bodyid[geom1])
            body2 = int(self.model.geom_bodyid[geom2])
            name1 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body1) or "world"
            name2 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body2) or "world"
            max_penetration = max(max_penetration, max(0.0, -float(contact.dist)))
            if geom1 == self.floor or geom2 == self.floor:
                other = name2 if geom1 == self.floor else name1
                if other not in self.foot_bodies:
                    nonfoot += 1
                continue
            if body1 == 0 or body2 == 0 or body1 == body2:
                continue
            pair = frozenset((name1, name2))
            if pair == EXPECTED_CLAP_BODY_PAIR and self._within_clap_window(action, motion_time_s):
                expected_clap += 1
            else:
                unexpected_self += 1
        self.total_expected_clap_contacts += expected_clap
        self.maximum_left_slip = max(self.maximum_left_slip, state.left_foot_slip_m)
        self.maximum_right_slip = max(self.maximum_right_slip, state.right_foot_slip_m)

        reasons: list[str] = []
        if max(abs(state.base_roll_rad), abs(state.base_pitch_rad)) > self.FALL_TILT_RAD:
            reasons.append("BASE_TILT_FALL_GATE")
        pelvis_height = float(data.xpos[self.controller.pelvis, 2])
        if pelvis_height < self.FALL_PELVIS_HEIGHT_M:
            reasons.append("PELVIS_HEIGHT_FALL_GATE")
        if minimum_margin < 0.0:
            reasons.append("JOINT_LIMIT_VIOLATION")
        if self._saturation_duration > self.MAX_SATURATION_DURATION_S:
            reasons.append("PERSISTENT_ACTUATOR_SATURATION")
        if unexpected_self:
            reasons.append("UNEXPECTED_SELF_COLLISION")
        if nonfoot:
            reasons.append("NONFOOT_GROUND_CONTACT")
        return SafetySnapshot(
            safe=not reasons,
            reasons=tuple(reasons),
            base_roll_deg=math.degrees(state.base_roll_rad),
            base_pitch_deg=math.degrees(state.base_pitch_rad),
            pelvis_height_m=pelvis_height,
            minimum_joint_margin_rad=minimum_margin,
            maximum_saturation_fraction=max_saturation,
            maximum_saturation_duration_s=self.maximum_saturation_duration,
            unexpected_self_contacts=unexpected_self,
            expected_clap_contacts=expected_clap,
            nonfoot_ground_contacts=nonfoot,
            maximum_contact_penetration_m=max_penetration,
            left_foot_slip_m=state.left_foot_slip_m,
            right_foot_slip_m=state.right_foot_slip_m,
        )
