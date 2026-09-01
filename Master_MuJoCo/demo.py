"""Contact-driven single-step keyboard demo for the Master MuJoCo model.

Only joint targets, the existing PD controller, actuator torques, contacts, and
MuJoCo physics are used.  This module never writes the free-joint position or
velocity, never teleports the robot, and never applies an external propulsive
force.
"""

from __future__ import annotations

import argparse
import csv
import ctypes
from ctypes import wintypes
from dataclasses import dataclass
from enum import Enum
import math
import os
from pathlib import Path
import time
from typing import Iterable

import mujoco
import numpy as np

from master_sim.controller import JointPositionController, POSES
from master_sim.model import FREE_SCENE, load_model, validate_model


HEAD_JOINT = "head_yaw_joint"
WAIST_JOINT = "waist_yaw_joint"
PELVIS_BODY = "pelvis"
LEFT_FOOT_BODY = "left_ankle_roll_link"
RIGHT_FOOT_BODY = "right_ankle_roll_link"
FLOOR_GEOM_NAME = "floor"

READY_SECONDS = 5.0
SAFE_ROLL_PITCH = math.radians(20.0)
FREEZE_ROLL_PITCH = math.radians(30.0)
STABLE_ROLL_PITCH = math.radians(8.0)
HEAD_RATE = math.radians(24.0)
WAIST_RATE = math.radians(20.0)

HELP = """Master MuJoCo Contact-Driven Single-Step Demo

Single-step commands (one key press, ignored while a step is active):
  8 / 2 : one forward / backward step
  4 / 6 : one left / right lateral step
  7 / 9 : one left / right turning step

Upper body:
  O / P : head left / head right
  K / L : waist left / waist right

Other:
  5     : report that continuous speed modes are disabled
  0     : abort active step and smoothly enter RECOVER
  Space : smoothly enter RECOVER
  R     : center head and waist
  Esc   : exit
"""


def _smoothstep(value: float) -> float:
    value = float(np.clip(value, 0.0, 1.0))
    return value * value * (3.0 - 2.0 * value)


def _quat_to_rpy(quat_wxyz: np.ndarray) -> tuple[float, float, float]:
    matrix = np.empty(9, dtype=float)
    mujoco.mju_quat2Mat(matrix, quat_wxyz)
    rotation = matrix.reshape(3, 3)
    roll = math.atan2(rotation[2, 1], rotation[2, 2])
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return roll, pitch, yaw


def _angle_delta(end: float, start: float) -> float:
    return math.atan2(math.sin(end - start), math.cos(end - start))


class GaitState(str, Enum):
    STAND = "STAND"
    SHIFT_TO_LEFT = "SHIFT_TO_LEFT"
    LIFT_RIGHT = "LIFT_RIGHT"
    SWING_RIGHT = "SWING_RIGHT"
    LAND_RIGHT = "LAND_RIGHT"
    DOUBLE_SUPPORT = "DOUBLE_SUPPORT"
    SHIFT_TO_RIGHT = "SHIFT_TO_RIGHT"
    LIFT_LEFT = "LIFT_LEFT"
    SWING_LEFT = "SWING_LEFT"
    LAND_LEFT = "LAND_LEFT"
    RECOVER = "RECOVER"


STATE_MAX_SECONDS = {
    GaitState.SHIFT_TO_LEFT: 1.60,
    GaitState.LIFT_RIGHT: 1.30,
    GaitState.SWING_RIGHT: 0.70,
    GaitState.LAND_RIGHT: 1.00,
    GaitState.DOUBLE_SUPPORT: 2.00,
    GaitState.SHIFT_TO_RIGHT: 1.60,
    GaitState.LIFT_LEFT: 1.30,
    GaitState.SWING_LEFT: 0.70,
    GaitState.LAND_LEFT: 1.00,
    GaitState.RECOVER: 6.00,
}


@dataclass(frozen=True)
class StepRequest:
    name: str
    forward_m: float = 0.0
    lateral_m: float = 0.0
    yaw_rad: float = 0.0
    lift_only_side: str | None = None


KEY_TO_STEP = {
    "8": StepRequest("forward", forward_m=0.040),
    "2": StepRequest("backward", forward_m=-0.040),
    "4": StepRequest("left", lateral_m=0.020),
    "6": StepRequest("right", lateral_m=-0.020),
    "7": StepRequest("turn_left", yaw_rad=math.radians(3.0)),
    "9": StepRequest("turn_right", yaw_rad=math.radians(-3.0)),
}


@dataclass
class RobotSample:
    time: float
    pelvis_pos: np.ndarray
    pelvis_quat: np.ndarray
    roll: float
    pitch: float
    yaw: float
    left_foot_pos: np.ndarray
    right_foot_pos: np.ndarray
    left_contact: bool
    right_contact: bool
    left_force: float
    right_force: float
    max_ctrl: float
    saturated_actuators: int
    head_yaw: float
    waist_yaw: float
    finite: bool


class FootContactMonitor:
    """Discovers foot collision geoms and reads contacts from mjData."""

    def __init__(self, model: mujoco.MjModel) -> None:
        self.model = model
        self.floor_geom = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, FLOOR_GEOM_NAME
        )
        if self.floor_geom < 0:
            raise RuntimeError(f"Floor geom {FLOOR_GEOM_NAME!r} not found")

        self.pelvis_body = self._body_id(PELVIS_BODY)
        self.left_body = self._body_id(LEFT_FOOT_BODY)
        self.right_body = self._body_id(RIGHT_FOOT_BODY)
        self.head_qpos_adr = self._joint_qpos_adr(HEAD_JOINT)
        self.waist_qpos_adr = self._joint_qpos_adr(WAIST_JOINT)
        self.left_geoms = self._collision_geoms(self.left_body)
        self.right_geoms = self._collision_geoms(self.right_body)
        if not self.left_geoms or not self.right_geoms:
            raise RuntimeError("No collision-enabled foot geoms were discovered")

    def _body_id(self, name: str) -> int:
        body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, name
        )
        if body_id < 0:
            raise RuntimeError(f"Body {name!r} not found")
        return body_id

    def _joint_qpos_adr(self, name: str) -> int:
        joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, name
        )
        if joint_id < 0:
            raise RuntimeError(f"Joint {name!r} not found")
        return int(self.model.jnt_qposadr[joint_id])

    def _collision_geoms(self, body_id: int) -> tuple[int, ...]:
        return tuple(
            geom_id
            for geom_id in range(self.model.ngeom)
            if int(self.model.geom_bodyid[geom_id]) == body_id
            and (
                int(self.model.geom_contype[geom_id]) != 0
                or int(self.model.geom_conaffinity[geom_id]) != 0
            )
        )

    def geom_label(self, geom_id: int) -> str:
        name = mujoco.mj_id2name(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id
        )
        body_id = int(self.model.geom_bodyid[geom_id])
        body_name = mujoco.mj_id2name(
            self.model, mujoco.mjtObj.mjOBJ_BODY, body_id
        )
        if name:
            return f"{name} (id={geom_id}, body={body_name})"
        return f"<unnamed geom id={geom_id}, body={body_name}>"

    def print_discovery(self) -> None:
        print(f"Floor geom: {self.geom_label(self.floor_geom)}")
        print("Left foot collision geoms:")
        for geom_id in self.left_geoms:
            print(f"  - {self.geom_label(geom_id)}")
        print("Right foot collision geoms:")
        for geom_id in self.right_geoms:
            print(f"  - {self.geom_label(geom_id)}")

    def foot_position(self, data: mujoco.MjData, side: str) -> np.ndarray:
        # Report the named ankle/foot rigid body's world origin.  Averaging
        # distributed collision-sphere centers is not a physical foot frame
        # and can hide a valid lift when the sphere layout is asymmetric.
        body_id = self.left_body if side == "left" else self.right_body
        return data.xpos[body_id].copy()

    def _contact_data(
        self, data: mujoco.MjData, foot_geoms: tuple[int, ...]
    ) -> tuple[bool, float]:
        foot_set = set(foot_geoms)
        total_normal_force = 0.0
        contact = False
        force = np.zeros(6, dtype=float)
        for contact_id in range(data.ncon):
            item = data.contact[contact_id]
            geom1 = int(item.geom1)
            geom2 = int(item.geom2)
            pair_has_floor = (
                geom1 == self.floor_geom or geom2 == self.floor_geom
            )
            pair_has_foot = geom1 in foot_set or geom2 in foot_set
            if pair_has_floor and pair_has_foot:
                contact = True
                mujoco.mj_contactForce(
                    self.model, data, contact_id, force
                )
                total_normal_force += max(0.0, float(force[0]))
        return contact, total_normal_force

    def sample(self, data: mujoco.MjData) -> RobotSample:
        left_contact, left_force = self._contact_data(
            data, self.left_geoms
        )
        right_contact, right_force = self._contact_data(
            data, self.right_geoms
        )
        roll, pitch, yaw = _quat_to_rpy(
            data.xquat[self.pelvis_body]
        )
        limited = np.asarray(
            self.model.actuator_ctrllimited, dtype=bool
        )
        ranges = self.model.actuator_ctrlrange
        saturated = np.zeros(self.model.nu, dtype=bool)
        if self.model.nu:
            saturated = np.logical_or(
                data.ctrl <= ranges[:, 0] + 1e-5,
                data.ctrl >= ranges[:, 1] - 1e-5,
            ) & limited
        finite = bool(
            np.all(np.isfinite(data.qpos))
            and np.all(np.isfinite(data.qvel))
            and np.all(np.isfinite(data.ctrl))
        )
        return RobotSample(
            time=float(data.time),
            pelvis_pos=data.xpos[self.pelvis_body].copy(),
            pelvis_quat=data.xquat[self.pelvis_body].copy(),
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            left_foot_pos=self.foot_position(data, "left"),
            right_foot_pos=self.foot_position(data, "right"),
            left_contact=left_contact,
            right_contact=right_contact,
            left_force=left_force,
            right_force=right_force,
            max_ctrl=float(np.max(np.abs(data.ctrl)))
            if self.model.nu
            else 0.0,
            saturated_actuators=int(np.count_nonzero(saturated)),
            head_yaw=float(data.qpos[self.head_qpos_adr]),
            waist_yaw=float(data.qpos[self.waist_qpos_adr]),
            finite=finite,
        )


class ContactStepController:
    """Finite-state, contact-gated single-step controller."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self.model = model
        self.pd = JointPositionController(model)
        self.pd.set_pose("home")
        self.pd.initialize_data(data)
        self.joints = {joint.name: joint for joint in self.pd.joints}
        self.monitor = FootContactMonitor(model)
        self.home = {name: 0.0 for name in self.joints}
        self.home.update(POSES["home"])
        self.current = {
            name: float(data.qpos[joint.qpos_adr])
            for name, joint in self.joints.items()
        }
        self.head_target = self.current[HEAD_JOINT]
        self.waist_target = self.current[WAIST_JOINT]
        self.state = GaitState.STAND
        self.state_enter_time = float(data.time)
        self.state_entry_sample = self.monitor.sample(data)
        self.request: StepRequest | None = None
        self.command_name = "idle"
        self.after_double_support = GaitState.RECOVER
        self.next_swing_side = "right"
        self.active_swing_side = "right"
        self.stand_hip_pitch_bias = 0.0
        self.stand_hip_roll_bias = 0.0
        self.stand_hip_yaw_bias = 0.0
        self.last_shift_sign = 0.0
        self.condition_seconds = 0.0
        self.both_off_seconds = 0.0
        self.stance_off_seconds = 0.0
        self.failure_reason = ""
        self.frozen = False
        self.completed_actions = 0
        self.actuator_saturation_seen = False
        self.target_violation_seen = False
        self.standing_height = float(
            data.xpos[self.monitor.pelvis_body, 2]
        )
        sample = self.monitor.sample(data)
        self.previous_roll = sample.roll
        self.previous_pitch = sample.pitch
        self.placed_offsets = {
            "left": {"pitch": 0.0, "roll": 0.0, "yaw": 0.0},
            "right": {"pitch": 0.0, "roll": 0.0, "yaw": 0.0},
        }
        self.lift_reference_z = {"left": 0.0, "right": 0.0}
        self._write_targets()

    @property
    def elapsed(self) -> float:
        return self._last_sample_time - self.state_enter_time

    def _write_targets(self) -> None:
        self.pd.set_targets(self.current)

    def base_attitude(
        self, data: mujoco.MjData
    ) -> tuple[float, float, float]:
        return _quat_to_rpy(data.xquat[self.monitor.pelvis_body])

    def _enter(
        self, state: GaitState, data: mujoco.MjData, reason: str = ""
    ) -> None:
        if state == GaitState.RECOVER:
            # Forget completed Cartesian-placement offsets at the start of
            # recovery.  The target rate limiter makes this a smooth return;
            # retaining them would make "home reached" impossible.
            self.placed_offsets = {
                "left": {"pitch": 0.0, "roll": 0.0, "yaw": 0.0},
                "right": {"pitch": 0.0, "roll": 0.0, "yaw": 0.0},
            }
            if (
                not reason
                and self.request is not None
                and self.request.lift_only_side is None
                and self.request.forward_m != 0.0
            ):
                self.stand_hip_pitch_bias = math.copysign(
                    0.05, self.request.forward_m
                )
                self.stand_hip_roll_bias = 0.0
                self.stand_hip_yaw_bias = 0.0
            elif (
                not reason
                and self.request is not None
                and self.request.lift_only_side is None
                and self.request.lateral_m != 0.0
            ):
                self.stand_hip_pitch_bias = 0.0
                self.stand_hip_roll_bias = (
                    -1.75 * self.request.lateral_m
                )
                self.stand_hip_yaw_bias = 0.0
            elif (
                not reason
                and self.request is not None
                and self.request.lift_only_side is None
                and self.request.yaw_rad != 0.0
            ):
                self.stand_hip_pitch_bias = 0.0
                self.stand_hip_roll_bias = 0.0
                self.stand_hip_yaw_bias = (
                    (
                        -2.50 * self.request.yaw_rad
                        if self.request.yaw_rad > 0.0
                        else -4.20 * self.request.yaw_rad
                    )
                )
            elif reason:
                self.stand_hip_pitch_bias = 0.0
                self.stand_hip_roll_bias = 0.0
                self.stand_hip_yaw_bias = 0.0
        self.state = state
        self.state_enter_time = float(data.time)
        self.state_entry_sample = self.monitor.sample(data)
        if state == GaitState.LIFT_RIGHT:
            self.lift_reference_z["right"] = float(
                self.state_entry_sample.right_foot_pos[2]
            )
        elif state == GaitState.LIFT_LEFT:
            self.lift_reference_z["left"] = float(
                self.state_entry_sample.left_foot_pos[2]
            )
        self.condition_seconds = 0.0
        self.stance_off_seconds = 0.0
        suffix = f" ({reason})" if reason else ""
        sample = self.state_entry_sample
        print(
            f"Gait state -> {state.value}{suffix}; "
            f"contacts L/R={sample.left_contact}/{sample.right_contact}; "
            f"forces L/R={sample.left_force:.1f}/{sample.right_force:.1f}N"
        )

    def ready_for_step(self, data: mujoco.MjData) -> bool:
        sample = self.monitor.sample(data)
        return (
            not self.frozen
            and self.state == GaitState.STAND
            and data.time >= READY_SECONDS
            and sample.left_contact
            and sample.right_contact
            and abs(sample.roll) < STABLE_ROLL_PITCH
            and abs(sample.pitch) < STABLE_ROLL_PITCH
        )

    def request_step(
        self, data: mujoco.MjData, request: StepRequest
    ) -> bool:
        if not self.ready_for_step(data):
            print(
                f"Ignored {request.name}: controller is not in stable STAND"
            )
            return False
        self.request = request
        self.command_name = request.name
        self.failure_reason = ""
        self.stand_hip_pitch_bias = 0.0
        self.stand_hip_roll_bias = 0.0
        self.stand_hip_yaw_bias = 0.0
        self.placed_offsets = {
            "left": {"pitch": 0.0, "roll": 0.0, "yaw": 0.0},
            "right": {"pitch": 0.0, "roll": 0.0, "yaw": 0.0},
        }
        if request.lift_only_side is not None:
            self.active_swing_side = request.lift_only_side
        elif request.lateral_m > 0.0 or request.yaw_rad > 0.0:
            self.active_swing_side = "left"
        elif request.lateral_m < 0.0 or request.yaw_rad < 0.0:
            self.active_swing_side = "right"
        else:
            self.active_swing_side = self.next_swing_side
        if self.active_swing_side == "left":
            self._enter(GaitState.SHIFT_TO_RIGHT, data)
        else:
            self._enter(GaitState.SHIFT_TO_LEFT, data)
        return True

    def request_lift_test(self, data: mujoco.MjData, side: str) -> bool:
        if side not in ("left", "right"):
            raise ValueError(side)
        return self.request_step(
            data,
            StepRequest(
                name=f"lift_{side}", lift_only_side=side
            ),
        )

    def abort_to_recover(
        self, data: mujoco.MjData, reason: str
    ) -> None:
        if self.frozen:
            return
        if self.state != GaitState.RECOVER:
            self.failure_reason = reason
            self.command_name = "recover"
            self._enter(GaitState.RECOVER, data, reason)

    def center_upper_body(self) -> None:
        self.head_target = 0.0
        self.waist_target = 0.0

    def recenter(self, data: mujoco.MjData) -> None:
        self.center_upper_body()
        self.stand_hip_pitch_bias = 0.0
        self.stand_hip_roll_bias = 0.0
        self.stand_hip_yaw_bias = 0.0
        self.abort_to_recover(data, "0 recenter requested")

    def _freeze(self, reason: str) -> None:
        if not self.frozen:
            self.frozen = True
            self.failure_reason = reason
            self.command_name = "frozen"
            print(f"SAFE FREEZE: {reason}")

    def _swing_side(self) -> str | None:
        if self.state in (
            GaitState.LIFT_RIGHT,
            GaitState.SWING_RIGHT,
            GaitState.LAND_RIGHT,
        ):
            return "right"
        if self.state in (
            GaitState.LIFT_LEFT,
            GaitState.SWING_LEFT,
            GaitState.LAND_LEFT,
        ):
            return "left"
        return None

    def _shift_sign(self) -> float:
        if self.state in (
            GaitState.SHIFT_TO_LEFT,
            GaitState.LIFT_RIGHT,
            GaitState.SWING_RIGHT,
            GaitState.LAND_RIGHT,
        ):
            return 1.0
        if self.state in (
            GaitState.SHIFT_TO_RIGHT,
            GaitState.LIFT_LEFT,
            GaitState.SWING_LEFT,
            GaitState.LAND_LEFT,
        ):
            return -1.0
        if self.state == GaitState.DOUBLE_SUPPORT:
            progress = _smoothstep(self.elapsed / 0.30)
            return self.last_shift_sign * (1.0 - progress)
        return 0.0

    def _swing_goal(self, side: str) -> dict[str, float]:
        if self.request is None:
            return {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
        # Joint-coordinate signs are based on the imported model's actual axes.
        return {
            "pitch": -2.0 * self.request.forward_m,
            "roll": 2.0 * self.request.lateral_m,
            "yaw": self.request.yaw_rad,
        }

    def _apply_placed_offsets(
        self, desired: dict[str, float]
    ) -> None:
        if self.state in (
            GaitState.LIFT_RIGHT,
            GaitState.LIFT_LEFT,
            GaitState.SWING_RIGHT,
            GaitState.SWING_LEFT,
        ):
            sagittal_coupling = 1.0
        elif self.state in (
            GaitState.LAND_RIGHT,
            GaitState.LAND_LEFT,
        ):
            sagittal_coupling = 1.0 - _smoothstep(
                self.elapsed / 0.45
            )
        else:
            sagittal_coupling = 0.0
        for side, offsets in self.placed_offsets.items():
            desired[f"{side}_hip_pitch_joint"] += offsets["pitch"]
            # Differential IK around the lifted pose: coupled knee and ankle
            # terms translate the foot in X while holding its height and
            # pitch, instead of using hip pitch alone and scraping the floor.
            desired[f"{side}_knee_joint"] += (
                -0.573 * offsets["pitch"] * sagittal_coupling
            )
            desired[f"{side}_ankle_pitch_joint"] += (
                -0.427 * offsets["pitch"] * sagittal_coupling
            )
            desired[f"{side}_hip_roll_joint"] += offsets["roll"]
            desired[f"{side}_hip_yaw_joint"] += offsets["yaw"]

    def _apply_shift_targets(
        self, desired: dict[str, float], shift_sign: float
    ) -> None:
        if not shift_sign:
            return
        swing = self._swing_side()
        shift_blend = (
            _smoothstep(self.elapsed / 0.90)
            if self.state
            in (GaitState.SHIFT_TO_LEFT, GaitState.SHIFT_TO_RIGHT)
            else 1.0
        )
        shift_state = self.state in (
            GaitState.SHIFT_TO_LEFT,
            GaitState.SHIFT_TO_RIGHT,
        )
        for side in ("left", "right"):
            # Both legs form the lateral parallelogram while the feet are
            # planted.  Once the swing foot is airborne, halve its roll
            # component so its world-Y position stays near the lift-off
            # position instead of arcing outward by roughly 6 cm.
            # Preserve the full unload geometry until contact confirms
            # lift-off; only then retract the airborne leg laterally.
            retract_airborne = self.state in (
                GaitState.SWING_RIGHT,
                GaitState.SWING_LEFT,
                GaitState.LAND_RIGHT,
                GaitState.LAND_LEFT,
            )
            if side == swing and self.state in (
                GaitState.LIFT_RIGHT,
                GaitState.LIFT_LEFT,
            ):
                # First flex the unloaded leg, then level its roll.  Leveling
                # immediately would reload the edge of the sole; leveling
                # after flexion clears the last foot geom from the floor.
                scale = 1.0 - 0.50 * _smoothstep(
                    (self.elapsed - 0.30) / 0.20
                )
            elif side == swing and retract_airborne:
                scale = 0.50
            else:
                scale = 1.0
            ankle_amplitude = (
                0.25
                if shift_state or side == swing
                else 0.21
            )
            desired[f"{side}_hip_roll_joint"] += (
                -0.23 * shift_sign * scale * shift_blend
            )
            desired[f"{side}_ankle_roll_joint"] += (
                ankle_amplitude
                * shift_sign
                * scale
                * shift_blend
            )
        desired["waist_roll_joint"] += (
            -0.15 * shift_sign * shift_blend
        )
        desired["left_shoulder_roll_joint"] += (
            0.15 * shift_sign * shift_blend
        )
        desired["right_shoulder_roll_joint"] += (
            0.15 * shift_sign * shift_blend
        )

    def _state_targets(
        self, data: mujoco.MjData
    ) -> dict[str, float]:
        desired = dict(self.home)
        if self.state in (GaitState.STAND, GaitState.RECOVER):
            desired["left_hip_pitch_joint"] += (
                self.stand_hip_pitch_bias
            )
            desired["right_hip_pitch_joint"] += (
                self.stand_hip_pitch_bias
            )
            desired["left_hip_roll_joint"] += (
                self.stand_hip_roll_bias
            )
            desired["right_hip_roll_joint"] += (
                self.stand_hip_roll_bias
            )
            desired["left_hip_yaw_joint"] += (
                self.stand_hip_yaw_bias
            )
            desired["right_hip_yaw_joint"] += (
                self.stand_hip_yaw_bias
            )
        desired[HEAD_JOINT] = self.head_target
        desired[WAIST_JOINT] = self.waist_target
        self._apply_placed_offsets(desired)

        shift_sign = self._shift_sign()
        # Measured on this model: this combination shifts the pelvis laterally
        # with both feet planted and unloads the future swing foot.
        self._apply_shift_targets(desired, shift_sign)

        swing = self._swing_side()
        if swing is not None:
            if self.state in (
                GaitState.LIFT_RIGHT,
                GaitState.LIFT_LEFT,
            ):
                lift_blend = _smoothstep(self.elapsed / 0.35)
            elif self.state in (
                GaitState.SWING_RIGHT,
                GaitState.SWING_LEFT,
            ):
                lift_blend = 1.0
            else:
                lift_blend = 1.0 - _smoothstep(
                    self.elapsed / 0.45
                )

            # This measured near-zero sagittal-foot-Jacobian combination
            # flexes the swing leg without scraping it backward and injecting
            # a forward impulse into the pelvis.
            lift_knee = 0.48 if swing == "right" else 0.45
            lift_hip = -0.235 if swing == "right" else -0.22
            lift_ankle = -0.41 if swing == "right" else -0.40
            desired[f"{swing}_knee_joint"] += lift_knee * lift_blend
            desired[f"{swing}_hip_pitch_joint"] += lift_hip * lift_blend
            desired[f"{swing}_ankle_pitch_joint"] += (
                lift_ankle * lift_blend
            )

            if self.state in (
                GaitState.SWING_RIGHT,
                GaitState.SWING_LEFT,
            ):
                swing_progress = _smoothstep(self.elapsed / 0.10)
                goal = self._swing_goal(swing)
                for key in ("pitch", "roll", "yaw"):
                    self.placed_offsets[swing][key] = (
                        goal[key] * swing_progress
                    )
            elif self.state in (
                GaitState.LAND_RIGHT,
                GaitState.LAND_LEFT,
            ):
                goal = self._swing_goal(swing)
                self.placed_offsets[swing].update(goal)

            # Refresh placed offsets after SWING/LAND updates.
            desired = dict(self.home)
            desired[HEAD_JOINT] = self.head_target
            desired[WAIST_JOINT] = self.waist_target
            self._apply_placed_offsets(desired)
            self._apply_shift_targets(desired, shift_sign)
            desired[f"{swing}_knee_joint"] += lift_knee * lift_blend
            desired[f"{swing}_hip_pitch_joint"] += lift_hip * lift_blend
            desired[f"{swing}_ankle_pitch_joint"] += (
                lift_ankle * lift_blend
            )

        return desired

    def _add_balance_feedback(
        self,
        data: mujoco.MjData,
        sample: RobotSample,
        desired: dict[str, float],
        dt: float,
    ) -> None:
        roll_rate = (sample.roll - self.previous_roll) / max(dt, 1e-9)
        pitch_rate = (
            sample.pitch - self.previous_pitch
        ) / max(dt, 1e-9)
        self.previous_roll = sample.roll
        self.previous_pitch = sample.pitch

        # This free-base model needs anticipatory ankle braking before
        # touchdown; waiting until DOUBLE_SUPPORT lets sagittal momentum grow
        # beyond the ankle torque authority.
        velocity_gain = 5.0
        pitch_correction = float(
            np.clip(
                3.0 * sample.pitch
                + 0.05 * pitch_rate
                + velocity_gain * float(data.qvel[0]),
                -0.65,
                0.40,
            )
        )
        braking_state = self.state in (
            GaitState.STAND,
            GaitState.LAND_RIGHT,
            GaitState.LAND_LEFT,
            GaitState.DOUBLE_SUPPORT,
            GaitState.RECOVER,
        )
        roll_velocity_gain = 0.60 if braking_state else 0.0
        wide_roll_authority = self.state in (
            GaitState.STAND,
            GaitState.DOUBLE_SUPPORT,
            GaitState.RECOVER,
        )
        roll_limit = 0.16 if wide_roll_authority else 0.04
        lateral_position_feedback = 0.0
        if wide_roll_authority:
            foot_mid_y = 0.5 * (
                sample.left_foot_pos[1] + sample.right_foot_pos[1]
            )
            lateral_position_feedback = -2.0 * (
                sample.pelvis_pos[1] - foot_mid_y
            )
        roll_correction = float(
            np.clip(
                2.0 * sample.roll
                + 0.03 * roll_rate
                - 1.4 * float(data.qvel[1])
                + lateral_position_feedback,
                -roll_limit,
                roll_limit,
            )
        )
        swing = self._swing_side()
        for side in ("left", "right"):
            side_pitch_correction = pitch_correction
            # While a foot is unloaded its ankle cannot balance the base.
            # Keeping feedback on it corrupts the Cartesian lift trajectory;
            # route the feedback through the actual stance ankle instead.
            if side == swing:
                continue
            if (
                self.request is not None
                and self.state
                in (
                    GaitState.SHIFT_TO_LEFT,
                    GaitState.SHIFT_TO_RIGHT,
                )
            ):
                sagittal_drive = (
                    (
                        -1.25 * self.request.forward_m
                        if self.state == GaitState.SHIFT_TO_LEFT
                        else -3.75 * self.request.forward_m
                    )
                    if self.request.forward_m >= 0.0
                    else -7.50 * self.request.forward_m
                )
                side_pitch_correction += sagittal_drive
            pitch_name = f"{side}_ankle_pitch_joint"
            pitch_joint = self.joints[pitch_name]
            desired[pitch_name] = float(
                np.clip(
                    desired[pitch_name] + side_pitch_correction,
                    pitch_joint.lower,
                    pitch_joint.upper,
                )
            )
            roll_name = f"{side}_ankle_roll_joint"
            roll_joint = self.joints[roll_name]
            desired[roll_name] = float(
                np.clip(
                    desired[roll_name] + roll_correction,
                    roll_joint.lower,
                    roll_joint.upper,
                )
            )

    def _state_condition(
        self, condition: bool, dt: float, required: float
    ) -> bool:
        self.condition_seconds = (
            self.condition_seconds + dt if condition else 0.0
        )
        return self.condition_seconds >= required

    def _swing_progress_ok(
        self, sample: RobotSample, side: str
    ) -> bool:
        if self.request is None:
            return False
        if self.request.lift_only_side:
            return True
        start = (
            self.state_entry_sample.right_foot_pos
            if side == "right"
            else self.state_entry_sample.left_foot_pos
        )
        current = (
            sample.right_foot_pos
            if side == "right"
            else sample.left_foot_pos
        )
        delta = current - start
        checks: list[bool] = []
        if self.request.forward_m:
            checks.append(
                delta[0] * math.copysign(1.0, self.request.forward_m)
                >= min(0.008, abs(self.request.forward_m) * 0.40)
            )
        if self.request.lateral_m:
            checks.append(
                delta[1] * math.copysign(1.0, self.request.lateral_m)
                >= min(0.006, abs(self.request.lateral_m) * 0.40)
            )
        if self.request.yaw_rad:
            goal = self._swing_goal(side)["yaw"]
            checks.append(
                abs(self.placed_offsets[side]["yaw"])
                >= abs(goal) * 0.90
            )
        return all(checks) if checks else True

    def _advance_state(
        self, data: mujoco.MjData, sample: RobotSample, dt: float
    ) -> None:
        elapsed = float(data.time - self.state_enter_time)
        self._last_sample_time = float(data.time)

        if self.state == GaitState.STAND:
            return

        if self.state in (
            GaitState.SHIFT_TO_LEFT,
            GaitState.SHIFT_TO_RIGHT,
        ):
            shift_left = self.state == GaitState.SHIFT_TO_LEFT
            swing_force = (
                sample.right_force if shift_left else sample.left_force
            )
            foot_mid_y = 0.5 * (
                sample.left_foot_pos[1] + sample.right_foot_pos[1]
            )
            pelvis_relative_y = sample.pelvis_pos[1] - foot_mid_y
            direction_ok = (
                pelvis_relative_y >= 0.020
                if shift_left
                else pelvis_relative_y <= -0.020
            )
            condition = (
                sample.left_contact
                and sample.right_contact
                and swing_force < 100.0
                and direction_ok
                and abs(float(data.qvel[1])) < 0.060
                and abs(sample.roll) < math.radians(12.0)
                and abs(sample.pitch) < math.radians(12.0)
            )
            if self._state_condition(condition, dt, 0.030):
                self.last_shift_sign = 1.0 if shift_left else -1.0
                self._enter(
                    GaitState.LIFT_RIGHT
                    if shift_left
                    else GaitState.LIFT_LEFT,
                    data,
                )
                return

        elif self.state in (
            GaitState.LIFT_RIGHT,
            GaitState.LIFT_LEFT,
        ):
            side = (
                "right"
                if self.state == GaitState.LIFT_RIGHT
                else "left"
            )
            contact = (
                sample.right_contact
                if side == "right"
                else sample.left_contact
            )
            condition = not contact
            if self._state_condition(condition, dt, 0.040):
                self._enter(
                    GaitState.SWING_RIGHT
                    if side == "right"
                    else GaitState.SWING_LEFT,
                    data,
                )
                return

        elif self.state in (
            GaitState.SWING_RIGHT,
            GaitState.SWING_LEFT,
        ):
            side = (
                "right"
                if self.state == GaitState.SWING_RIGHT
                else "left"
            )
            min_swing_duration = (
                0.15
                if self.request is not None
                and self.request.lift_only_side is not None
                else 0.25
            )
            if (
                elapsed >= min_swing_duration
            ):
                self._enter(
                    GaitState.LAND_RIGHT
                    if side == "right"
                    else GaitState.LAND_LEFT,
                    data,
                )
                return

        elif self.state in (
            GaitState.LAND_RIGHT,
            GaitState.LAND_LEFT,
        ):
            both_contact = (
                sample.left_contact and sample.right_contact
            )
            if self._state_condition(both_contact, dt, 0.100):
                landed_right = self.state == GaitState.LAND_RIGHT
                lift_only = (
                    self.request is not None
                    and self.request.lift_only_side is not None
                )
                self.after_double_support = GaitState.RECOVER
                self._enter(GaitState.DOUBLE_SUPPORT, data)
                return

        elif self.state == GaitState.DOUBLE_SUPPORT:
            foot_mid_y = 0.5 * (
                sample.left_foot_pos[1] + sample.right_foot_pos[1]
            )
            pelvis_relative_y = sample.pelvis_pos[1] - foot_mid_y
            condition = (
                sample.left_contact
                and sample.right_contact
                and abs(sample.roll) < math.radians(12.0)
                and abs(sample.pitch) < math.radians(12.0)
                and abs(pelvis_relative_y) < 0.060
                and abs(float(data.qvel[1])) < 0.080
            )
            if self._state_condition(condition, dt, 0.500):
                self._enter(self.after_double_support, data)
                return

        elif self.state == GaitState.RECOVER:
            recovery_target = dict(self.home)
            for side in ("left", "right"):
                recovery_target[f"{side}_hip_pitch_joint"] += (
                    self.stand_hip_pitch_bias
                )
                recovery_target[f"{side}_hip_roll_joint"] += (
                    self.stand_hip_roll_bias
                )
                recovery_target[f"{side}_hip_yaw_joint"] += (
                    self.stand_hip_yaw_bias
                )
            target_error = max(
                abs(self.current[name] - recovery_target[name])
                for name in self.joints
                if "ankle_pitch" not in name
            )
            horizontal_speed = float(
                np.linalg.norm(data.qvel[:2])
            )
            condition = (
                sample.left_contact
                and sample.right_contact
                and abs(sample.roll) < STABLE_ROLL_PITCH
                and abs(sample.pitch) < STABLE_ROLL_PITCH
                and sample.pelvis_pos[2] > self.standing_height - 0.060
                and horizontal_speed < 0.08
                and target_error < 0.060
            )
            if self._state_condition(condition, dt, 0.500):
                completed_request = self.request
                self.state = GaitState.STAND
                self.state_enter_time = float(data.time)
                self.state_entry_sample = sample
                self.command_name = "idle"
                self.request = None
                self.placed_offsets = {
                    "left": {
                        "pitch": 0.0,
                        "roll": 0.0,
                        "yaw": 0.0,
                    },
                    "right": {
                        "pitch": 0.0,
                        "roll": 0.0,
                        "yaw": 0.0,
                    },
                }
                self.completed_actions += 1
                if (
                    completed_request is not None
                    and completed_request.lift_only_side is None
                ):
                    self.next_swing_side = "right"
                print("Gait state -> STAND (recovery complete)")
                return

        timeout = STATE_MAX_SECONDS.get(self.state)
        if timeout is not None and elapsed > timeout:
            if self.state == GaitState.RECOVER:
                self._freeze(
                    f"RECOVER timeout after {elapsed:.2f}s; "
                    f"original reason: {self.failure_reason or 'none'}"
                )
            else:
                self.abort_to_recover(
                    data,
                    f"{self.state.value} contact/pose condition "
                    f"timeout after {elapsed:.2f}s",
                )

    def _safety_check(
        self, data: mujoco.MjData, sample: RobotSample, dt: float
    ) -> None:
        if not sample.finite:
            self._freeze("NaN/Inf detected")
            return

        if not sample.left_contact and not sample.right_contact:
            self.both_off_seconds += dt
        else:
            self.both_off_seconds = 0.0
        if self.both_off_seconds > 0.100:
            self.abort_to_recover(
                data, "both feet lost floor contact for >100ms"
            )

        swing = self._swing_side()
        if swing == "right":
            stance_contact = sample.left_contact
        elif swing == "left":
            stance_contact = sample.right_contact
        else:
            stance_contact = True
        self.stance_off_seconds = (
            0.0 if stance_contact else self.stance_off_seconds + dt
        )
        if self.stance_off_seconds > 0.050:
            self.abort_to_recover(
                data, "stance foot lost contact for >50ms"
            )

        angle_unsafe = (
            abs(sample.roll) > SAFE_ROLL_PITCH
            or abs(sample.pitch) > SAFE_ROLL_PITCH
        )
        height_unsafe = (
            sample.pelvis_pos[2] < self.standing_height - 0.100
        )
        if angle_unsafe or height_unsafe:
            reason = (
                f"unsafe pose roll={math.degrees(sample.roll):.1f}deg "
                f"pitch={math.degrees(sample.pitch):.1f}deg "
                f"pelvis_z={sample.pelvis_pos[2]:.3f}m"
            )
            if self.state == GaitState.RECOVER and (
                abs(sample.roll) > FREEZE_ROLL_PITCH
                or abs(sample.pitch) > FREEZE_ROLL_PITCH
                or sample.pelvis_pos[2] < 0.35
            ):
                self._freeze(reason)
            else:
                self.abort_to_recover(data, reason)

    def _rate_limit_targets(
        self,
        data: mujoco.MjData,
        desired: dict[str, float],
        dt: float,
    ) -> None:
        violations: list[str] = []
        for name, joint in self.joints.items():
            raw = float(desired[name])
            if raw < joint.lower - 1e-9 or raw > joint.upper + 1e-9:
                violations.append(
                    f"{name}={raw:.4f} outside "
                    f"[{joint.lower:.4f},{joint.upper:.4f}]"
                )
        if violations:
            self.target_violation_seen = True
            self.abort_to_recover(
                data, "joint target violation: " + "; ".join(violations)
            )

        for name, joint in self.joints.items():
            target = float(
                np.clip(desired[name], joint.lower, joint.upper)
            )
            if "ankle" in name:
                max_rate = 4.0
            elif any(
                token in name for token in ("hip", "knee", "shoulder")
            ):
                max_rate = 1.50
            elif name in (HEAD_JOINT, WAIST_JOINT):
                max_rate = math.radians(30.0)
            else:
                max_rate = 0.85
            self.current[name] = float(
                np.clip(
                    self.current[name]
                    + np.clip(
                        target - self.current[name],
                        -max_rate * dt,
                        max_rate * dt,
                    ),
                    joint.lower,
                    joint.upper,
                )
            )

    def step(
        self,
        data: mujoco.MjData,
        dt: float,
        *,
        head_input: float = 0.0,
        waist_input: float = 0.0,
    ) -> RobotSample:
        self._last_sample_time = float(data.time)
        sample = self.monitor.sample(data)
        self._safety_check(data, sample, dt)

        if not self.frozen:
            if self.state == GaitState.STAND and self.request is None:
                upper_commands: list[str] = []
                if head_input > 0.0:
                    upper_commands.append("head_left")
                elif head_input < 0.0:
                    upper_commands.append("head_right")
                if waist_input > 0.0:
                    upper_commands.append("waist_left")
                elif waist_input < 0.0:
                    upper_commands.append("waist_right")
                self.command_name = (
                    "+".join(upper_commands) if upper_commands else "idle"
                )
            self.head_target = float(
                np.clip(
                    self.head_target + head_input * HEAD_RATE * dt,
                    self.joints[HEAD_JOINT].lower,
                    self.joints[HEAD_JOINT].upper,
                )
            )
            self.waist_target = float(
                np.clip(
                    self.waist_target + waist_input * WAIST_RATE * dt,
                    self.joints[WAIST_JOINT].lower,
                    self.joints[WAIST_JOINT].upper,
                )
            )
            self._advance_state(data, sample, dt)

        desired = (
            dict(self.current)
            if self.frozen
            else self._state_targets(data)
        )
        if not self.frozen:
            self._add_balance_feedback(data, sample, desired, dt)
            self._rate_limit_targets(data, desired, dt)
        self._write_targets()
        self.pd.apply(data)

        post_control = self.monitor.sample(data)
        self.actuator_saturation_seen |= (
            post_control.saturated_actuators > 0
        )
        return post_control


# GLFW keypad codes 320..329 and Win32 VK_NUMPAD0..9.
GLFW_KEYPAD_TO_LOGICAL = {
    320 + number: str(number) for number in range(10)
}
WIN32_VIRTUAL_KEYS = {
    **{
        str(number): (ord(str(number)), 0x60 + number)
        for number in range(10)
    },
    "O": (ord("O"),),
    "P": (ord("P"),),
    "K": (ord("K"),),
    "L": (ord("L"),),
    "R": (ord("R"),),
    "SPACE": (0x20,),
    "ESC": (0x1B,),
}
CONTROL_KEYS = frozenset(
    {
        "0",
        "2",
        "4",
        "5",
        "6",
        "7",
        "8",
        "9",
        "O",
        "P",
        "K",
        "L",
        "R",
        "SPACE",
        "ESC",
    }
)


class ViewerKeyboard:
    """Viewer-focused held state plus reliable rising-edge taps on Windows."""

    def __init__(self) -> None:
        self._pulse_until: dict[str, float] = {}
        self._native = os.name == "nt"
        self._user32 = None
        if self._native:
            self._user32 = ctypes.WinDLL(
                "user32", use_last_error=True
            )
            self._user32.GetAsyncKeyState.argtypes = [ctypes.c_int]
            self._user32.GetAsyncKeyState.restype = ctypes.c_short
            self._user32.GetForegroundWindow.argtypes = []
            self._user32.GetForegroundWindow.restype = wintypes.HWND
            self._user32.GetWindowThreadProcessId.argtypes = [
                wintypes.HWND,
                ctypes.POINTER(wintypes.DWORD),
            ]
            self._user32.GetWindowThreadProcessId.restype = wintypes.DWORD
            self._user32.GetWindowTextLengthW.argtypes = [wintypes.HWND]
            self._user32.GetWindowTextLengthW.restype = ctypes.c_int
            self._user32.GetWindowTextW.argtypes = [
                wintypes.HWND,
                wintypes.LPWSTR,
                ctypes.c_int,
            ]
            self._user32.GetWindowTextW.restype = ctypes.c_int

    @staticmethod
    def _logical_from_keycode(keycode: int) -> str | None:
        if keycode in GLFW_KEYPAD_TO_LOGICAL:
            return GLFW_KEYPAD_TO_LOGICAL[keycode]
        if ord("0") <= keycode <= ord("9"):
            return chr(keycode)
        if keycode in (ord("o"), ord("O")):
            return "O"
        if keycode in (ord("p"), ord("P")):
            return "P"
        if keycode in (ord("k"), ord("K")):
            return "K"
        if keycode in (ord("l"), ord("L")):
            return "L"
        if keycode in (ord("r"), ord("R")):
            return "R"
        if keycode == ord(" "):
            return "SPACE"
        if keycode == 256:
            return "ESC"
        return None

    def on_key(self, keycode: int) -> None:
        logical = self._logical_from_keycode(keycode)
        if logical in CONTROL_KEYS:
            self._pulse_until[logical] = time.monotonic() + (
                0.08 if self._native else 0.70
            )

    def _foreground_is_this_process(self) -> bool:
        if not self._native or self._user32 is None:
            return False
        window = self._user32.GetForegroundWindow()
        if not window:
            return False
        length = self._user32.GetWindowTextLengthW(window)
        title = ctypes.create_unicode_buffer(max(1, length + 1))
        self._user32.GetWindowTextW(window, title, len(title))
        return "mujoco" in title.value.lower()

    def _native_down(self, logical: str) -> bool:
        if not self._native or self._user32 is None:
            return False
        return any(
            bool(self._user32.GetAsyncKeyState(vk) & 0x8000)
            for vk in WIN32_VIRTUAL_KEYS.get(logical, ())
        )

    def snapshot(self) -> set[str]:
        now = time.monotonic()
        focused = self._foreground_is_this_process()
        down: set[str] = set()
        for logical in CONTROL_KEYS:
            if (
                focused and self._native_down(logical)
            ) or now < self._pulse_until.get(logical, 0.0):
                down.add(logical)
        self._pulse_until = {
            key: until
            for key, until in self._pulse_until.items()
            if until > now
        }
        return down


class MotionCsvLogger:
    HEADER = [
        "timestamp",
        "command",
        "phase",
        "base_x",
        "base_y",
        "base_z",
        "quat_w",
        "quat_x",
        "quat_y",
        "quat_z",
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
        "left_foot_x",
        "left_foot_y",
        "left_foot_z",
        "right_foot_x",
        "right_foot_y",
        "right_foot_z",
        "left_contact",
        "right_contact",
        "left_force_n",
        "right_force_n",
        "max_ctrl",
        "saturated_actuators",
        "head_yaw_deg",
        "waist_yaw_deg",
        "fallen",
    ]

    def __init__(self, path: Path | None, *, append: bool = False) -> None:
        self.path = path
        self.file = None
        self.writer = None
        self.next_time = 0.0
        if path is not None:
            path.parent.mkdir(parents=True, exist_ok=True)
            write_header = (
                not append or not path.exists() or path.stat().st_size == 0
            )
            self.file = path.open(
                "a" if append else "w",
                newline="",
                encoding="utf-8",
            )
            self.writer = csv.writer(self.file)
            if write_header:
                self.writer.writerow(self.HEADER)

    def log(
        self,
        controller: ContactStepController,
        sample: RobotSample,
    ) -> None:
        if self.writer is None or sample.time + 1e-12 < self.next_time:
            return
        self.next_time = sample.time + 1.0 / 30.0
        fallen = (
            controller.frozen
            or abs(sample.roll) > SAFE_ROLL_PITCH
            or abs(sample.pitch) > SAFE_ROLL_PITCH
            or sample.pelvis_pos[2]
            < controller.standing_height - 0.100
        )
        self.writer.writerow(
            [
                f"{sample.time:.6f}",
                controller.command_name,
                controller.state.value,
                *[f"{value:.8f}" for value in sample.pelvis_pos],
                *[f"{value:.8f}" for value in sample.pelvis_quat],
                f"{math.degrees(sample.roll):.5f}",
                f"{math.degrees(sample.pitch):.5f}",
                f"{math.degrees(sample.yaw):.5f}",
                *[f"{value:.8f}" for value in sample.left_foot_pos],
                *[f"{value:.8f}" for value in sample.right_foot_pos],
                int(sample.left_contact),
                int(sample.right_contact),
                f"{sample.left_force:.5f}",
                f"{sample.right_force:.5f}",
                f"{sample.max_ctrl:.5f}",
                sample.saturated_actuators,
                f"{math.degrees(sample.head_yaw):.5f}",
                f"{math.degrees(sample.waist_yaw):.5f}",
                int(fallen),
            ]
        )

    def close(self) -> None:
        if self.file is not None:
            self.file.flush()
            self.file.close()


@dataclass
class ValidationResult:
    name: str
    passed: bool
    delta_x: float
    delta_y: float
    delta_yaw_deg: float
    max_roll_deg: float
    max_pitch_deg: float
    min_pelvis_height: float
    left_contact_ratio: float
    right_contact_ratio: float
    both_off_seen: bool
    saturation_seen: bool
    fallen: bool
    completed: bool
    left_foot_delta: np.ndarray
    right_foot_delta: np.ndarray
    max_left_lift: float
    max_right_lift: float
    note: str


class MetricAccumulator:
    def __init__(
        self,
        controller: ContactStepController,
        start: RobotSample,
    ) -> None:
        self.controller = controller
        self.start = start
        self.frames = 0
        self.left_contact_frames = 0
        self.right_contact_frames = 0
        self.both_off_seen = False
        self.max_roll = 0.0
        self.max_pitch = 0.0
        self.min_height = float("inf")
        self.max_left_z = start.left_foot_pos[2]
        self.max_right_z = start.right_foot_pos[2]
        self.fallen = False
        self.saturation = False
        self.final = start

    def add(self, sample: RobotSample) -> None:
        self.frames += 1
        self.left_contact_frames += int(sample.left_contact)
        self.right_contact_frames += int(sample.right_contact)
        self.both_off_seen |= (
            not sample.left_contact and not sample.right_contact
        )
        self.max_roll = max(self.max_roll, abs(sample.roll))
        self.max_pitch = max(self.max_pitch, abs(sample.pitch))
        self.min_height = min(
            self.min_height, float(sample.pelvis_pos[2])
        )
        self.max_left_z = max(
            self.max_left_z, float(sample.left_foot_pos[2])
        )
        self.max_right_z = max(
            self.max_right_z, float(sample.right_foot_pos[2])
        )
        self.saturation |= sample.saturated_actuators > 0
        self.fallen |= (
            self.controller.frozen
            or abs(sample.roll) > SAFE_ROLL_PITCH
            or abs(sample.pitch) > SAFE_ROLL_PITCH
            or sample.pelvis_pos[2]
            < self.controller.standing_height - 0.100
        )
        self.final = sample

    def result(
        self,
        name: str,
        passed: bool,
        completed: bool,
        note: str,
    ) -> ValidationResult:
        yaw_delta = _angle_delta(self.final.yaw, self.start.yaw)
        return ValidationResult(
            name=name,
            passed=passed,
            delta_x=float(
                self.final.pelvis_pos[0] - self.start.pelvis_pos[0]
            ),
            delta_y=float(
                self.final.pelvis_pos[1] - self.start.pelvis_pos[1]
            ),
            delta_yaw_deg=math.degrees(yaw_delta),
            max_roll_deg=math.degrees(self.max_roll),
            max_pitch_deg=math.degrees(self.max_pitch),
            min_pelvis_height=self.min_height,
            left_contact_ratio=self.left_contact_frames
            / max(1, self.frames),
            right_contact_ratio=self.right_contact_frames
            / max(1, self.frames),
            both_off_seen=self.both_off_seen,
            saturation_seen=self.saturation,
            fallen=self.fallen,
            completed=completed,
            left_foot_delta=(
                self.final.left_foot_pos - self.start.left_foot_pos
            ),
            right_foot_delta=(
                self.final.right_foot_pos - self.start.right_foot_pos
            ),
            max_left_lift=self.max_left_z
            - self.start.left_foot_pos[2],
            max_right_lift=self.max_right_z
            - self.start.right_foot_pos[2],
            note=note,
        )


def _new_sim() -> tuple[mujoco.MjModel, mujoco.MjData, ContactStepController]:
    model = load_model(free_base=True)
    data = mujoco.MjData(model)
    controller = ContactStepController(model, data)
    return model, data, controller


def _simulate_seconds(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    controller: ContactStepController,
    seconds: float,
    metrics: MetricAccumulator | None = None,
) -> None:
    end = data.time + seconds
    while data.time < end and not controller.frozen:
        sample = controller.step(data, float(model.opt.timestep))
        mujoco.mj_step(model, data)
        if metrics is not None:
            metrics.add(controller.monitor.sample(data))


def _run_idle_case() -> ValidationResult:
    model, data, controller = _new_sim()
    _simulate_seconds(model, data, controller, READY_SECONDS)
    start = controller.monitor.sample(data)
    metrics = MetricAccumulator(controller, start)
    _simulate_seconds(model, data, controller, 20.0, metrics)
    final = metrics.final
    passed = (
        not metrics.fallen
        and metrics.max_roll < math.radians(8.0)
        and metrics.max_pitch < math.radians(8.0)
        and metrics.min_height > controller.standing_height - 0.050
        and metrics.left_contact_frames / metrics.frames > 0.95
        and metrics.right_contact_frames / metrics.frames > 0.95
        and final.finite
    )
    return metrics.result(
        "idle",
        passed,
        completed=True,
        note="" if passed else "20s idle stability criterion failed",
    )


def _run_action_case(
    name: str,
    request: StepRequest,
    *,
    timeout: float = 14.0,
) -> ValidationResult:
    model, data, controller = _new_sim()
    _simulate_seconds(model, data, controller, READY_SECONDS)
    start = controller.monitor.sample(data)
    metrics = MetricAccumulator(controller, start)
    completed_before = controller.completed_actions
    accepted = controller.request_step(data, request)
    deadline = data.time + timeout
    while (
        accepted
        and data.time < deadline
        and not controller.frozen
        and controller.completed_actions == completed_before
    ):
        controller.step(data, float(model.opt.timestep))
        mujoco.mj_step(model, data)
        metrics.add(controller.monitor.sample(data))
    completed = (
        controller.completed_actions > completed_before
        and not controller.failure_reason
    )
    if completed:
        _simulate_seconds(model, data, controller, 2.0, metrics)

    final = metrics.final
    common = (
        accepted
        and completed
        and not metrics.fallen
        and not controller.target_violation_seen
        and final.left_contact
        and final.right_contact
        and abs(final.roll) < STABLE_ROLL_PITCH
        and abs(final.pitch) < STABLE_ROLL_PITCH
        and final.finite
    )
    dx = float(final.pelvis_pos[0] - start.pelvis_pos[0])
    dy = float(final.pelvis_pos[1] - start.pelvis_pos[1])
    dyaw = _angle_delta(final.yaw, start.yaw)
    if request.lift_only_side == "right":
        specific = (
            metrics.max_right_z - start.right_foot_pos[2] >= 0.005
            and metrics.right_contact_frames < metrics.frames
        )
    elif request.lift_only_side == "left":
        specific = (
            metrics.max_left_z - start.left_foot_pos[2] >= 0.005
            and metrics.left_contact_frames < metrics.frames
        )
    elif name == "forward":
        specific = dx >= 0.030 and abs(dy) < abs(dx)
    elif name == "backward":
        specific = dx <= -0.030
    elif name == "left":
        specific = dy >= 0.020 and abs(dyaw) < math.radians(5.0)
    elif name == "right":
        specific = dy <= -0.020 and abs(dyaw) < math.radians(5.0)
    elif name == "turn_left":
        specific = dyaw >= math.radians(5.0)
    elif name == "turn_right":
        specific = dyaw <= math.radians(-5.0)
    else:
        specific = False
    passed = common and specific
    note_parts: list[str] = []
    if not accepted:
        note_parts.append("request not accepted")
    if not completed:
        note_parts.append(
            controller.failure_reason or "state machine did not complete"
        )
    if common and not specific:
        note_parts.append("measured displacement criterion failed")
    if metrics.fallen:
        note_parts.append("fall/safety limit observed")
    return metrics.result(
        name,
        passed,
        completed,
        "; ".join(note_parts),
    )


def _print_result(result: ValidationResult) -> None:
    print(
        f"{result.name:11} {'PASS' if result.passed else 'FAIL'} | "
        f"dxy=({result.delta_x:+.4f},{result.delta_y:+.4f})m "
        f"dyaw={result.delta_yaw_deg:+.3f}deg "
        f"max_rp=({result.max_roll_deg:.2f},"
        f"{result.max_pitch_deg:.2f})deg "
        f"min_z={result.min_pelvis_height:.3f}m "
        f"contact_ratio=({result.left_contact_ratio:.3f},"
        f"{result.right_contact_ratio:.3f}) "
        f"lift=({result.max_left_lift:.4f},"
        f"{result.max_right_lift:.4f})m "
        f"both_off={result.both_off_seen} "
        f"sat={result.saturation_seen} "
        f"fallen={result.fallen} "
        f"completed={result.completed}"
        + (f" | {result.note}" if result.note else "")
    )


def run_staged_validation() -> int:
    print("=== Stage 1: idle and isolated foot lift/land ===")
    stage1 = [
        _run_idle_case(),
        _run_action_case(
            "lift_right",
            StepRequest("lift_right", lift_only_side="right"),
        ),
        _run_action_case(
            "lift_left",
            StepRequest("lift_left", lift_only_side="left"),
        ),
    ]
    for result in stage1:
        _print_result(result)
    if not all(result.passed for result in stage1):
        print("Stage 1 FAIL: stages 2-4 were not executed.")
        return 1

    print("=== Stage 2: forward and backward ===")
    stage2 = [
        _run_action_case("forward", KEY_TO_STEP["8"]),
        _run_action_case("backward", KEY_TO_STEP["2"]),
    ]
    for result in stage2:
        _print_result(result)
    if not all(result.passed for result in stage2):
        print("Stage 2 FAIL: stages 3-4 were not executed.")
        return 1

    print("=== Stage 3: lateral ===")
    stage3 = [
        _run_action_case("left", KEY_TO_STEP["4"]),
        _run_action_case("right", KEY_TO_STEP["6"]),
    ]
    for result in stage3:
        _print_result(result)
    if not all(result.passed for result in stage3):
        print("Stage 3 FAIL: stage 4 was not executed.")
        return 1

    print("=== Stage 4: turning ===")
    stage4 = [
        _run_action_case("turn_left", KEY_TO_STEP["7"]),
        _run_action_case("turn_right", KEY_TO_STEP["9"]),
    ]
    for result in stage4:
        _print_result(result)
    return 0 if all(result.passed for result in stage4) else 1


def run_viewer(
    duration: float | None,
    log_path: Path | None,
    *,
    append_log: bool,
    camera_view: str,
    warmup_ready: bool,
) -> int:
    import mujoco.viewer

    model = load_model(free_base=True)
    errors = validate_model(model)
    if errors:
        for error in errors:
            print(f"MODEL ERROR: {error}")
        return 2
    data = mujoco.MjData(model)
    controller = ContactStepController(model, data)
    if warmup_ready:
        _simulate_seconds(
            model, data, controller, READY_SECONDS
        )
    keyboard = ViewerKeyboard()
    logger = MotionCsvLogger(log_path, append=append_log)
    previous_keys: set[str] = set()
    print(HELP)
    print(f"XML: {FREE_SCENE}")
    print(f"MuJoCo timestep: {model.opt.timestep:.6f}s")
    print(f"DoF/actuators: {model.nv}/{model.nu}")
    print(f"Head yaw joint: {HEAD_JOINT}")
    print(f"Waist yaw joint: {WAIST_JOINT}")
    print("Speed mode: SAFE single-step only")
    print(
        "Continuous speed modes disabled until stable stepping is validated."
    )
    controller.monitor.print_discovery()

    try:
        with mujoco.viewer.launch_passive(
            model,
            data,
            key_callback=keyboard.on_key,
            show_left_ui=True,
            show_right_ui=True,
        ) as viewer:
            # Fixed camera: no tracking and no motion-based camera update.
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            viewer.cam.lookat[:] = (0.0, 0.0, 0.70)
            if camera_view == "high":
                viewer.cam.distance = 2.45
                viewer.cam.azimuth = 180
                viewer.cam.elevation = -55
            else:
                viewer.cam.distance = 2.30
                viewer.cam.azimuth = 145
                viewer.cam.elevation = -18
            start_wall = time.monotonic()
            exit_requested = False
            while viewer.is_running() and not exit_requested:
                frame_start = time.perf_counter()
                keys = keyboard.snapshot()
                pressed = keys - previous_keys
                previous_keys = keys

                for key in ("8", "2", "4", "6", "7", "9"):
                    if key in pressed:
                        controller.request_step(
                            data, KEY_TO_STEP[key]
                        )
                if "5" in pressed:
                    print(
                        "Continuous speed modes disabled until stable "
                        "stepping is validated."
                    )
                if "0" in pressed:
                    controller.recenter(data)
                if "SPACE" in pressed:
                    controller.abort_to_recover(
                        data, "Space stop requested"
                    )
                if "R" in pressed:
                    controller.center_upper_body()
                    print("Head and waist returning to center")
                if "ESC" in pressed:
                    exit_requested = True
                    continue

                head_input = float(("O" in keys) - ("P" in keys))
                waist_input = float(("K" in keys) - ("L" in keys))
                frame_end = data.time + 1.0 / 30.0
                while data.time < frame_end:
                    sample = controller.step(
                        data,
                        float(model.opt.timestep),
                        head_input=head_input,
                        waist_input=waist_input,
                    )
                    mujoco.mj_step(model, data)
                    logger.log(
                        controller, controller.monitor.sample(data)
                    )
                viewer.sync()

                if (
                    duration is not None
                    and time.monotonic() - start_wall >= duration
                ):
                    break
                delay = (
                    1.0 / 30.0
                    - (time.perf_counter() - frame_start)
                )
                if delay > 0:
                    time.sleep(delay)
    finally:
        logger.close()
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--validate",
        "--self-test",
        action="store_true",
        help="run stage-gated physical validation without opening Viewer",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        metavar="SECONDS",
        help="close Viewer after this many wall-clock seconds",
    )
    parser.add_argument(
        "--log",
        type=Path,
        default=None,
        metavar="CSV",
        help="write 30Hz MuJoCo state/contact log",
    )
    parser.add_argument(
        "--append-log",
        action="store_true",
        help="append to --log instead of replacing it",
    )
    parser.add_argument(
        "--camera",
        choices=("side", "high"),
        default="side",
        help="fixed Viewer validation camera",
    )
    parser.add_argument(
        "--warmup-ready",
        action="store_true",
        help="physically settle to READY_SECONDS before opening Viewer",
    )
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.duration is not None and args.duration <= 0:
        raise SystemExit("--duration must be positive")
    try:
        if args.validate:
            return run_staged_validation()
        return run_viewer(
            args.duration,
            args.log,
            append_log=args.append_log,
            camera_view=args.camera,
            warmup_ready=args.warmup_ready,
        )
    except KeyboardInterrupt:
        print("\nCtrl+C received; exiting cleanly.")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
