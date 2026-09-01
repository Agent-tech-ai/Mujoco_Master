#!/usr/bin/env python3
"""Simulation-only replay core for Phase 3A-R controller robustness work.

This module never connects to the robot and never reads reported effort.  It
does not edit MJCF or the production controller.  All safety mechanisms are
in-memory simulation-controller candidates.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import importlib.util
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
RUNS = HERE / "runs"
NUMERICAL_CONTACT_TOLERANCE_M = 0.0005
SAMPLE_RATE_HZ = 50.0
ARM_TRACKING_JOINTS = (
    "left_shoulder_roll_joint",
    "right_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "right_shoulder_yaw_joint",
    "left_wrist_yaw_joint",
    "right_wrist_yaw_joint",
    "left_wrist_roll_joint",
    "right_wrist_roll_joint",
)
BALANCE_JOINTS = (
    "left_ankle_pitch_joint",
    "right_ankle_pitch_joint",
    "left_hip_pitch_joint",
    "right_hip_pitch_joint",
    "left_knee_joint",
    "right_knee_joint",
    "waist_pitch_joint",
    "waist_roll_joint",
)


def load_phase3a():
    path = CALIBRATION / "phase3a_position_only" / "run_phase3a_experiments.py"
    spec = importlib.util.spec_from_file_location("phase3ar_frozen_phase3a", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


P3A = load_phase3a()
if str(PROJECT) not in sys.path:
    sys.path.insert(0, str(PROJECT))
from master_sim.controller import DrivenJoint  # noqa: E402
from master_sim.model import load_model, validate_model  # noqa: E402


@dataclass(frozen=True)
class Dataset:
    name: str
    reference_path: Path
    real_joint_path: Path
    motion_end_s: float


@dataclass(frozen=True)
class Design:
    experiment_id: str
    candidate_family: str
    parameter: str
    old_value: str
    new_value: str
    standing_reference_scale: float = 1.0
    left_hip_pitch_offset_scale: float = 1.0
    left_knee_offset_scale: float = 1.0
    pitch_kp: float = 140.0
    pitch_kd: float = 21.0
    roll_kp: float = 70.0
    roll_kd: float = 14.0
    pitch_clamp_nm: float | None = None
    roll_clamp_nm: float | None = None
    pitch_rate_nm_s: float | None = None
    roll_rate_nm_s: float | None = None
    ankle_pitch_weight: float = 1.0
    hip_pitch_weight: float = 0.0
    knee_pitch_weight: float = 0.0
    waist_pitch_weight: float = 0.0
    ankle_roll_weight: float = 1.0
    hip_roll_weight: float = 0.0
    waist_roll_weight: float = 0.0
    limit_guard_margin_rad: float = 0.0
    shoulder_gain_scale: float = 8.0
    wrist_gain_scale: float = 8.0
    warning: str = "SIMULATION CONTROLLER DESIGN; NOT HARDWARE CALIBRATION"


def datasets() -> dict[str, Dataset]:
    heart_lock = json.loads(
        (CALIBRATION / "phase2e_replay" / "source_data_lock.json").read_text(encoding="utf-8")
    )
    wave_lock = json.loads(
        (CALIBRATION / "phase3av_validation" / "phase3av_capture_metadata.json").read_text(encoding="utf-8")
    )
    return {
        "heart": Dataset(
            "heart",
            CALIBRATION / "phase2e_replay" / "phase2e_heart_measured_reference.csv",
            CALIBRATION / "phase2e_replay" / "phase2e_aligned_joint_data.csv",
            float(heart_lock["motion_duration_seconds"]),
        ),
        "wave": Dataset(
            "wave",
            CALIBRATION / "phase3av_validation" / "phase3av_measured_reference.csv",
            CALIBRATION / "phase3av_validation" / "phase3av_aligned_joint_data.csv",
            float(wave_lock["motion_duration_seconds"]),
        ),
    }


def standing_offsets(design: Design) -> dict[str, float]:
    candidate = json.loads(
        (CALIBRATION / "phase3a_position_only" / "simulation_controller_alignment_candidate.json").read_text(encoding="utf-8")
    )
    offsets = {str(k): float(v) * design.standing_reference_scale for k, v in candidate["standing_reference_offsets_rad"].items()}
    offsets["left_hip_pitch_joint"] *= design.left_hip_pitch_offset_scale
    offsets["left_knee_joint"] *= design.left_knee_offset_scale
    return offsets


def scaled_joints(joints: Iterable[DrivenJoint], design: Design) -> list[DrivenJoint]:
    result = []
    for joint in joints:
        scale = 1.0
        if joint.name in P3A.SHOULDER_JOINTS:
            scale = design.shoulder_gain_scale
        elif joint.name in P3A.WRIST_JOINTS:
            scale = design.wrist_gain_scale
        result.append(replace(joint, kp=joint.kp * scale, kd=joint.kd * math.sqrt(scale)))
    return result


class RobustController:
    """PD tracking with simulation-only bounded, allocated attitude feedback."""

    def __init__(self, model: mujoco.MjModel, design: Design):
        from master_sim.controller import SimulationStabilityController

        base = SimulationStabilityController(model)
        self.model = model
        self.design = design
        self.joints = scaled_joints(base.joints, design)
        self.target = base.target.copy()
        self._pelvis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        self._last_pitch = 0.0
        self._last_roll = 0.0
        self._last_time: float | None = None
        self.last_raw_pitch = 0.0
        self.last_raw_roll = 0.0
        self.last_applied_pitch = 0.0
        self.last_applied_roll = 0.0
        self.last_additions = {joint.name: 0.0 for joint in self.joints}

    @staticmethod
    def _roll_pitch(rotation: np.ndarray) -> tuple[float, float]:
        matrix = rotation.reshape(3, 3)
        roll = math.atan2(float(matrix[2, 1]), float(matrix[2, 2]))
        pitch = math.asin(float(np.clip(-matrix[2, 0], -1.0, 1.0)))
        return roll, pitch

    @staticmethod
    def _bounded(value: float, limit: float | None) -> float:
        return float(np.clip(value, -limit, limit)) if limit is not None else value

    @staticmethod
    def _rate_limited(value: float, previous: float, rate: float | None, dt: float) -> float:
        if rate is None:
            return value
        maximum = rate * dt
        return float(np.clip(value, previous - maximum, previous + maximum))

    def _limit_guard(self, joint: DrivenJoint, data: mujoco.MjData, addition: float) -> float:
        margin = self.design.limit_guard_margin_rad
        if margin <= 0.0 or addition == 0.0:
            return addition
        q = float(data.qpos[joint.qpos_adr])
        relevant = joint.upper - q if addition > 0.0 else q - joint.lower
        return addition * float(np.clip(relevant / margin, 0.0, 1.0))

    def apply(self, data: mujoco.MjData) -> None:
        design = self.design
        roll, pitch = self._roll_pitch(data.xmat[self._pelvis])
        roll_rate = float(data.qvel[3])
        pitch_rate = float(data.qvel[4])
        raw_pitch = design.pitch_kp * pitch + design.pitch_kd * pitch_rate
        raw_roll = -(design.roll_kp * roll + design.roll_kd * roll_rate)
        bounded_pitch = self._bounded(raw_pitch, design.pitch_clamp_nm)
        bounded_roll = self._bounded(raw_roll, design.roll_clamp_nm)
        dt = max(float(data.time - self._last_time), 1e-6) if self._last_time is not None else 0.001
        applied_pitch = self._rate_limited(bounded_pitch, self._last_pitch, design.pitch_rate_nm_s, dt)
        applied_roll = self._rate_limited(bounded_roll, self._last_roll, design.roll_rate_nm_s, dt)
        self._last_pitch, self._last_roll, self._last_time = applied_pitch, applied_roll, float(data.time)
        self.last_raw_pitch, self.last_raw_roll = raw_pitch, raw_roll
        self.last_applied_pitch, self.last_applied_roll = applied_pitch, applied_roll

        additions = {joint.name: 0.0 for joint in self.joints}
        for side in ("left", "right"):
            additions[f"{side}_ankle_pitch_joint"] += design.ankle_pitch_weight * applied_pitch
            additions[f"{side}_hip_pitch_joint"] += design.hip_pitch_weight * applied_pitch
            additions[f"{side}_knee_joint"] += design.knee_pitch_weight * applied_pitch
            additions[f"{side}_ankle_roll_joint"] += design.ankle_roll_weight * applied_roll
            additions[f"{side}_hip_roll_joint"] += design.hip_roll_weight * applied_roll
        additions["waist_pitch_joint"] += design.waist_pitch_weight * applied_pitch
        additions["waist_roll_joint"] += design.waist_roll_weight * applied_roll

        for joint in self.joints:
            addition = self._limit_guard(joint, data, additions[joint.name])
            additions[joint.name] = addition
            error = float(self.target[joint.qpos_adr] - data.qpos[joint.qpos_adr])
            velocity = float(data.qvel[joint.dof_adr])
            bias = float(data.qfrc_bias[joint.dof_adr])
            frictionloss = float(self.model.dof_frictionloss[joint.dof_adr])
            friction = 1.5 * frictionloss * math.tanh(error / 0.005)
            torque = joint.kp * error - joint.kd * velocity + bias + friction + addition
            if bool(self.model.actuator_ctrllimited[joint.actuator_id]):
                lower, upper = self.model.actuator_ctrlrange[joint.actuator_id]
                torque = float(np.clip(torque, lower, upper))
            data.ctrl[joint.actuator_id] = torque
        self.last_additions = additions


def load_frames(dataset: Dataset) -> tuple[pd.DataFrame, pd.DataFrame]:
    # Explicit usecols prevents reported_effort from entering the design loop.
    reference = pd.read_csv(dataset.reference_path, usecols=["t", "joint_name", "position", "velocity"])
    real = pd.read_csv(dataset.real_joint_path, usecols=["t", "joint_name", "position", "velocity"])
    return reference, real


def active_joints(frame: pd.DataFrame, mode: str) -> set[str]:
    names = set(frame.joint_name)
    if mode == "whole_body":
        return names
    if mode == "standing":
        return set()
    return {name for name in names if any(token in name for token in ("shoulder", "elbow", "wrist"))}


def geom_info(model: mujoco.MjModel) -> dict[str, int]:
    result = {}
    for geom_id in range(model.ngeom):
        body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[geom_id])) or ""
        if body == "pelvis" and int(model.geom_contype[geom_id]) != 0:
            result["pelvis"] = geom_id
        elif body == "left_hip_roll_link" and int(model.geom_contype[geom_id]) != 0:
            result["left_hip"] = geom_id
        elif body == "right_hip_roll_link" and int(model.geom_contype[geom_id]) != 0:
            result["right_hip"] = geom_id
    if set(result) != {"pelvis", "left_hip", "right_hip"}:
        raise RuntimeError(f"Cannot identify pelvis/hip collision geoms: {result}")
    return result


def pair_distance(model: mujoco.MjModel, data: mujoco.MjData, first: int, second: int) -> tuple[float, np.ndarray]:
    fromto = np.zeros(6, dtype=np.float64)
    distance = float(mujoco.mj_geomDistance(model, data, first, second, 0.20, fromto))
    return distance, fromto


def actual_pair_contacts(model: mujoco.MjModel, data: mujoco.MjData, first: int, second: int):
    result = []
    for index in range(data.ncon):
        contact = data.contact[index]
        if {int(contact.geom1), int(contact.geom2)} == {first, second}:
            result.append(contact)
    return result


def contact_counts(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[int, int, bool, bool, float]:
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    feet = {"left_ankle_roll_link", "right_ankle_roll_link"}
    self_count = nonfoot = 0
    left = right = False
    maximum = 0.0
    for index in range(data.ncon):
        contact = data.contact[index]
        g1, g2 = int(contact.geom1), int(contact.geom2)
        b1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g1])) or ""
        b2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g2])) or ""
        is_floor = g1 == floor or g2 == floor
        other = b2 if g1 == floor else b1 if g2 == floor else ""
        left |= is_floor and other == "left_ankle_roll_link"
        right |= is_floor and other == "right_ankle_roll_link"
        nonfoot += int(is_floor and other not in feet)
        self_count += int(not is_floor and b1 != b2)
        maximum = max(maximum, max(0.0, -float(contact.dist)))
    return self_count, nonfoot, left, right, maximum


def lag_seconds(reference: np.ndarray, response: np.ndarray) -> float | None:
    if len(reference) < 10 or np.std(reference) < 1e-6 or np.std(response) < 1e-6:
        return None
    best: tuple[float, int] | None = None
    maximum = int(SAMPLE_RATE_HZ)
    for shift in range(-maximum, maximum + 1):
        if shift < 0:
            x, y = reference[-shift:], response[:shift]
        elif shift > 0:
            x, y = reference[:-shift], response[shift:]
        else:
            x, y = reference, response
        if len(x) < 10 or np.std(x) < 1e-9 or np.std(y) < 1e-9:
            continue
        score = float(np.corrcoef(x, y)[0, 1])
        if best is None or score > best[0]:
            best = (score, shift)
    return None if best is None else best[1] / SAMPLE_RATE_HZ


def _metrics(dataset: Dataset, real: pd.DataFrame, joints: pd.DataFrame) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    tracking: list[dict[str, object]] = []
    balance: list[dict[str, object]] = []
    motion_mask = joints.t.between(0.0, dataset.motion_end_s)
    for name in ARM_TRACKING_JOINTS:
        sim = joints[(joints.joint_name == name) & motion_mask]
        if sim.empty:
            continue
        error = sim.position.to_numpy(float) - sim.reference_position.to_numpy(float)
        tracking.append(
            {
                "joint_name": name,
                "rmse_rad": float(np.sqrt(np.mean(error**2))),
                "mae_rad": float(np.mean(np.abs(error))),
                "lag_s": lag_seconds(sim.reference_position.to_numpy(float), sim.position.to_numpy(float)),
                "peak_error_rad": float(np.max(np.abs(error))),
                "settling_error_rad": float(abs(error[-1])),
                "real_excursion_rad": float(sim.reference_position.max() - sim.reference_position.min()),
            }
        )
    for name in BALANCE_JOINTS:
        sim = joints[joints.joint_name == name].sort_values("t")
        real_group = real[real.joint_name == name].sort_values("t")
        if sim.empty or real_group.empty:
            continue
        sim_pre = sim[sim.t.between(-3.0, -0.2)].position.mean()
        real_pre = real_group[real_group.t.between(-3.0, -0.2)].position.mean()
        sim_motion = sim[sim.t.between(0.0, dataset.motion_end_s)]
        real_motion = real_group[real_group.t.between(0.0, dataset.motion_end_s)]
        common_t = sim_motion.t.to_numpy(float)
        real_position = np.interp(common_t, real_motion.t.to_numpy(float), real_motion.position.to_numpy(float))
        sim_relative = sim_motion.position.to_numpy(float) - sim_pre
        real_relative = real_position - real_pre
        real_excursion = float(real_relative.max() - real_relative.min())
        sim_excursion = float(sim_relative.max() - sim_relative.min())
        balance.append(
            {
                "joint_name": name,
                "real_excursion_rad": real_excursion,
                "sim_excursion_rad": sim_excursion,
                "excursion_ratio": sim_excursion / real_excursion if real_excursion > 1e-9 else None,
                "relative_rmse_rad": float(np.sqrt(np.mean((sim_relative - real_relative) ** 2))),
                "lag_s": lag_seconds(real_relative, sim_relative),
            }
        )
    return tracking, balance


def run_replay(
    design: Design,
    dataset: Dataset,
    mode: str,
    *,
    pre_s: float = 5.0,
    post_s: float = 5.0,
    save_detail: bool = False,
) -> dict[str, object]:
    RUNS.mkdir(parents=True, exist_ok=True)
    reference_frame, real = load_frames(dataset)
    if mode == "standing":
        t_start, t_end = -3.0, 7.0
    else:
        t_start = max(float(reference_frame.t.min()), -pre_s)
        t_end = min(float(reference_frame.t.max()), dataset.motion_end_s + post_s)
    reference_frame = reference_frame[reference_frame.t.between(t_start, t_end)].copy()
    reference = P3A.Reference(reference_frame, "linear", 50.0)
    model = load_model(free_base=True)
    model.opt.timestep = 0.001
    errors = validate_model(model)
    if errors:
        raise RuntimeError(errors)
    controller = RobustController(model, design)
    data = mujoco.MjData(model)
    by_name = {joint.name: joint for joint in controller.joints}
    mapped = sorted(set(by_name) & set(reference.data))
    active = active_joints(reference_frame, mode) & set(mapped)
    initial = {name: reference.at(name, t_start, "position") for name in mapped}
    offsets = standing_offsets(design)
    for name in mapped:
        joint = by_name[name]
        target = initial[name] + offsets.get(name, 0.0)
        controller.target[joint.qpos_adr] = float(np.clip(target, joint.lower, joint.upper))
        data.qpos[joint.qpos_adr] = initial[name]
        data.qvel[joint.dof_adr] = 0.0
    mujoco.mj_forward(model, data)
    data.qpos[2] -= P3A.foot_surface_minimum(model, data)
    mujoco.mj_forward(model, data)

    pelvis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    geoms = geom_info(model)
    feet = {
        "left": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_ankle_roll_link"),
        "right": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_ankle_roll_link"),
    }
    initial_foot_xy = {side: data.xpos[body, :2].copy() for side, body in feet.items()}
    joint_rows: list[dict[str, object]] = []
    base_rows: list[dict[str, object]] = []
    contact_rows: list[dict[str, object]] = []
    next_sample = 0.0
    next_control = 0.0
    target_clip_count = 0
    fall_time: float | None = None
    duration = t_end - t_start
    while data.time < duration - 1e-12:
        sim_time = float(data.time)
        t = t_start + sim_time
        if sim_time + 1e-12 >= next_control:
            for name in mapped:
                joint = by_name[name]
                requested = reference.at(name, t, "position") if name in active else initial[name] + offsets.get(name, 0.0)
                clipped = float(np.clip(requested, joint.lower, joint.upper))
                target_clip_count += int(abs(clipped - requested) > 1e-10)
                controller.target[joint.qpos_adr] = clipped
            controller.apply(data)
            next_control += 0.001
        mujoco.mj_step(model, data)
        roll, pitch, yaw = P3A.rpy(data.xmat[pelvis])
        if fall_time is None and (float(data.xpos[pelvis, 2]) < 0.30 or max(abs(roll), abs(pitch)) > math.radians(45.0)):
            fall_time = float(data.time)
        if data.time + 1e-12 < next_sample:
            continue
        sim_time = float(data.time)
        t = t_start + sim_time
        self_count, nonfoot, left_contact, right_contact, max_penetration = contact_counts(model, data)
        left_distance, left_fromto = pair_distance(model, data, geoms["pelvis"], geoms["left_hip"])
        right_distance, right_fromto = pair_distance(model, data, geoms["pelvis"], geoms["right_hip"])
        left_pairs = actual_pair_contacts(model, data, geoms["pelvis"], geoms["left_hip"])
        right_pairs = actual_pair_contacts(model, data, geoms["pelvis"], geoms["right_hip"])
        base_rows.append(
            {
                "sim_time": sim_time,
                "t": t,
                "base_x": float(data.xpos[pelvis, 0]),
                "base_y": float(data.xpos[pelvis, 1]),
                "base_z": float(data.xpos[pelvis, 2]),
                "base_roll_rad": roll,
                "base_pitch_rad": pitch,
                "base_yaw_rad": yaw,
                "com_x": float(data.subtree_com[pelvis, 0]),
                "com_y": float(data.subtree_com[pelvis, 1]),
                "com_z": float(data.subtree_com[pelvis, 2]),
                "raw_pitch_feedback_nm": controller.last_raw_pitch,
                "applied_pitch_feedback_nm": controller.last_applied_pitch,
                "raw_roll_feedback_nm": controller.last_raw_roll,
                "applied_roll_feedback_nm": controller.last_applied_roll,
                "self_collision_contact_count": self_count,
                "nonfoot_ground_contact_count": nonfoot,
                "left_foot_contact": int(left_contact),
                "right_foot_contact": int(right_contact),
                "max_contact_penetration_m": max_penetration,
                "left_pelvis_hip_distance_m": left_distance,
                "right_pelvis_hip_distance_m": right_distance,
                "left_foot_slip_proxy_m": float(np.linalg.norm(data.xpos[feet["left"], :2] - initial_foot_xy["left"])),
                "right_foot_slip_proxy_m": float(np.linalg.norm(data.xpos[feet["right"], :2] - initial_foot_xy["right"])),
            }
        )
        for side, distance, fromto, contacts_for_pair in (
            ("left", left_distance, left_fromto, left_pairs),
            ("right", right_distance, right_fromto, right_pairs),
        ):
            contact = contacts_for_pair[0] if contacts_for_pair else None
            contact_rows.append(
                {
                    "sim_time": sim_time,
                    "t": t,
                    "side": side,
                    "signed_geom_distance_m": distance,
                    "segment_x1": float(fromto[0]),
                    "segment_y1": float(fromto[1]),
                    "segment_z1": float(fromto[2]),
                    "segment_x2": float(fromto[3]),
                    "segment_y2": float(fromto[4]),
                    "segment_z2": float(fromto[5]),
                    "contact_active": int(contact is not None),
                    "pair_contact_count": len(contacts_for_pair),
                    "contact_dist_m": float(contact.dist) if contact is not None else None,
                    "contact_pos_x": float(contact.pos[0]) if contact is not None else None,
                    "contact_pos_y": float(contact.pos[1]) if contact is not None else None,
                    "contact_pos_z": float(contact.pos[2]) if contact is not None else None,
                    "contact_normal_x": float(contact.frame[0]) if contact is not None else None,
                    "contact_normal_y": float(contact.frame[1]) if contact is not None else None,
                    "contact_normal_z": float(contact.frame[2]) if contact is not None else None,
                }
            )
        for name in mapped:
            joint = by_name[name]
            ctrl = float(data.ctrl[joint.actuator_id])
            limit = max(abs(float(x)) for x in model.actuator_ctrlrange[joint.actuator_id])
            joint_rows.append(
                {
                    "sim_time": sim_time,
                    "t": t,
                    "joint_name": name,
                    "input_mode": "MEASURED_REAL_TRAJECTORY" if name in active else "STANDING_TARGET",
                    "reference_position": reference.at(name, t, "position"),
                    "target_position": float(controller.target[joint.qpos_adr]),
                    "position": float(data.qpos[joint.qpos_adr]),
                    "velocity": float(data.qvel[joint.dof_adr]),
                    "tracking_error_rad": float(controller.target[joint.qpos_adr] - data.qpos[joint.qpos_adr]),
                    "balance_addition_nm": float(controller.last_additions[name]),
                    "ctrl_nm": ctrl,
                    "ctrl_saturation_fraction": abs(ctrl) / limit if limit else None,
                    "limit_margin_rad": min(float(data.qpos[joint.qpos_adr] - joint.lower), float(joint.upper - data.qpos[joint.qpos_adr])),
                }
            )
        next_sample += 1.0 / SAMPLE_RATE_HZ

    joints = pd.DataFrame(joint_rows)
    base = pd.DataFrame(base_rows)
    contacts = pd.DataFrame(contact_rows)
    tracking, balance = _metrics(dataset, real, joints) if mode != "standing" else ([], [])
    saturation = joints[joints.ctrl_saturation_fraction >= 0.98]
    pelvis_hip_over = contacts[(contacts.contact_active == 1) & (contacts.contact_dist_m < -NUMERICAL_CONTACT_TOLERANCE_M)]
    active_hip_by_time = contacts.groupby("sim_time").pair_contact_count.sum()
    other_self_samples = 0
    for row in base.itertuples(index=False):
        hip_count = int(active_hip_by_time.get(row.sim_time, 0))
        other_self_samples += int(int(row.self_collision_contact_count) > hip_count)
    summary: dict[str, object] = {
        "experiment_id": design.experiment_id,
        "dataset": dataset.name,
        "mode": mode,
        "design": asdict(design),
        "time_window_s": [t_start, t_end],
        "reported_effort_loaded": False,
        "physical_parameters_modified": False,
        "mjcf_modified": False,
        "stable_no_fall": fall_time is None,
        "fall_time_s": fall_time,
        "self_collision_samples": int((base.self_collision_contact_count > 0).sum()),
        "pelvis_hip_over_tolerance_samples": int(len(pelvis_hip_over)),
        "other_self_collision_samples": other_self_samples,
        "maximum_pelvis_hip_penetration_m": float(max(0.0, -contacts.contact_dist_m.min())) if contacts.contact_dist_m.notna().any() else 0.0,
        "minimum_left_pelvis_hip_distance_m": float(contacts[contacts.side == "left"].signed_geom_distance_m.min()),
        "minimum_right_pelvis_hip_distance_m": float(contacts[contacts.side == "right"].signed_geom_distance_m.min()),
        "nonfoot_ground_contact_samples": int((base.nonfoot_ground_contact_count > 0).sum()),
        "minimum_limit_margin_rad": float(joints.limit_margin_rad.min()),
        "target_clip_samples": int(target_clip_count),
        "persistent_saturation_fraction": float(len(saturation) / len(joints)),
        "maximum_saturation_fraction": float(joints.ctrl_saturation_fraction.max()),
        "both_feet_contact_fraction": float(((base.left_foot_contact == 1) & (base.right_foot_contact == 1)).mean()),
        "maximum_left_foot_slip_proxy_m": float(base.left_foot_slip_proxy_m.max()),
        "maximum_right_foot_slip_proxy_m": float(base.right_foot_slip_proxy_m.max()),
        "maximum_abs_tilt_deg": float(np.degrees(max(base.base_roll_rad.abs().max(), base.base_pitch_rad.abs().max()))),
        "tracking_metrics": tracking,
        "balance_metrics": balance,
    }
    summary["safety_pass"] = bool(
        summary["stable_no_fall"]
        and summary["pelvis_hip_over_tolerance_samples"] == 0
        and summary["other_self_collision_samples"] == 0
        and summary["nonfoot_ground_contact_samples"] == 0
        and summary["minimum_limit_margin_rad"] >= 0.0
        and summary["persistent_saturation_fraction"] <= 0.01
    )
    stem = f"{design.experiment_id}__{dataset.name}__{mode}"
    (RUNS / f"{stem}_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    if save_detail:
        joints.to_csv(RUNS / f"{stem}_joint_log.csv", index=False)
        base.to_csv(RUNS / f"{stem}_base_log.csv", index=False)
        contacts.to_csv(RUNS / f"{stem}_contact_log.csv", index=False)
    return summary


def run_standing(design: Design, dataset: Dataset, *, save_detail: bool = False) -> dict[str, object]:
    return run_replay(design, dataset, "standing", save_detail=save_detail)


def metric(summary: dict[str, object], category: str, joint: str, field: str) -> float | None:
    rows = summary[f"{category}_metrics"]
    assert isinstance(rows, list)
    for row in rows:
        if row["joint_name"] == joint:
            value = row.get(field)
            return None if value is None else float(value)
    return None


def aggregate_score(summaries: list[dict[str, object]]) -> dict[str, float | int | bool]:
    arm = []
    balance = []
    for summary in summaries:
        for row in summary["tracking_metrics"]:
            if float(row["real_excursion_rad"]) >= 0.02:
                arm.append(float(row["rmse_rad"]))
        for row in summary["balance_metrics"]:
            balance.append(float(row["relative_rmse_rad"]))
    return {
        "safety_pass_count": sum(bool(summary["safety_pass"]) for summary in summaries),
        "all_safe": all(bool(summary["safety_pass"]) for summary in summaries),
        "mean_arm_rmse_rad": float(np.mean(arm)) if arm else math.inf,
        "mean_balance_rmse_rad": float(np.mean(balance)) if balance else math.inf,
        "max_contact_penetration_m": max(float(summary["maximum_pelvis_hip_penetration_m"]) for summary in summaries),
        "minimum_limit_margin_rad": min(float(summary["minimum_limit_margin_rad"]) for summary in summaries),
        "maximum_saturation_fraction": max(float(summary["persistent_saturation_fraction"]) for summary in summaries),
    }
