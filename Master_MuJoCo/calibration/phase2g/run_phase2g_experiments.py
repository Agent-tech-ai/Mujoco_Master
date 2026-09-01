#!/usr/bin/env python3
"""Run Phase 2G simulation-only controller alignment experiments.

These experiments never connect to the robot and never modify MJCF physical
parameters.  Each candidate changes one controller/reference category only.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import sys

import mujoco
import numpy as np
import pandas as pd


HERE = Path(__file__).resolve().parent
CALIBRATION = HERE.parent
PROJECT = CALIBRATION.parent
PHASE2E = CALIBRATION / "phase2e_replay"
if str(PROJECT) not in sys.path:
    sys.path.insert(0, str(PROJECT))

from master_sim.controller import SimulationStabilityConfig, SimulationStabilityController
from master_sim.model import load_model, validate_model


REFERENCE_PATH = PHASE2E / "phase2e_heart_measured_reference.csv"
METRICS_PATH = PHASE2E / "phase2e_joint_metrics.csv"
BASELINE_LOG = PHASE2E / "replay1_arm_only_joint_log.csv"
RATE_HZ = 50.0


@dataclass(frozen=True)
class Experiment:
    name: str
    free_base: bool
    interpolation: str = "linear"
    reference_advance_s: float = 0.0
    balance_gain_scale: float = 1.0
    equilibrium_compensation: bool = False
    classification: str = "DIAGNOSTIC_BASELINE"


EXPERIMENTS = (
    Experiment("fixed_base_baseline", False),
    Experiment("fixed_base_50hz_zoh", False, interpolation="zoh_50hz"),
    Experiment(
        "free_reference_advance_030",
        True,
        reference_advance_s=0.30,
        classification="SIM_CONTROLLER_ALIGNMENT_CANDIDATE",
    ),
    Experiment(
        "free_balance_gain_scale_060",
        True,
        balance_gain_scale=0.60,
        classification="SIM_CONTROLLER_ALIGNMENT_CANDIDATE",
    ),
    Experiment(
        "free_equilibrium_target_compensation",
        True,
        equilibrium_compensation=True,
        classification="SIM_CONTROLLER_ALIGNMENT_CANDIDATE",
    ),
)


def rpy(rotation: np.ndarray) -> tuple[float, float, float]:
    matrix = rotation.reshape(3, 3)
    return (
        math.atan2(float(matrix[2, 1]), float(matrix[2, 2])),
        math.asin(float(np.clip(-matrix[2, 0], -1.0, 1.0))),
        math.atan2(float(matrix[1, 0]), float(matrix[0, 0])),
    )


def sensor_value(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    address = int(model.sensor_adr[sensor_id])
    dimension = int(model.sensor_dim[sensor_id])
    return data.sensordata[address : address + dimension].copy()


def prepare_reference() -> tuple[dict[str, dict[str, np.ndarray]], set[str]]:
    long = pd.read_csv(REFERENCE_PATH)
    metrics = pd.read_csv(METRICS_PATH)
    reference: dict[str, dict[str, np.ndarray]] = {}
    for joint_name, frame in long.groupby("joint_name", sort=False):
        frame = frame.sort_values("t")
        reference[joint_name] = {
            "t": frame.t.to_numpy(float),
            "position": frame.position.to_numpy(float),
            "velocity": frame.velocity.to_numpy(float),
        }
    controlled = set(
        metrics.loc[
            metrics.classification.isin(["GESTURE_PRIMARY", "GESTURE_SECONDARY"]),
            "joint_name",
        ]
    )
    return reference, controlled


def at(reference: dict[str, dict[str, np.ndarray]], name: str, t: float, field: str) -> float:
    values = reference[name]
    return float(np.interp(t, values["t"], values[field]))


def equilibrium_offsets() -> dict[str, float]:
    """One-shot target corrections derived only from the baseline simulation."""
    frame = pd.read_csv(BASELINE_LOG)
    pre = frame[(frame.t >= -3.0) & (frame.t <= -0.2)]
    offsets: dict[str, float] = {}
    for name, group in pre.groupby("joint_name"):
        if not any(token in name for token in ("hip", "knee", "ankle", "waist")):
            continue
        offsets[name] = float(group.reference_position.mean() - group.position.mean())
    return offsets


def foot_surface_minimum(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    feet = {
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_ankle_roll_link"),
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_ankle_roll_link"),
    }
    surfaces: list[float] = []
    for geom_id in range(model.ngeom):
        if int(model.geom_bodyid[geom_id]) not in feet or int(model.geom_contype[geom_id]) == 0:
            continue
        if int(model.geom_type[geom_id]) == int(mujoco.mjtGeom.mjGEOM_SPHERE):
            surfaces.append(float(data.geom_xpos[geom_id, 2] - model.geom_size[geom_id, 0]))
        else:
            surfaces.append(float(data.geom_xpos[geom_id, 2] - model.geom_rbound[geom_id]))
    return min(surfaces)


def contact_summary(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float | int]:
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    feet = {"left_ankle_roll_link", "right_ankle_roll_link"}
    left = right = False
    self_count = nonfoot = 0
    max_penetration = 0.0
    for index in range(data.ncon):
        contact = data.contact[index]
        g1, g2 = int(contact.geom1), int(contact.geom2)
        b1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g1])) or ""
        b2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g2])) or ""
        is_floor = g1 == floor or g2 == floor
        other = b2 if g1 == floor else b1 if g2 == floor else ""
        left |= bool(is_floor and other == "left_ankle_roll_link")
        right |= bool(is_floor and other == "right_ankle_roll_link")
        nonfoot += int(is_floor and other not in feet)
        self_count += int(not is_floor and b1 != b2)
        max_penetration = max(max_penetration, max(0.0, -float(contact.dist)))
    return {
        "contact_count": int(data.ncon),
        "left_foot_contact": int(left),
        "right_foot_contact": int(right),
        "nonfoot_ground_contact_count": nonfoot,
        "self_collision_contact_count": self_count,
        "max_contact_penetration_m": max_penetration,
    }


def run(experiment: Experiment, reference: dict[str, dict[str, np.ndarray]], controlled: set[str]) -> dict[str, object]:
    model = load_model(free_base=experiment.free_base)
    errors = validate_model(model)
    if errors:
        raise RuntimeError("Invalid model: " + "; ".join(errors))
    config = SimulationStabilityConfig(
        pitch_kp=200.0 * experiment.balance_gain_scale,
        pitch_kd=30.0 * experiment.balance_gain_scale,
        roll_kp=100.0 * experiment.balance_gain_scale,
        roll_kd=20.0 * experiment.balance_gain_scale,
        friction_compensation_scale=1.5,
        friction_error_width=0.005,
    )
    controller = SimulationStabilityController(model, config)
    controller.pose_name = "phase2g_measured_replay"
    data = mujoco.MjData(model)
    by_name = {joint.name: joint for joint in controller.joints}
    mapped = sorted(set(by_name) & set(reference))
    controlled = controlled & set(mapped)
    t_start = max(reference[name]["t"].min() for name in mapped)
    t_end = min(reference[name]["t"].max() for name in mapped)
    duration = float(t_end - t_start)
    initial = {name: at(reference, name, t_start, "position") for name in mapped}
    offsets = equilibrium_offsets() if experiment.equilibrium_compensation else {}

    for name in mapped:
        joint = by_name[name]
        target = float(np.clip(initial[name] + offsets.get(name, 0.0), joint.lower, joint.upper))
        controller.target[joint.qpos_adr] = target
        data.qpos[joint.qpos_adr] = initial[name]
        data.qvel[joint.dof_adr] = 0.0
    mujoco.mj_forward(model, data)
    base_z_adjustment = 0.0
    if experiment.free_base:
        base_z_adjustment = -foot_surface_minimum(model, data)
        data.qpos[2] += base_z_adjustment
        mujoco.mj_forward(model, data)

    pelvis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    joint_rows: list[dict[str, object]] = []
    base_rows: list[dict[str, object]] = []
    next_sample = 0.0
    fall_time: float | None = None
    target_clip_count = 0

    while data.time < duration - 1e-12:
        t_reference = t_start + float(data.time)
        sample_t = t_reference + experiment.reference_advance_s
        if experiment.interpolation == "zoh_50hz":
            sample_t = t_start + math.floor((sample_t - t_start) * RATE_HZ + 1e-12) / RATE_HZ
        current: dict[str, tuple[float, float, float, int]] = {}
        for name in mapped:
            joint = by_name[name]
            ref_position = at(reference, name, sample_t, "position")
            ref_velocity = at(reference, name, sample_t, "velocity")
            requested = ref_position if name in controlled else initial[name] + offsets.get(name, 0.0)
            clipped = float(np.clip(requested, joint.lower, joint.upper))
            was_clipped = int(abs(clipped - requested) > 1e-10)
            target_clip_count += was_clipped
            controller.target[joint.qpos_adr] = clipped
            current[name] = (ref_position, ref_velocity, clipped, was_clipped)

        controller.apply(data)
        mujoco.mj_step(model, data)
        roll, pitch, _ = rpy(data.xmat[pelvis])
        if fall_time is None and (
            float(data.xpos[pelvis, 2]) < 0.30
            or max(abs(roll), abs(pitch)) > math.radians(45.0)
        ):
            fall_time = float(data.time)
        if data.time + 1e-12 < next_sample:
            continue

        sim_time = float(data.time)
        t_log = t_start + sim_time
        roll, pitch, yaw = rpy(data.xmat[pelvis])
        quat = sensor_value(model, data, "body-orientation")
        gyro = sensor_value(model, data, "body-angular-velocity")
        accel = sensor_value(model, data, "body-linear-acceleration")
        base_rows.append(
            {
                "sim_time": sim_time,
                "t": t_log,
                "base_x": float(data.xpos[pelvis, 0]),
                "base_y": float(data.xpos[pelvis, 1]),
                "base_z": float(data.xpos[pelvis, 2]),
                "base_roll_rad": roll,
                "base_pitch_rad": pitch,
                "base_yaw_rad": yaw,
                "imu_quat_w": float(quat[0]),
                "imu_quat_x": float(quat[1]),
                "imu_quat_y": float(quat[2]),
                "imu_quat_z": float(quat[3]),
                "imu_gyro_x": float(gyro[0]),
                "imu_gyro_y": float(gyro[1]),
                "imu_gyro_z": float(gyro[2]),
                "imu_accel_x": float(accel[0]),
                "imu_accel_y": float(accel[1]),
                "imu_accel_z": float(accel[2]),
                **contact_summary(model, data),
            }
        )
        for name in mapped:
            joint = by_name[name]
            ref_position, ref_velocity, target, clipped = current[name]
            position = float(data.qpos[joint.qpos_adr])
            ctrl = float(data.ctrl[joint.actuator_id])
            ctrl_limit = max(abs(float(x)) for x in model.actuator_ctrlrange[joint.actuator_id])
            joint_rows.append(
                {
                    "sim_time": sim_time,
                    "t": t_log,
                    "joint_name": name,
                    "input_mode": "MEASURED_REAL_TRAJECTORY" if name in controlled else "STANDING_TARGET",
                    "reference_position": ref_position,
                    "reference_velocity": ref_velocity,
                    "target_position": target,
                    "target_clipped": clipped,
                    "position": position,
                    "velocity": float(data.qvel[joint.dof_adr]),
                    "actuator_force": float(data.actuator_force[joint.actuator_id]),
                    "ctrl": ctrl,
                    "ctrl_saturation_fraction": abs(ctrl) / ctrl_limit if ctrl_limit else np.nan,
                    "lower_limit": joint.lower,
                    "upper_limit": joint.upper,
                    "limit_margin": min(position - joint.lower, joint.upper - position),
                }
            )
        next_sample += 1.0 / RATE_HZ

    joints = pd.DataFrame(joint_rows)
    base = pd.DataFrame(base_rows)
    joints.to_csv(HERE / f"{experiment.name}_joint_log.csv", index=False)
    base.to_csv(HERE / f"{experiment.name}_base_log.csv", index=False)
    summary: dict[str, object] = {
        **asdict(experiment),
        "warning": "NOT HARDWARE CALIBRATION",
        "model_physical_parameters_modified": False,
        "controller_source_modified": False,
        "model_timestep_seconds": float(model.opt.timestep),
        "sample_rate_hz": RATE_HZ,
        "duration_seconds": duration,
        "controlled_joints": sorted(controlled),
        "standing_target_offsets_rad": offsets,
        "base_z_initialization_adjustment_m": base_z_adjustment,
        "target_clip_samples": int(target_clip_count),
        "maximum_ctrl_saturation_fraction": float(joints.ctrl_saturation_fraction.max()),
        "minimum_limit_margin_rad": float(joints.limit_margin.min()),
        "stable_no_fall": fall_time is None,
        "fall_time_seconds": fall_time,
        "max_abs_base_roll_deg": float(np.rad2deg(base.base_roll_rad.abs().max())),
        "max_abs_base_pitch_deg": float(np.rad2deg(base.base_pitch_rad.abs().max())),
        "both_feet_contact_fraction": float(((base.left_foot_contact == 1) & (base.right_foot_contact == 1)).mean()),
        "nonfoot_ground_contact_samples": int((base.nonfoot_ground_contact_count > 0).sum()),
        "self_collision_samples": int((base.self_collision_contact_count > 0).sum()),
        "maximum_contact_penetration_m": float(base.max_contact_penetration_m.max()),
    }
    (HERE / f"{experiment.name}_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def main() -> int:
    reference, controlled = prepare_reference()
    summaries = [run(experiment, reference, controlled) for experiment in EXPERIMENTS]
    (HERE / "experiment_run_summary.json").write_text(json.dumps(summaries, indent=2), encoding="utf-8")
    print(json.dumps([{k: s[k] for k in ("name", "stable_no_fall", "maximum_ctrl_saturation_fraction")} for s in summaries], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
