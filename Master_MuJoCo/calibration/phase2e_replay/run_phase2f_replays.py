#!/usr/bin/env python3
"""Run offline free-base MuJoCo replays of the measured real heart trajectory."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
import pandas as pd


HERE = Path(__file__).resolve().parent
CALIBRATION = HERE.parent
PROJECT = CALIBRATION.parent
if str(PROJECT) not in sys.path:
    sys.path.insert(0, str(PROJECT))

from master_sim.controller import SimulationStabilityController
from master_sim.model import FREE_SCENE, actuated_joint_names, load_model, validate_model


REFERENCE_PATH = HERE / "phase2e_heart_measured_reference.csv"
METRICS_PATH = HERE / "phase2e_joint_metrics.csv"
PLOT_ROOT = HERE / "plots"
RATE_HZ = 50.0


def sha256(path: Path) -> str:
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    return digest


def rpy(rotation: np.ndarray) -> tuple[float, float, float]:
    matrix = rotation.reshape(3, 3)
    roll = math.atan2(float(matrix[2, 1]), float(matrix[2, 2]))
    pitch = math.asin(float(np.clip(-matrix[2, 0], -1.0, 1.0)))
    yaw = math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))
    return roll, pitch, yaw


def sensor_value(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    if sensor_id < 0:
        raise KeyError(name)
    address = int(model.sensor_adr[sensor_id])
    dimension = int(model.sensor_dim[sensor_id])
    return data.sensordata[address : address + dimension].copy()


def target_at(reference: dict[str, dict[str, np.ndarray]], joint_name: str, t: float, field: str) -> float:
    joint = reference[joint_name]
    return float(np.interp(t, joint["t"], joint[field]))


def prepare_reference() -> tuple[pd.DataFrame, dict[str, dict[str, np.ndarray]], pd.DataFrame]:
    long = pd.read_csv(REFERENCE_PATH)
    metrics = pd.read_csv(METRICS_PATH)
    reference = {}
    for joint_name, frame in long.groupby("joint_name", sort=False):
        frame = frame.sort_values("t")
        reference[joint_name] = {
            "t": frame.t.to_numpy(float),
            "position": frame.position.to_numpy(float),
            "velocity": frame.velocity.to_numpy(float),
        }
    return long, reference, metrics


def foot_surface_minimum(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    body_ids = {
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_ankle_roll_link"),
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_ankle_roll_link"),
    }
    surfaces = []
    for geom_id in range(model.ngeom):
        if int(model.geom_bodyid[geom_id]) not in body_ids or int(model.geom_contype[geom_id]) == 0:
            continue
        if int(model.geom_type[geom_id]) == int(mujoco.mjtGeom.mjGEOM_SPHERE):
            surfaces.append(float(data.geom_xpos[geom_id, 2] - model.geom_size[geom_id, 0]))
        else:
            surfaces.append(float(data.geom_xpos[geom_id, 2] - model.geom_rbound[geom_id]))
    if not surfaces:
        raise RuntimeError("No foot collision geometry found")
    return min(surfaces)


def contacts_at_sample(model: mujoco.MjModel, data: mujoco.MjData, sim_time: float, t: float) -> tuple[dict[str, object], list[dict[str, object]]]:
    floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    foot_bodies = {"left_ankle_roll_link", "right_ankle_roll_link"}
    left_contact = right_contact = False
    self_count = nonfoot_ground_count = 0
    normal_force_sum = 0.0
    rows = []
    for index in range(data.ncon):
        contact = data.contact[index]
        geom1, geom2 = int(contact.geom1), int(contact.geom2)
        body1_id, body2_id = int(model.geom_bodyid[geom1]), int(model.geom_bodyid[geom2])
        body1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body1_id) or f"body_{body1_id}"
        body2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body2_id) or f"body_{body2_id}"
        geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom1) or f"geom_{geom1}"
        geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom2) or f"geom_{geom2}"
        involves_floor = geom1 == floor_id or geom2 == floor_id
        other_body = body2 if geom1 == floor_id else body1 if geom2 == floor_id else None
        is_foot_ground = bool(involves_floor and other_body in foot_bodies)
        is_nonfoot_ground = bool(involves_floor and other_body not in foot_bodies)
        is_self = bool(not involves_floor and body1_id != body2_id)
        left_contact |= bool(is_foot_ground and other_body == "left_ankle_roll_link")
        right_contact |= bool(is_foot_ground and other_body == "right_ankle_roll_link")
        nonfoot_ground_count += int(is_nonfoot_ground)
        self_count += int(is_self)
        wrench = np.zeros(6)
        mujoco.mj_contactForce(model, data, index, wrench)
        normal_force_sum += max(0.0, float(wrench[0]))
        rows.append(
            {
                "sim_time": sim_time,
                "t": t,
                "geom1": geom1_name,
                "geom2": geom2_name,
                "body1": body1,
                "body2": body2,
                "distance": float(contact.dist),
                "normal_force": float(wrench[0]),
                "is_foot_ground": int(is_foot_ground),
                "is_nonfoot_ground": int(is_nonfoot_ground),
                "is_self_collision": int(is_self),
            }
        )
    return (
        {
            "contact_count": int(data.ncon),
            "left_foot_contact": int(left_contact),
            "right_foot_contact": int(right_contact),
            "nonfoot_ground_contact_count": nonfoot_ground_count,
            "self_collision_contact_count": self_count,
            "normal_force_sum": normal_force_sum,
        },
        rows,
    )


def run_replay(label: str, controlled_joints: set[str], reference: dict[str, dict[str, np.ndarray]]) -> dict[str, object]:
    model = load_model(free_base=True)
    model_errors = validate_model(model)
    if model_errors:
        raise RuntimeError("Invalid model: " + "; ".join(model_errors))
    data = mujoco.MjData(model)
    controller = SimulationStabilityController(model)
    controller.pose_name = "phase2e_measured_replay"
    by_name = {joint.name: joint for joint in controller.joints}
    model_joints = set(by_name)
    mapped = sorted(model_joints & set(reference))
    missing = sorted(model_joints - set(reference))
    if missing:
        raise RuntimeError(f"Missing reference joints: {missing}")

    t_start = max(values["t"].min() for values in reference.values())
    t_end = min(values["t"].max() for values in reference.values())
    duration = float(t_end - t_start)
    initial_requested = {}
    initial_clipped = {}
    for joint_name in mapped:
        joint = by_name[joint_name]
        requested = target_at(reference, joint_name, t_start, "position")
        clipped = float(np.clip(requested, joint.lower, joint.upper))
        initial_requested[joint_name] = requested
        initial_clipped[joint_name] = clipped
        controller.target[joint.qpos_adr] = clipped
        data.qpos[joint.qpos_adr] = clipped
        data.qvel[joint.dof_adr] = 0.0
    mujoco.mj_forward(model, data)
    initial_foot_min_z = foot_surface_minimum(model, data)
    base_z_adjustment = -initial_foot_min_z
    data.qpos[2] += base_z_adjustment
    mujoco.mj_forward(model, data)

    pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    foot_ids = {
        side: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_ankle_roll_link")
        for side in ("left", "right")
    }
    initial_foot_xy = {side: data.xpos[body_id, :2].copy() for side, body_id in foot_ids.items()}
    initial_base = data.xpos[pelvis_id].copy()
    sample_period = 1.0 / RATE_HZ
    next_sample = 0.0
    joint_rows: list[dict[str, object]] = []
    base_rows: list[dict[str, object]] = []
    contact_rows: list[dict[str, object]] = []
    range_request_count = 0
    range_request_joints: set[str] = set()
    fall_time = None
    max_penetration = 0.0

    while data.time < duration - 1e-12:
        t_reference = t_start + float(data.time)
        current_targets = {}
        current_requested = {}
        current_reference_velocity = {}
        current_clipped = {}
        for joint_name in mapped:
            joint = by_name[joint_name]
            reference_position = target_at(reference, joint_name, t_reference, "position")
            reference_velocity = target_at(reference, joint_name, t_reference, "velocity")
            requested = reference_position if joint_name in controlled_joints else initial_requested[joint_name]
            clipped = float(np.clip(requested, joint.lower, joint.upper))
            if abs(clipped - requested) > 1e-10:
                range_request_count += 1
                range_request_joints.add(joint_name)
            controller.target[joint.qpos_adr] = clipped
            current_targets[joint_name] = clipped
            current_requested[joint_name] = reference_position
            current_reference_velocity[joint_name] = reference_velocity
            current_clipped[joint_name] = int(abs(clipped - requested) > 1e-10)

        controller.apply(data)
        mujoco.mj_step(model, data)
        if fall_time is None:
            roll_now, pitch_now, _ = rpy(data.xmat[pelvis_id])
            if data.xpos[pelvis_id, 2] < 0.30 or max(abs(roll_now), abs(pitch_now)) > math.radians(45):
                fall_time = float(data.time)
        for index in range(data.ncon):
            max_penetration = max(max_penetration, max(0.0, -float(data.contact[index].dist)))

        if data.time + 1e-12 < next_sample:
            continue
        sim_time = float(data.time)
        t_log = t_start + sim_time
        roll, pitch, yaw = rpy(data.xmat[pelvis_id])
        contact_summary, rows = contacts_at_sample(model, data, sim_time, t_log)
        contact_rows.extend(rows)
        left_slip = float(np.linalg.norm(data.xpos[foot_ids["left"], :2] - initial_foot_xy["left"]))
        right_slip = float(np.linalg.norm(data.xpos[foot_ids["right"], :2] - initial_foot_xy["right"]))
        sensor_quat = sensor_value(model, data, "body-orientation")
        sensor_gyro = sensor_value(model, data, "body-angular-velocity")
        sensor_accel = sensor_value(model, data, "body-linear-acceleration")
        base_rows.append(
            {
                "sim_time": sim_time,
                "t": t_log,
                "base_x": float(data.xpos[pelvis_id, 0]),
                "base_y": float(data.xpos[pelvis_id, 1]),
                "base_z": float(data.xpos[pelvis_id, 2]),
                "base_roll_rad": roll,
                "base_pitch_rad": pitch,
                "base_yaw_rad": yaw,
                "imu_quat_w": float(sensor_quat[0]),
                "imu_quat_x": float(sensor_quat[1]),
                "imu_quat_y": float(sensor_quat[2]),
                "imu_quat_z": float(sensor_quat[3]),
                "imu_gyro_x": float(sensor_gyro[0]),
                "imu_gyro_y": float(sensor_gyro[1]),
                "imu_gyro_z": float(sensor_gyro[2]),
                "imu_accel_x": float(sensor_accel[0]),
                "imu_accel_y": float(sensor_accel[1]),
                "imu_accel_z": float(sensor_accel[2]),
                "left_foot_slip_proxy_m": left_slip,
                "right_foot_slip_proxy_m": right_slip,
                **contact_summary,
            }
        )
        for joint_name in mapped:
            joint = by_name[joint_name]
            position = float(data.qpos[joint.qpos_adr])
            lower_margin = position - joint.lower
            upper_margin = joint.upper - position
            force = float(data.actuator_force[joint.actuator_id])
            ctrl = float(data.ctrl[joint.actuator_id])
            ctrl_limit = max(abs(float(model.actuator_ctrlrange[joint.actuator_id, 0])), abs(float(model.actuator_ctrlrange[joint.actuator_id, 1])))
            joint_rows.append(
                {
                    "sim_time": sim_time,
                    "t": t_log,
                    "joint_name": joint_name,
                    "input_mode": "MEASURED_REAL_TRAJECTORY" if joint_name in controlled_joints else "FIXED_AT_REAL_INITIAL",
                    "coordinate_mapping_status": "IDENTITY_NAME_CANDIDATE_UNVERIFIED",
                    "reference_position": current_requested[joint_name],
                    "reference_velocity": current_reference_velocity[joint_name],
                    "target_position": current_targets[joint_name],
                    "target_clipped": current_clipped[joint_name],
                    "position": position,
                    "velocity": float(data.qvel[joint.dof_adr]),
                    "actuator_force": force,
                    "ctrl": ctrl,
                    "ctrl_saturation_fraction": abs(ctrl) / ctrl_limit if ctrl_limit > 0 else np.nan,
                    "lower_limit": joint.lower,
                    "upper_limit": joint.upper,
                    "limit_margin": min(lower_margin, upper_margin),
                    "at_or_beyond_limit": int(min(lower_margin, upper_margin) <= 1e-5),
                }
            )
        next_sample += sample_period

    joint_frame = pd.DataFrame(joint_rows)
    base_frame = pd.DataFrame(base_rows)
    contact_frame = pd.DataFrame(contact_rows)
    joint_frame.to_csv(HERE / f"{label}_joint_log.csv", index=False)
    base_frame.to_csv(HERE / f"{label}_base_log.csv", index=False)
    contact_frame.to_csv(HERE / f"{label}_contacts.csv", index=False)

    summary = {
        "label": label,
        "input_reference": "MEASURED_REAL_TRAJECTORY (not MC internal command)",
        "coordinate_mapping": "identity-by-live-name candidate; hardware sign and zero remain unverified",
        "free_base": True,
        "simulation_stability_controller_enabled": True,
        "model_validation_errors": model_errors,
        "model_timestep_seconds": float(model.opt.timestep),
        "sample_rate_hz": RATE_HZ,
        "reference_t_start": t_start,
        "reference_t_end": t_end,
        "duration_seconds": duration,
        "controlled_joints": sorted(controlled_joints),
        "fixed_at_initial_joints": sorted(model_joints - controlled_joints),
        "base_z_initialization_adjustment_m": base_z_adjustment,
        "initial_base_xyz": initial_base.tolist(),
        "range_clip_requests": range_request_count,
        "range_clip_joints": sorted(range_request_joints),
        "minimum_joint_limit_margin_rad": float(joint_frame.limit_margin.min()),
        "limit_contact_samples": int(joint_frame.at_or_beyond_limit.sum()),
        "maximum_ctrl_saturation_fraction": float(joint_frame.ctrl_saturation_fraction.max()),
        "ctrl_saturation_samples": int((joint_frame.ctrl_saturation_fraction >= 0.98).sum()),
        "maximum_abs_actuator_force": float(joint_frame.actuator_force.abs().max()),
        "max_abs_base_roll_deg": float(np.rad2deg(base_frame.base_roll_rad.abs().max())),
        "max_abs_base_pitch_deg": float(np.rad2deg(base_frame.base_pitch_rad.abs().max())),
        "base_displacement_m": float(np.linalg.norm(base_frame[["base_x", "base_y", "base_z"]].iloc[-1].to_numpy() - base_frame[["base_x", "base_y", "base_z"]].iloc[0].to_numpy())),
        "max_left_foot_slip_proxy_m": float(base_frame.left_foot_slip_proxy_m.max()),
        "max_right_foot_slip_proxy_m": float(base_frame.right_foot_slip_proxy_m.max()),
        "both_feet_contact_fraction": float(((base_frame.left_foot_contact == 1) & (base_frame.right_foot_contact == 1)).mean()),
        "nonfoot_ground_contact_samples": int((base_frame.nonfoot_ground_contact_count > 0).sum()),
        "self_collision_samples": int((base_frame.self_collision_contact_count > 0).sum()),
        "maximum_penetration_m": max_penetration,
        "fall_time_seconds": fall_time,
        "stable_no_fall": fall_time is None,
    }
    (HERE / f"{label}_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def create_mapping_assumptions(reference: dict[str, dict[str, np.ndarray]]) -> None:
    model = load_model(free_base=True)
    names = set(actuated_joint_names(model))
    rows = []
    for real_name in sorted(reference):
        if real_name in names:
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, real_name)
            lower, upper = (float(value) for value in model.jnt_range[joint_id])
            q_min = float(np.min(reference[real_name]["position"]))
            q_max = float(np.max(reference[real_name]["position"]))
            rows.append(
                {
                    "real_joint_name": real_name,
                    "mujoco_joint_name": real_name,
                    "replay_transform": "q_mujoco=q_real (candidate only)",
                    "mapping_status": "IDENTITY_NAME_CANDIDATE_UNVERIFIED",
                    "model_lower": lower,
                    "model_upper": upper,
                    "real_reference_min": q_min,
                    "real_reference_max": q_max,
                    "reference_within_model_range": bool(q_min >= lower - 1e-9 and q_max <= upper + 1e-9),
                    "evidence": "exact live JointState.name match; sign/zero not physically verified",
                }
            )
        else:
            rows.append(
                {
                    "real_joint_name": real_name,
                    "mujoco_joint_name": "",
                    "replay_transform": "NOT_MAPPED",
                    "mapping_status": "MODEL_DOF_MISMATCH",
                    "model_lower": np.nan,
                    "model_upper": np.nan,
                    "real_reference_min": float(np.min(reference[real_name]["position"])),
                    "real_reference_max": float(np.max(reference[real_name]["position"])),
                    "reference_within_model_range": False,
                    "evidence": "real joint exists; current X2 MuJoCo fixes head pitch",
                }
            )
    pd.DataFrame(rows).to_csv(HERE / "phase2f_mapping_assumptions.csv", index=False)


def plot_replay(label: str, controlled: set[str]) -> None:
    joint = pd.read_csv(HERE / f"{label}_joint_log.csv")
    base = pd.read_csv(HERE / f"{label}_base_log.csv")
    plot_dir = PLOT_ROOT / ("replay1" if label == "replay1_arm_only" else "replay2")
    plot_dir.mkdir(parents=True, exist_ok=True)
    motion_duration = json.loads((HERE / "source_data_lock.json").read_text())["motion_duration_seconds"]

    names = sorted(controlled)
    fig, axes = plt.subplots(max(1, math.ceil(len(names) / 2)), 2, figsize=(14, max(6, 2.5 * math.ceil(len(names) / 2))), squeeze=False)
    for axis, joint_name in zip(axes.flat, names):
        frame = joint[joint.joint_name == joint_name]
        axis.plot(frame.t, frame.reference_position, label="real measured", linewidth=1.3)
        axis.plot(frame.t, frame.position, label="sim", linewidth=0.9)
        axis.set_title(joint_name, fontsize=9)
        axis.grid(alpha=0.25)
    for axis in axes.flat[len(names):]:
        axis.axis("off")
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=2)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(plot_dir / "joint_tracking.png", dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(14, 11), sharex=True)
    axes[0].plot(base.t, base.base_x - base.base_x.iloc[0], label="x")
    axes[0].plot(base.t, base.base_y - base.base_y.iloc[0], label="y")
    axes[0].plot(base.t, base.base_z - base.base_z.iloc[0], label="z")
    axes[1].plot(base.t, np.rad2deg(base.base_roll_rad - base.base_roll_rad.iloc[0]), label="roll")
    axes[1].plot(base.t, np.rad2deg(base.base_pitch_rad - base.base_pitch_rad.iloc[0]), label="pitch")
    axes[2].plot(base.t, base.left_foot_slip_proxy_m, label="left slip proxy")
    axes[2].plot(base.t, base.right_foot_slip_proxy_m, label="right slip proxy")
    for axis in axes:
        axis.axvline(0, color="black", linestyle="--")
        axis.axvline(motion_duration, color="black", linestyle=":")
        axis.grid(alpha=0.25)
        axis.legend()
    axes[0].set_ylabel("base delta (m)")
    axes[1].set_ylabel("relative angle (deg)")
    axes[2].set_ylabel("m")
    axes[2].set_xlabel("t from real detected motion start (s)")
    fig.tight_layout()
    fig.savefig(plot_dir / "base_contact_stability.png", dpi=150)
    plt.close(fig)


def main() -> int:
    PLOT_ROOT.mkdir(parents=True, exist_ok=True)
    _, reference, metrics = prepare_reference()
    create_mapping_assumptions(reference)
    model = load_model(free_base=True)
    model_names = set(actuated_joint_names(model))
    replay1 = set(
        metrics[
            metrics.classification.isin(["GESTURE_PRIMARY", "GESTURE_SECONDARY"])
            & metrics.joint_name.isin(model_names)
        ].joint_name
    )
    replay2 = model_names & set(reference)
    summary1 = run_replay("replay1_arm_only", replay1, reference)
    summary2 = run_replay("replay2_whole_body", replay2, reference)
    plot_replay("replay1_arm_only", replay1)
    plot_replay("replay2_whole_body", replay2)

    provenance = {
        "mujoco_version": mujoco.__version__,
        "scene": str(FREE_SCENE.relative_to(PROJECT)),
        "scene_sha256": sha256(FREE_SCENE),
        "included_model": "assets/Master/ff_master_ultra_x2_limits.xml",
        "included_model_sha256": sha256(PROJECT / "assets" / "Master" / "ff_master_ultra_x2_limits.xml"),
        "controller": "master_sim.controller.SimulationStabilityController",
        "controller_source_sha256": sha256(PROJECT / "master_sim" / "controller.py"),
        "parameters_modified_for_replay": False,
        "dynamics_modified": False,
        "gain_modified": False,
        "friction_modified": False,
        "mapping_modified": False,
    }
    (HERE / "phase2f_replay_provenance.json").write_text(json.dumps(provenance, indent=2), encoding="utf-8")
    print(json.dumps({"replay1": summary1, "replay2": summary2}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
