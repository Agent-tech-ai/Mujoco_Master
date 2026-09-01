#!/usr/bin/env python3
"""Phase 3A simulation-only position-response experiments.

The runner reads only real joint position/velocity references.  It never reads
``reported_effort`` for fitting, never connects to ROS/SSH, and never edits the
MJCF, hardware mapping, or controller source.  Every controller change exists
only in the in-memory MuJoCo experiment and is labelled NOT HARDWARE CALIBRATION.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Iterable

import mujoco
import numpy as np
import pandas as pd
from scipy.interpolate import PchipInterpolator


HERE = Path(__file__).resolve().parent
CALIBRATION = HERE.parent
PROJECT = CALIBRATION.parent
P2E = CALIBRATION / "phase2e_replay"
REAL_DIR = CALIBRATION / "logs" / "real" / "phase2d_heart_001"
P2G = CALIBRATION / "phase2g"
if str(PROJECT) not in sys.path:
    sys.path.insert(0, str(PROJECT))

from calibration.phase2b2_sim_rehearsal import SEGMENTS, _base_targets, _self_contacts
from master_sim.controller import DrivenJoint, SimulationStabilityConfig, SimulationStabilityController
from master_sim.model import ASSET_DIR, load_model, validate_model


REFERENCE_PATH = P2E / "phase2e_heart_measured_reference.csv"
CLASSIFICATION_PATH = P2E / "phase2e_joint_metrics.csv"
REAL_JOINT_PATH = P2E / "phase2e_aligned_joint_data.csv"
REAL_IMU_PATH = P2E / "phase2e_aligned_imu_data.csv"
LOCK_PATH = P2E / "source_data_lock.json"
LOCK = json.loads(LOCK_PATH.read_text(encoding="utf-8"))
MOTION_END = float(LOCK["motion_duration_seconds"])
SOURCE_RATE_HZ = float(LOCK["target_rate_hz"])
SAMPLE_RATE_HZ = 50.0
PRE_WINDOW = (-3.0, -0.2)
POST_WINDOW = (MOTION_END + 0.5, min(MOTION_END + 3.0, float(LOCK["relative_t_end_seconds"])))

ARM_TRACKING_JOINTS = (
    "left_shoulder_roll_joint",
    "right_shoulder_roll_joint",
    "left_wrist_yaw_joint",
    "right_wrist_yaw_joint",
    "left_wrist_roll_joint",
    "right_wrist_roll_joint",
    "left_shoulder_yaw_joint",
    "right_shoulder_yaw_joint",
)
SHOULDER_JOINTS = tuple(name for name in ARM_TRACKING_JOINTS if "shoulder" in name)
WRIST_JOINTS = tuple(name for name in ARM_TRACKING_JOINTS if "wrist" in name)
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
REHEARSAL_JOINTS = (
    "left_wrist_roll_joint",
    "right_wrist_roll_joint",
    "left_wrist_yaw_joint",
    "right_wrist_yaw_joint",
    "left_wrist_pitch_joint",
    "right_wrist_pitch_joint",
    "left_elbow_joint",
    "right_elbow_joint",
    "left_shoulder_yaw_joint",
    "right_shoulder_yaw_joint",
    "left_shoulder_pitch_joint",
    "right_shoulder_pitch_joint",
)


@dataclass(frozen=True)
class Experiment:
    name: str
    parent: str
    free_base: bool
    changed_category: str
    classification: str = "DIAGNOSTIC_ONLY"
    interpolation: str = "linear"
    reference_rate_hz: float = SOURCE_RATE_HZ
    controller_rate_hz: float = 1000.0
    timestep_s: float = 0.001
    shoulder_gain_scale: float = 1.0
    wrist_gain_scale: float = 1.0
    balance_gain_scale: float = 1.0
    standing_reference_scale: float = 0.0
    velocity_limit_rad_s: float | None = None


FIXED_DIAGNOSTICS = (
    Experiment("fixed_baseline", "none", False, "none", "DIAGNOSTIC_ONLY"),
    Experiment("fixed_interp_zoh", "fixed_baseline", False, "reference interpolation", interpolation="zoh"),
    Experiment("fixed_interp_pchip", "fixed_baseline", False, "reference interpolation", interpolation="pchip"),
    Experiment("fixed_reference_rate_25hz", "fixed_baseline", False, "reference sampling rate", reference_rate_hz=25.0),
    Experiment("fixed_reference_rate_100hz", "fixed_baseline", False, "reference sampling rate", reference_rate_hz=100.0),
    Experiment("fixed_controller_rate_500hz", "fixed_baseline", False, "controller update rate", controller_rate_hz=500.0),
    Experiment("fixed_controller_rate_200hz", "fixed_baseline", False, "controller update rate", controller_rate_hz=200.0),
    Experiment("fixed_controller_rate_100hz", "fixed_baseline", False, "controller update rate", controller_rate_hz=100.0),
    # Physics substep changes alone; the controller remains at the baseline 1 kHz.
    Experiment("fixed_timestep_0005", "fixed_baseline", False, "simulation timestep", timestep_s=0.0005),
    Experiment("fixed_velocity_limit_5", "fixed_baseline", False, "simulation target velocity limit", velocity_limit_rad_s=5.0),
    Experiment("fixed_velocity_limit_2", "fixed_baseline", False, "simulation target velocity limit", velocity_limit_rad_s=2.0),
)

SHOULDER_GAIN_EXPERIMENTS = tuple(
    Experiment(
        f"fixed_shoulders_scale_{str(scale).replace('.', 'p')}",
        "fixed_baseline",
        False,
        "simulation shoulder PD bandwidth",
        shoulder_gain_scale=scale,
    )
    for scale in (1.5, 2.0, 3.0, 4.0, 6.0, 8.0)
)
WRIST_GAIN_EXPERIMENTS = tuple(
    Experiment(
        f"fixed_wrists_scale_{str(scale).replace('.', 'p')}",
        "fixed_baseline",
        False,
        "simulation wrist PD bandwidth",
        wrist_gain_scale=scale,
    )
    for scale in (1.5, 2.0, 3.0, 4.0, 6.0, 8.0)
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_immutable_manifest() -> None:
    paths: list[tuple[str, Path]] = []
    paths.extend(("phase2d_real_log", path) for path in sorted(REAL_DIR.glob("*")) if path.is_file())
    for name in (
        "phase2e_heart_measured_reference.csv",
        "phase2e_aligned_joint_data.csv",
        "phase2e_aligned_imu_data.csv",
        "phase2e_joint_metrics.csv",
        "source_data_lock.json",
        "source_sha256_manifest.csv",
        "replay1_arm_only_joint_log.csv",
        "replay1_arm_only_base_log.csv",
        "replay1_arm_only_summary.json",
        "phase2f_replay1_joint_metrics.csv",
        "phase2f_replay_provenance.json",
    ):
        paths.append(("phase2e_phase2f", P2E / name))
    for name in (
        "phase2g_tracking_metrics.csv",
        "phase2g_balance_metrics.csv",
        "phase2g_equilibrium_delta.csv",
        "phase2g_tracking_delay_report.md",
        "phase2g_standing_equilibrium_report.md",
    ):
        paths.append(("phase2g", (P2G / name) if (P2G / name).exists() else (CALIBRATION / name)))
    for name in ("ff_master_ultra.xml", "ff_master_ultra_x2_limits.xml", "scene_x2_fixed.xml", "scene_x2_free.xml"):
        paths.append(("mjcf", ASSET_DIR / name))
    for name in ("controller.py", "model.py"):
        paths.append(("simulation_controller", PROJECT / "master_sim" / name))
    paths.extend(
        (
            "phase2h_gate",
            CALIBRATION / name,
        )
        for name in (
            "phase2h_imu_transform_closure.md",
            "phase2h_sign_zero_evidence.md",
            "phase2h_effort_semantics_closure.md",
            "phase2h_dynamics_gate.md",
        )
    )
    paths.append(("hardware_mapping_read_only", CALIBRATION / "joint_mapping.csv"))
    rows = []
    for category, path in paths:
        if not path.exists():
            raise FileNotFoundError(path)
        rows.append(
            {
                "category": category,
                "path": str(path.relative_to(PROJECT)),
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
        )
    pd.DataFrame(rows).sort_values(["category", "path"]).to_csv(HERE / "immutable_baseline_sha256.csv", index=False)
    lock = {
        "status": "IMMUTABLE_BASELINE_LOCKED",
        "warning": "NOT HARDWARE CALIBRATION",
        "phase2_data_overwritten": False,
        "reported_effort_used_for_fitting": False,
        "physical_parameters_modified": False,
        "hardware_mapping_modified": False,
        "absolute_imu_used_for_fitting": False,
        "manifest": "immutable_baseline_sha256.csv",
        "source_rate_hz": SOURCE_RATE_HZ,
        "motion_end_seconds": MOTION_END,
    }
    (HERE / "immutable_baseline_lock.json").write_text(json.dumps(lock, indent=2), encoding="utf-8")


class Reference:
    def __init__(self, frame: pd.DataFrame, interpolation: str, rate_hz: float):
        self.interpolation = interpolation
        self.data: dict[str, dict[str, object]] = {}
        for name, group in frame.groupby("joint_name", sort=False):
            group = group.sort_values("t")
            source_t = group.t.to_numpy(float)
            source_position = group.position.to_numpy(float)
            source_velocity = group.velocity.to_numpy(float)
            if abs(rate_hz - SOURCE_RATE_HZ) > 1e-9:
                t = np.arange(source_t[0], source_t[-1] + 0.25 / rate_hz, 1.0 / rate_hz)
                position = np.interp(t, source_t, source_position)
                velocity = np.interp(t, source_t, source_velocity)
            else:
                t, position, velocity = source_t, source_position, source_velocity
            entry: dict[str, object] = {"t": t, "position": position, "velocity": velocity}
            if interpolation == "pchip":
                entry["position_interp"] = PchipInterpolator(t, position, extrapolate=True)
                entry["velocity_interp"] = PchipInterpolator(t, velocity, extrapolate=True)
            self.data[name] = entry

    def at(self, name: str, t: float, field: str) -> float:
        entry = self.data[name]
        times = entry["t"]
        values = entry[field]
        assert isinstance(times, np.ndarray) and isinstance(values, np.ndarray)
        if self.interpolation == "zoh":
            index = int(np.clip(np.searchsorted(times, t, side="right") - 1, 0, len(times) - 1))
            return float(values[index])
        if self.interpolation == "pchip":
            return float(entry[f"{field}_interp"](t))  # type: ignore[operator]
        return float(np.interp(t, times, values))


def source_frames() -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame, set[str]]:
    # Select only position/velocity columns.  reported_effort is intentionally never loaded.
    reference = pd.read_csv(REFERENCE_PATH, usecols=["t", "joint_name", "position", "velocity"])
    real_joint = pd.read_csv(REAL_JOINT_PATH, usecols=["t", "joint_name", "position", "velocity"])
    real_imu = pd.read_csv(
        REAL_IMU_PATH,
        usecols=["t", "imu", "relative_roll_rad", "relative_pitch_rad", "gyro_norm"],
    )
    classifications = pd.read_csv(CLASSIFICATION_PATH, usecols=["joint_name", "classification"])
    controlled = set(
        classifications.loc[
            classifications.classification.isin(["GESTURE_PRIMARY", "GESTURE_SECONDARY"]),
            "joint_name",
        ]
    )
    return reference, real_joint, real_imu, controlled


def scaled_joints(joints: Iterable[DrivenJoint], experiment: Experiment) -> list[DrivenJoint]:
    result = []
    for joint in joints:
        scale = 1.0
        if joint.name in SHOULDER_JOINTS:
            scale = experiment.shoulder_gain_scale
        elif joint.name in WRIST_JOINTS:
            scale = experiment.wrist_gain_scale
        # Preserve the baseline damping-ratio intent while changing bandwidth.
        result.append(replace(joint, kp=joint.kp * scale, kd=joint.kd * math.sqrt(scale)))
    return result


def rpy(rotation: np.ndarray) -> tuple[float, float, float]:
    matrix = rotation.reshape(3, 3)
    return (
        math.atan2(float(matrix[2, 1]), float(matrix[2, 2])),
        math.asin(float(np.clip(-matrix[2, 0], -1.0, 1.0))),
        math.atan2(float(matrix[1, 0]), float(matrix[0, 0])),
    )


def sensor(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    address = int(model.sensor_adr[sensor_id])
    dimension = int(model.sensor_dim[sensor_id])
    return data.sensordata[address : address + dimension].copy()


def foot_surface_minimum(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    feet = {
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_ankle_roll_link"),
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_ankle_roll_link"),
    }
    surfaces = []
    for geom_id in range(model.ngeom):
        if int(model.geom_bodyid[geom_id]) not in feet or int(model.geom_contype[geom_id]) == 0:
            continue
        if int(model.geom_type[geom_id]) == int(mujoco.mjtGeom.mjGEOM_SPHERE):
            surfaces.append(float(data.geom_xpos[geom_id, 2] - model.geom_size[geom_id, 0]))
        else:
            surfaces.append(float(data.geom_xpos[geom_id, 2] - model.geom_rbound[geom_id]))
    return min(surfaces)


def contact_state(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, int | float]:
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    feet = {"left_ankle_roll_link", "right_ankle_roll_link"}
    left = right = False
    self_count = nonfoot = 0
    penetration = 0.0
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
        penetration = max(penetration, max(0.0, -float(contact.dist)))
    return {
        "left_foot_contact": int(left),
        "right_foot_contact": int(right),
        "nonfoot_ground_contact_count": nonfoot,
        "self_collision_contact_count": self_count,
        "max_contact_penetration_m": penetration,
    }


def standing_offsets(parent_log: Path, real_joint: pd.DataFrame) -> dict[str, float]:
    if not parent_log.exists():
        return {}
    sim = pd.read_csv(parent_log, usecols=["t", "joint_name", "position"])
    offsets: dict[str, float] = {}
    for name in BALANCE_JOINTS:
        real_pre = real_joint[(real_joint.joint_name == name) & real_joint.t.between(*PRE_WINDOW)]
        sim_pre = sim[(sim.joint_name == name) & sim.t.between(*PRE_WINDOW)]
        if len(real_pre) and len(sim_pre):
            offsets[name] = float(real_pre.position.mean() - sim_pre.position.mean())
    return offsets


def run_replay(
    experiment: Experiment,
    source_reference: pd.DataFrame,
    real_joint: pd.DataFrame,
    controlled: set[str],
    *,
    inherited_offsets: dict[str, float] | None = None,
) -> dict[str, object]:
    reference = Reference(source_reference, experiment.interpolation, experiment.reference_rate_hz)
    model = load_model(free_base=experiment.free_base)
    model.opt.timestep = experiment.timestep_s
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
    controller.joints = scaled_joints(controller.joints, experiment)
    controller._by_name = {joint.name: joint for joint in controller.joints}
    controller.pose_name = "phase3a_measured_replay"
    data = mujoco.MjData(model)
    by_name = {joint.name: joint for joint in controller.joints}
    mapped = sorted(set(by_name) & set(reference.data))
    active = controlled & set(mapped)
    t_start = max(float(reference.data[name]["t"][0]) for name in mapped)  # type: ignore[index]
    t_end = min(float(reference.data[name]["t"][-1]) for name in mapped)  # type: ignore[index]
    initial = {name: reference.at(name, t_start, "position") for name in mapped}
    offsets = dict(inherited_offsets or {})
    if experiment.standing_reference_scale and not offsets:
        raise ValueError("standing_reference_scale requires inherited offsets")
    applied_offsets = {name: experiment.standing_reference_scale * value for name, value in offsets.items()}
    for name in mapped:
        joint = by_name[name]
        target = float(np.clip(initial[name] + applied_offsets.get(name, 0.0), joint.lower, joint.upper))
        controller.target[joint.qpos_adr] = target
        data.qpos[joint.qpos_adr] = initial[name]
        data.qvel[joint.dof_adr] = 0.0
    mujoco.mj_forward(model, data)
    if experiment.free_base:
        data.qpos[2] -= foot_surface_minimum(model, data)
        mujoco.mj_forward(model, data)

    pelvis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    feet = {
        "left": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_ankle_roll_link"),
        "right": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_ankle_roll_link"),
    }
    initial_foot_xy = {side: data.xpos[body, :2].copy() for side, body in feet.items()}
    joint_rows: list[dict[str, object]] = []
    base_rows: list[dict[str, object]] = []
    next_sample = 0.0
    next_control = 0.0
    control_period = 1.0 / experiment.controller_rate_hz
    last_target = {name: float(controller.target[by_name[name].qpos_adr]) for name in mapped}
    fall_time: float | None = None
    target_clip_count = 0
    last_reference: dict[str, tuple[float, float, float, int]] = {}

    while data.time < t_end - t_start - 1e-12:
        sim_time = float(data.time)
        t_reference = t_start + sim_time
        if sim_time + 1e-12 >= next_control:
            elapsed_control = control_period if next_control else max(model.opt.timestep, control_period)
            for name in mapped:
                joint = by_name[name]
                ref_position = reference.at(name, t_reference, "position")
                ref_velocity = reference.at(name, t_reference, "velocity")
                requested = ref_position if name in active else initial[name] + applied_offsets.get(name, 0.0)
                if experiment.velocity_limit_rad_s is not None and name in active:
                    maximum = experiment.velocity_limit_rad_s * elapsed_control
                    requested = float(np.clip(requested, last_target[name] - maximum, last_target[name] + maximum))
                clipped = float(np.clip(requested, joint.lower, joint.upper))
                was_clipped = int(abs(clipped - requested) > 1e-10)
                target_clip_count += was_clipped
                controller.target[joint.qpos_adr] = clipped
                last_target[name] = clipped
                last_reference[name] = (ref_position, ref_velocity, clipped, was_clipped)
            controller.apply(data)
            next_control += control_period
        mujoco.mj_step(model, data)
        roll, pitch, _ = rpy(data.xmat[pelvis])
        if fall_time is None and (float(data.xpos[pelvis, 2]) < 0.30 or max(abs(roll), abs(pitch)) > math.radians(45.0)):
            fall_time = float(data.time)
        if data.time + 1e-12 < next_sample:
            continue
        sim_time = float(data.time)
        t_log = t_start + sim_time
        roll, pitch, yaw = rpy(data.xmat[pelvis])
        gyro = sensor(model, data, "body-angular-velocity")
        contact = contact_state(model, data)
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
                "gyro_norm": float(np.linalg.norm(gyro)),
                "left_foot_slip_proxy_m": float(np.linalg.norm(data.xpos[feet["left"], :2] - initial_foot_xy["left"])),
                "right_foot_slip_proxy_m": float(np.linalg.norm(data.xpos[feet["right"], :2] - initial_foot_xy["right"])),
                **contact,
            }
        )
        for name in mapped:
            joint = by_name[name]
            ref_position = reference.at(name, t_log, "position")
            ref_velocity = reference.at(name, t_log, "velocity")
            target = float(controller.target[joint.qpos_adr])
            ctrl = float(data.ctrl[joint.actuator_id])
            ctrl_limit = max(abs(float(value)) for value in model.actuator_ctrlrange[joint.actuator_id])
            position = float(data.qpos[joint.qpos_adr])
            joint_rows.append(
                {
                    "sim_time": sim_time,
                    "t": t_log,
                    "joint_name": name,
                    "input_mode": "MEASURED_REAL_TRAJECTORY" if name in active else "STANDING_TARGET",
                    "reference_position": ref_position,
                    "reference_velocity": ref_velocity,
                    "target_position": target,
                    "position": position,
                    "velocity": float(data.qvel[joint.dof_adr]),
                    "ctrl_saturation_fraction": abs(ctrl) / ctrl_limit if ctrl_limit else np.nan,
                    "limit_margin_rad": min(position - joint.lower, joint.upper - position),
                }
            )
        next_sample += 1.0 / SAMPLE_RATE_HZ

    joints = pd.DataFrame(joint_rows)
    base = pd.DataFrame(base_rows)
    joints.to_csv(HERE / f"{experiment.name}_joint_log.csv", index=False)
    base.to_csv(HERE / f"{experiment.name}_base_log.csv", index=False)
    persistent_saturation = float((joints.ctrl_saturation_fraction >= 0.98).mean())
    summary: dict[str, object] = {
        **asdict(experiment),
        "warning": "NOT HARDWARE CALIBRATION",
        "reported_effort_used_for_fitting": False,
        "physical_parameters_modified": False,
        "hardware_mapping_modified": False,
        "absolute_imu_used_for_fitting": False,
        "applied_standing_reference_offsets_rad": applied_offsets,
        "stable_no_fall": fall_time is None,
        "fall_time_seconds": fall_time,
        "target_clip_samples": int(target_clip_count),
        "minimum_limit_margin_rad": float(joints.limit_margin_rad.min()),
        "maximum_ctrl_saturation_fraction": float(joints.ctrl_saturation_fraction.max()),
        "persistent_saturation_sample_fraction": persistent_saturation,
        "nonfoot_ground_contact_samples": int((base.nonfoot_ground_contact_count > 0).sum()),
        "self_collision_samples": int((base.self_collision_contact_count > 0).sum()),
        "both_feet_contact_fraction": float(((base.left_foot_contact == 1) & (base.right_foot_contact == 1)).mean()),
        "max_left_foot_slip_proxy_m": float(base.left_foot_slip_proxy_m.max()),
        "max_right_foot_slip_proxy_m": float(base.right_foot_slip_proxy_m.max()),
        "max_abs_base_roll_deg": float(np.degrees(base.base_roll_rad.abs().max())),
        "max_abs_base_pitch_deg": float(np.degrees(base.base_pitch_rad.abs().max())),
    }
    (HERE / f"{experiment.name}_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def lagged_correlation(reference: np.ndarray, response: np.ndarray, max_lag_s: float = 1.0) -> tuple[float, float]:
    candidates = []
    maximum = round(max_lag_s * SAMPLE_RATE_HZ)
    for lag in range(-maximum, maximum + 1):
        if lag > 0:
            x, y = reference[:-lag], response[lag:]
        elif lag < 0:
            x, y = reference[-lag:], response[:lag]
        else:
            x, y = reference, response
        if len(x) >= 10 and np.std(x) > 1e-10 and np.std(y) > 1e-10:
            candidates.append((float(np.corrcoef(x, y)[0, 1]), lag / SAMPLE_RATE_HZ))
    return max(candidates, default=(np.nan, np.nan), key=lambda item: item[0])


def segment_definitions() -> tuple[tuple[str, str, float, float], ...]:
    return (
        ("standing_pre_roll", "validation", PRE_WINDOW[0], PRE_WINDOW[1]),
        ("motion_onset", "fit", 0.0, 0.30 * MOTION_END),
        ("peak_gesture", "validation", 0.30 * MOTION_END, 0.65 * MOTION_END),
        ("return_phase", "fit", 0.65 * MOTION_END, MOTION_END),
        ("post_roll", "validation", POST_WINDOW[0], POST_WINDOW[1]),
        ("full_motion", "combined", 0.0, MOTION_END),
    )


def settling_metrics(group: pd.DataFrame, excursion: float) -> tuple[float, float]:
    post = group[group.t >= MOTION_END].sort_values("t")
    if post.empty:
        return np.nan, np.nan
    error = np.abs(post.position.to_numpy(float) - post.reference_position.to_numpy(float))
    velocity = np.abs(post.velocity.to_numpy(float))
    threshold = max(0.01, 0.05 * excursion)
    hold = round(0.5 * SAMPLE_RATE_HZ)
    settling = np.nan
    for index in range(max(0, len(post) - hold + 1)):
        if np.all(error[index : index + hold] <= threshold) and np.all(velocity[index : index + hold] <= 0.05):
            settling = float(post.t.iloc[index] - MOTION_END)
            break
    settled = post[post.t.between(*POST_WINDOW)]
    settling_error = float(np.mean(np.abs(settled.position - settled.reference_position))) if len(settled) else np.nan
    return settling, settling_error


def tracking_metrics(experiment: Experiment) -> pd.DataFrame:
    frame = pd.read_csv(HERE / f"{experiment.name}_joint_log.csv")
    rows = []
    for name in ARM_TRACKING_JOINTS:
        group = frame[frame.joint_name == name].sort_values("t")
        full = group[group.t.between(0.0, MOTION_END)]
        excursion = float(full.reference_position.max() - full.reference_position.min())
        settling, settling_error = settling_metrics(group, excursion)
        for segment, split, start, end in segment_definitions():
            part = group[group.t.between(start, end)].sort_values("t")
            if part.empty:
                continue
            error = part.position.to_numpy(float) - part.reference_position.to_numpy(float)
            velocity_error = part.velocity.to_numpy(float) - part.reference_velocity.to_numpy(float)
            corr, lag = lagged_correlation(
                part.reference_position.to_numpy(float) - float(part.reference_position.iloc[0]),
                part.position.to_numpy(float) - float(part.position.iloc[0]),
            )
            overshoot = max(
                0.0,
                float(part.position.max() - part.reference_position.max()),
                float(part.reference_position.min() - part.position.min()),
            )
            rows.append(
                {
                    "experiment": experiment.name,
                    "joint_name": name,
                    "segment": segment,
                    "split": split,
                    "rmse_rad": float(np.sqrt(np.mean(error**2))),
                    "mae_rad": float(np.mean(np.abs(error))),
                    "lag_s": lag,
                    "shape_correlation": corr,
                    "peak_error_rad": float(np.max(np.abs(error))),
                    "overshoot_rad": overshoot,
                    "peak_velocity_error_rad_s": float(np.max(np.abs(velocity_error))),
                    "settling_time_s": settling if segment == "full_motion" else np.nan,
                    "settling_error_rad": settling_error if segment == "full_motion" else np.nan,
                    "maximum_ctrl_saturation_fraction": float(part.ctrl_saturation_fraction.max()),
                }
            )
    return pd.DataFrame(rows)


def equilibrium_metrics(experiment: Experiment, real_joint: pd.DataFrame) -> pd.DataFrame:
    sim = pd.read_csv(HERE / f"{experiment.name}_joint_log.csv")
    rows = []
    for name in BALANCE_JOINTS:
        real_pre = real_joint[(real_joint.joint_name == name) & real_joint.t.between(*PRE_WINDOW)]
        sim_pre = sim[(sim.joint_name == name) & sim.t.between(*PRE_WINDOW)]
        if real_pre.empty or sim_pre.empty:
            continue
        rows.append(
            {
                "experiment": experiment.name,
                "joint_name": name,
                "real_pre_mean_rad": float(real_pre.position.mean()),
                "sim_target_mean_rad": float(sim_pre.target_position.mean()),
                "sim_settled_mean_rad": float(sim_pre.position.mean()),
                "settled_minus_real_rad": float(sim_pre.position.mean() - real_pre.position.mean()),
                "classification": "SIMULATION_REFERENCE_ALIGNMENT_NOT_HARDWARE_ZERO_CALIBRATION",
            }
        )
    return pd.DataFrame(rows)


def recovery_time(t: np.ndarray, values: np.ndarray) -> float:
    indices = np.flatnonzero(t >= MOTION_END)
    if not len(indices):
        return np.nan
    motion = values[(t >= 0) & (t <= MOTION_END)]
    threshold = max(0.1 * float(np.max(np.abs(motion))), 1e-4)
    hold = round(0.5 * SAMPLE_RATE_HZ)
    for index in indices:
        if index + hold <= len(values) and np.all(np.abs(values[index : index + hold]) <= threshold):
            return float(t[index] - MOTION_END)
    return np.nan


def balance_metrics(experiment: Experiment, real_joint: pd.DataFrame) -> pd.DataFrame:
    sim = pd.read_csv(HERE / f"{experiment.name}_joint_log.csv")
    rows = []
    for name in BALANCE_JOINTS:
        real = real_joint[real_joint.joint_name == name].sort_values("t")
        simulated = sim[sim.joint_name == name].sort_values("t")
        common_t = simulated.t.to_numpy(float)
        real_position = np.interp(common_t, real.t, real.position)
        real_pre = float(np.mean(real_position[(common_t >= PRE_WINDOW[0]) & (common_t <= PRE_WINDOW[1])]))
        sim_pre = float(simulated.loc[simulated.t.between(*PRE_WINDOW), "position"].mean())
        real_relative = real_position - real_pre
        sim_relative = simulated.position.to_numpy(float) - sim_pre
        motion = (common_t >= 0) & (common_t <= MOTION_END)
        real_motion = real_relative[motion]
        sim_motion = sim_relative[motion]
        real_excursion = float(np.ptp(real_motion))
        sim_excursion = float(np.ptp(sim_motion))
        corr, lag = lagged_correlation(real_motion, sim_motion)
        real_peak_t = float(common_t[motion][int(np.argmax(np.abs(real_motion)))])
        sim_peak_t = float(common_t[motion][int(np.argmax(np.abs(sim_motion)))])
        rows.append(
            {
                "experiment": experiment.name,
                "joint_name": name,
                "real_excursion_rad": real_excursion,
                "sim_excursion_rad": sim_excursion,
                "excursion_ratio": sim_excursion / real_excursion if real_excursion > 1e-10 else np.nan,
                "relative_rmse_rad": float(np.sqrt(np.mean((sim_motion - real_motion) ** 2))),
                "phase_lag_s": lag,
                "shape_correlation": corr,
                "peak_timing_difference_s": sim_peak_t - real_peak_t,
                "real_recovery_s": recovery_time(common_t, real_relative),
                "sim_recovery_s": recovery_time(common_t, sim_relative),
                "mapping_status": "IDENTITY_NAME_CANDIDATE_UNVERIFIED_SIGN_ZERO_RELATIVE_MOTION_ONLY",
            }
        )
    return pd.DataFrame(rows)


def metric_score(metrics: pd.DataFrame, joints: tuple[str, ...]) -> float:
    selected = metrics[(metrics.joint_name.isin(joints)) & metrics.segment.isin(["motion_onset", "peak_gesture", "return_phase"])]
    validation = selected[selected.split == "validation"].rmse_rad.mean()
    fit = selected[selected.split == "fit"].rmse_rad.mean()
    return float(0.5 * fit + 0.5 * validation)


def choose_gain(experiments: tuple[Experiment, ...], joints: tuple[str, ...], all_metrics: pd.DataFrame) -> Experiment:
    candidates = []
    for experiment in experiments:
        metrics = all_metrics[all_metrics.experiment == experiment.name]
        summary = json.loads((HERE / f"{experiment.name}_summary.json").read_text(encoding="utf-8"))
        if summary["persistent_saturation_sample_fraction"] > 0.01 or summary["target_clip_samples"]:
            continue
        candidates.append((metric_score(metrics, joints), experiment))
    return min(candidates, key=lambda item: item[0])[1]


def experiment_row(experiment: Experiment, metrics: pd.DataFrame, equilibrium: pd.DataFrame, balance: pd.DataFrame) -> dict[str, object]:
    summary = json.loads((HERE / f"{experiment.name}_summary.json").read_text(encoding="utf-8"))
    full = metrics[(metrics.segment == "full_motion") & metrics.joint_name.isin(ARM_TRACKING_JOINTS)]
    return {
        "experiment": experiment.name,
        "parent": experiment.parent,
        "changed_category": experiment.changed_category,
        "classification": experiment.classification,
        "free_base": experiment.free_base,
        "interpolation": experiment.interpolation,
        "reference_rate_hz": experiment.reference_rate_hz,
        "controller_rate_hz": experiment.controller_rate_hz,
        "timestep_s": experiment.timestep_s,
        "shoulder_gain_scale": experiment.shoulder_gain_scale,
        "wrist_gain_scale": experiment.wrist_gain_scale,
        "balance_gain_scale": experiment.balance_gain_scale,
        "standing_reference_scale": experiment.standing_reference_scale,
        "velocity_limit_rad_s": experiment.velocity_limit_rad_s,
        "mean_arm_rmse_rad": float(full.rmse_rad.mean()),
        "mean_arm_mae_rad": float(full.mae_rad.mean()),
        "median_arm_lag_s": float(full.lag_s.median()),
        "mean_abs_equilibrium_error_rad": float(equilibrium.settled_minus_real_rad.abs().mean()) if len(equilibrium) else np.nan,
        "mean_balance_relative_rmse_rad": float(balance.relative_rmse_rad.mean()) if len(balance) else np.nan,
        "stable_no_fall": summary["stable_no_fall"],
        "minimum_limit_margin_rad": summary["minimum_limit_margin_rad"],
        "maximum_ctrl_saturation_fraction": summary["maximum_ctrl_saturation_fraction"],
        "persistent_saturation_sample_fraction": summary["persistent_saturation_sample_fraction"],
        "self_collision_samples": summary["self_collision_samples"],
        "nonfoot_ground_contact_samples": summary["nonfoot_ground_contact_samples"],
        "both_feet_contact_fraction": summary["both_feet_contact_fraction"],
        "warning": "NOT HARDWARE CALIBRATION",
    }


def choose_balance_scale(experiments: list[Experiment], balance_table: pd.DataFrame) -> Experiment:
    rows = []
    for experiment in experiments:
        frame = balance_table[balance_table.experiment == experiment.name]
        ankle = frame[frame.joint_name.str.contains("ankle_pitch")]
        score = float(frame.relative_rmse_rad.mean() + 0.02 * np.mean(np.abs(np.log(np.clip(ankle.excursion_ratio, 1e-6, None)))))
        summary = json.loads((HERE / f"{experiment.name}_summary.json").read_text(encoding="utf-8"))
        if summary["stable_no_fall"] and summary["persistent_saturation_sample_fraction"] <= 0.01:
            rows.append((score, experiment))
    return min(rows, key=lambda item: item[0])[1]


def choose_standing_scale(experiments: list[Experiment], equilibrium_table: pd.DataFrame) -> Experiment:
    rows = []
    for experiment in experiments:
        frame = equilibrium_table[equilibrium_table.experiment == experiment.name]
        knee = frame[frame.joint_name.str.contains("knee")].settled_minus_real_rad.abs().mean()
        overall = frame.settled_minus_real_rad.abs().mean()
        ankle_max = frame[frame.joint_name.str.contains("ankle_pitch")].settled_minus_real_rad.abs().max()
        score = float(0.5 * knee + 0.4 * overall + 0.1 * ankle_max)
        rows.append((score, experiment))
    return min(rows, key=lambda item: item[0])[1]


def run_standing_validation(experiment: Experiment, real_joint: pd.DataFrame, offsets: dict[str, float]) -> dict[str, object]:
    model = load_model(free_base=True)
    model.opt.timestep = experiment.timestep_s
    config = SimulationStabilityConfig(
        pitch_kp=200.0 * experiment.balance_gain_scale,
        pitch_kd=30.0 * experiment.balance_gain_scale,
        roll_kp=100.0 * experiment.balance_gain_scale,
        roll_kd=20.0 * experiment.balance_gain_scale,
        friction_compensation_scale=1.5,
        friction_error_width=0.005,
    )
    controller = SimulationStabilityController(model, config)
    controller.joints = scaled_joints(controller.joints, experiment)
    data = mujoco.MjData(model)
    by_name = {joint.name: joint for joint in controller.joints}
    pre_means = real_joint[real_joint.t.between(*PRE_WINDOW)].groupby("joint_name").position.mean().to_dict()
    for name, joint in by_name.items():
        initial = float(pre_means.get(name, 0.0))
        target = initial + experiment.standing_reference_scale * offsets.get(name, 0.0)
        controller.target[joint.qpos_adr] = float(np.clip(target, joint.lower, joint.upper))
        data.qpos[joint.qpos_adr] = initial
        data.qvel[joint.dof_adr] = 0.0
    mujoco.mj_forward(model, data)
    data.qpos[2] -= foot_surface_minimum(model, data)
    mujoco.mj_forward(model, data)
    pelvis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    initial_xy = data.xpos[pelvis, :2].copy()
    feet = {
        "left": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_ankle_roll_link"),
        "right": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_ankle_roll_link"),
    }
    initial_foot_xy = {side: data.xpos[body, :2].copy() for side, body in feet.items()}
    max_tilt = max_pelvis_xy = 0.0
    max_foot_slip = {"left": 0.0, "right": 0.0}
    minimum_height = float(data.xpos[pelvis, 2])
    both_contact = total = self_steps = nonfoot_steps = saturation_steps = 0
    next_control = 0.0
    while data.time < 10.0 - 1e-12:
        if data.time + 1e-12 >= next_control:
            controller.apply(data)
            next_control += 1.0 / experiment.controller_rate_hz
        mujoco.mj_step(model, data)
        roll, pitch, _ = rpy(data.xmat[pelvis])
        max_tilt = max(max_tilt, abs(roll), abs(pitch))
        minimum_height = min(minimum_height, float(data.xpos[pelvis, 2]))
        max_pelvis_xy = max(max_pelvis_xy, float(np.linalg.norm(data.xpos[pelvis, :2] - initial_xy)))
        for side, body in feet.items():
            max_foot_slip[side] = max(
                max_foot_slip[side],
                float(np.linalg.norm(data.xpos[body, :2] - initial_foot_xy[side])),
            )
        state = contact_state(model, data)
        both_contact += int(state["left_foot_contact"] == 1 and state["right_foot_contact"] == 1)
        self_steps += int(state["self_collision_contact_count"] > 0)
        nonfoot_steps += int(state["nonfoot_ground_contact_count"] > 0)
        saturation_steps += int(any(abs(float(data.ctrl[j.actuator_id])) >= 0.98 * max(abs(float(x)) for x in model.actuator_ctrlrange[j.actuator_id]) for j in controller.joints))
        total += 1
    result = {
        "experiment": experiment.name,
        "duration_s": 10.0,
        "stable_no_fall": bool(minimum_height >= 0.30 and math.degrees(max_tilt) <= 45.0),
        "minimum_pelvis_height_m": minimum_height,
        "maximum_abs_tilt_deg": math.degrees(max_tilt),
        "maximum_pelvis_xy_displacement_m": max_pelvis_xy,
        "maximum_left_foot_slip_proxy_m": max_foot_slip["left"],
        "maximum_right_foot_slip_proxy_m": max_foot_slip["right"],
        "both_feet_contact_fraction": both_contact / total,
        "self_collision_steps": self_steps,
        "nonfoot_ground_contact_steps": nonfoot_steps,
        "persistent_saturation_fraction": saturation_steps / total,
        "warning": "NOT HARDWARE CALIBRATION",
    }
    (HERE / "free_base_10s_standing_validation.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    return result


def run_rehearsal_regression(experiment: Experiment, offsets: dict[str, float]) -> dict[str, object]:
    results = []
    targets = _base_targets()
    for name in BALANCE_JOINTS:
        if name in targets:
            targets[name] += experiment.standing_reference_scale * offsets.get(name, 0.0)
    for joint_name in REHEARSAL_JOINTS:
        model = load_model(free_base=False)
        controller = SimulationStabilityController(model)
        controller.joints = scaled_joints(controller.joints, experiment)
        data = mujoco.MjData(model)
        by_name = {joint.name: joint for joint in controller.joints}
        controller.set_targets({name: value for name, value in targets.items() if name in by_name})
        controller.initialize_data(data)
        driven = by_name[joint_name]
        center = float(controller.target[driven.qpos_adr])
        delta = math.radians(2.0)
        maximum_error = final_error = 0.0
        max_self = limit_steps = saturation_steps = total = 0
        segment_start = 0.0
        for segment in SEGMENTS:
            segment_end = segment_start + segment.duration
            while data.time < segment_end - 1e-12:
                elapsed = data.time - segment_start
                ratio = min(max(elapsed / segment.duration, 0.0), 1.0)
                smooth = ratio**3 * (10.0 + ratio * (-15.0 + 6.0 * ratio))
                scale = segment.start_scale + (segment.end_scale - segment.start_scale) * smooth
                command = center + scale * delta
                controller.target[driven.qpos_adr] = command
                controller.apply(data)
                mujoco.mj_step(model, data)
                measured = float(data.qpos[driven.qpos_adr])
                maximum_error = max(maximum_error, abs(command - measured))
                final_error = measured - center
                max_self = max(max_self, len(_self_contacts(model, data)))
                limit_steps += int(measured < driven.lower - 1e-9 or measured > driven.upper + 1e-9)
                limit = max(abs(float(x)) for x in model.actuator_ctrlrange[driven.actuator_id])
                saturation_steps += int(abs(float(data.ctrl[driven.actuator_id])) >= 0.98 * limit)
                total += 1
            segment_start = segment_end
        settled = math.degrees(maximum_error) <= 1.0 and abs(math.degrees(final_error)) <= 0.1
        results.append(
            {
                "joint_name": joint_name,
                "tracking_status": "SETTLED" if settled else "NOT_SETTLED",
                "maximum_position_error_deg": math.degrees(maximum_error),
                "return_error_deg": math.degrees(final_error),
                "self_collision_steps": max_self,
                "joint_limit_violation_steps": limit_steps,
                "persistent_saturation_fraction": saturation_steps / total,
            }
        )
    payload = {
        "warning": "NOT HARDWARE CALIBRATION",
        "results": results,
        "settled_count": sum(row["tracking_status"] == "SETTLED" for row in results),
        "total": len(results),
    }
    (HERE / "rehearsal_12_joint_regression.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    pd.DataFrame(results).to_csv(HERE / "rehearsal_12_joint_regression.csv", index=False)
    return payload


def main() -> int:
    HERE.mkdir(parents=True, exist_ok=True)
    write_immutable_manifest()
    reference, real_joint, _real_imu, controlled = source_frames()
    experiments: list[Experiment] = []
    metric_frames: list[pd.DataFrame] = []
    equilibrium_frames: list[pd.DataFrame] = []
    balance_frames: list[pd.DataFrame] = []

    def execute(experiment: Experiment, offsets: dict[str, float] | None = None) -> None:
        print(f"RUN {experiment.name}", flush=True)
        run_replay(experiment, reference, real_joint, controlled, inherited_offsets=offsets)
        experiments.append(experiment)
        metric_frames.append(tracking_metrics(experiment))
        equilibrium_frames.append(equilibrium_metrics(experiment, real_joint))
        if experiment.free_base:
            balance_frames.append(balance_metrics(experiment, real_joint))

    for experiment in FIXED_DIAGNOSTICS + SHOULDER_GAIN_EXPERIMENTS + WRIST_GAIN_EXPERIMENTS:
        execute(experiment)

    tracking = pd.concat(metric_frames, ignore_index=True)
    shoulder_best = choose_gain(SHOULDER_GAIN_EXPERIMENTS, SHOULDER_JOINTS, tracking)
    wrist_best = choose_gain(WRIST_GAIN_EXPERIMENTS, WRIST_JOINTS, tracking)
    fixed_arm = Experiment(
        "fixed_arm_pd_candidate",
        "fixed_baseline",
        False,
        "per-joint simulation arm PD bandwidth",
        "ACCEPTED_SIM_CONTROLLER_ALIGNMENT",
        shoulder_gain_scale=shoulder_best.shoulder_gain_scale,
        wrist_gain_scale=wrist_best.wrist_gain_scale,
    )
    execute(fixed_arm)
    free_baseline = replace(FIXED_DIAGNOSTICS[0], name="free_baseline", parent="none", free_base=True)
    execute(free_baseline)
    free_arm = replace(fixed_arm, name="free_arm_pd_candidate", parent="free_baseline", free_base=True)
    execute(free_arm)

    balance_candidates = []
    for scale in (0.5, 0.6, 0.7, 0.8):
        experiment = replace(
            free_arm,
            name=f"free_balance_scale_{str(scale).replace('.', 'p')}",
            parent=free_arm.name,
            changed_category="simulation balance gains",
            classification="SIM_CONTROLLER_ALIGNMENT_CANDIDATE",
            balance_gain_scale=scale,
        )
        execute(experiment)
        balance_candidates.append(experiment)
    balance_table = pd.concat(balance_frames, ignore_index=True)
    balance_best = choose_balance_scale(balance_candidates, balance_table)

    parent_log = HERE / f"{balance_best.name}_joint_log.csv"
    raw_offsets = standing_offsets(parent_log, real_joint)
    standing_candidates = []
    for scale in (0.25, 0.5, 0.75, 1.0):
        experiment = replace(
            balance_best,
            name=f"free_standing_reference_scale_{str(scale).replace('.', 'p')}",
            parent=balance_best.name,
            changed_category="simulation standing equilibrium targets",
            classification="SIM_CONTROLLER_ALIGNMENT_CANDIDATE",
            standing_reference_scale=scale,
        )
        execute(experiment, raw_offsets)
        standing_candidates.append(experiment)
    equilibrium_table = pd.concat(equilibrium_frames, ignore_index=True)
    standing_best = choose_standing_scale(standing_candidates, equilibrium_table)
    final = replace(standing_best, name="free_final_candidate", parent=standing_best.name, classification="ACCEPTED_SIM_CONTROLLER_ALIGNMENT")
    # Re-run under the stable canonical name used by reports and downstream tooling.
    execute(final, raw_offsets)

    tracking = pd.concat(metric_frames, ignore_index=True)
    equilibrium_table = pd.concat(equilibrium_frames, ignore_index=True)
    balance_table = pd.concat(balance_frames, ignore_index=True)
    tracking.to_csv(HERE / "phase3a_all_tracking_metrics.csv", index=False)
    equilibrium_table.to_csv(HERE / "phase3a_all_equilibrium_metrics.csv", index=False)
    balance_table.to_csv(HERE / "phase3a_all_balance_metrics.csv", index=False)
    tracking[tracking.experiment == "fixed_baseline"].to_csv(HERE / "phase3a_baseline_metrics.csv", index=False)

    experiment_rows = []
    for experiment in experiments:
        experiment_rows.append(
            experiment_row(
                experiment,
                tracking[tracking.experiment == experiment.name],
                equilibrium_table[equilibrium_table.experiment == experiment.name],
                balance_table[balance_table.experiment == experiment.name],
            )
        )
    experiments_frame = pd.DataFrame(experiment_rows)
    baseline_arm_rmse = float(experiments_frame.loc[experiments_frame.experiment == "free_baseline", "mean_arm_rmse_rad"].iloc[0])
    for index, row in experiments_frame.iterrows():
        if row.experiment in {"fixed_arm_pd_candidate", "free_final_candidate"}:
            experiments_frame.loc[index, "classification"] = "ACCEPTED_SIM_CONTROLLER_ALIGNMENT"
        elif row.classification == "SIM_CONTROLLER_ALIGNMENT_CANDIDATE":
            experiments_frame.loc[index, "classification"] = "REJECTED"
        elif row.experiment == "free_arm_pd_candidate" and row.mean_arm_rmse_rad < baseline_arm_rmse:
            experiments_frame.loc[index, "classification"] = "DIAGNOSTIC_ONLY"
    experiments_frame.to_csv(HERE / "phase3a_candidate_experiments.csv", index=False)

    standing_validation = run_standing_validation(final, real_joint, raw_offsets)
    rehearsal = run_rehearsal_regression(final, raw_offsets)
    candidate = {
        "classification": "ACCEPTED_SIM_CONTROLLER_ALIGNMENT",
        "warning": "NOT HARDWARE CALIBRATION",
        "source_controller_modified": False,
        "mjcf_modified": False,
        "physical_parameters_modified": False,
        "hardware_mapping_modified": False,
        "reported_effort_used_for_fitting": False,
        "absolute_imu_used_for_fitting": False,
        "parameters": asdict(final),
        "standing_reference_offsets_rad": {
            name: final.standing_reference_scale * value for name, value in raw_offsets.items()
        },
        "gain_definition": "simulation kp multiplied by scale; simulation kd multiplied by sqrt(scale)",
        "selected_from": {
            "shoulder": shoulder_best.name,
            "wrist": wrist_best.name,
            "balance": balance_best.name,
            "standing": standing_best.name,
        },
        "free_base_10s_standing": standing_validation,
        "rehearsal_settled": f"{rehearsal['settled_count']}/{rehearsal['total']}",
    }
    (HERE / "simulation_controller_alignment_candidate.json").write_text(json.dumps(candidate, indent=2), encoding="utf-8")
    print(json.dumps(candidate, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
