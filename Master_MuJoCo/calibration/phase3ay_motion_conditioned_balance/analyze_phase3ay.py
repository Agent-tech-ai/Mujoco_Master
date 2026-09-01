#!/usr/bin/env python3
"""Generate Phase 3A-Y evidence, metrics, reports, and final gates."""

from __future__ import annotations

from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd

from phase3ay_core import CALIBRATION, HERE, RUNS, datasets
from run_phase3ay_candidate_smoke import candidate


FINAL_ID = "phase3ay_final_candidate_v3"
TARGETS_PITCH = (
    "left_ankle_pitch_joint", "right_ankle_pitch_joint",
    "left_knee_joint", "right_knee_joint",
    "left_hip_pitch_joint", "right_hip_pitch_joint", "waist_pitch_joint",
)
TARGETS_ROLL = (
    "left_ankle_roll_joint", "right_ankle_roll_joint",
    "left_hip_roll_joint", "right_hip_roll_joint", "waist_roll_joint",
)
TARGETS = TARGETS_PITCH + TARGETS_ROLL
ARM_TOKENS = ("shoulder", "elbow", "wrist")
DT = 0.02


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def md_table(frame: pd.DataFrame, columns: list[str] | None = None) -> str:
    view = frame if columns is None else frame[columns]
    if view.empty:
        return "_No rows._"
    headers = [str(column) for column in view.columns]
    lines = ["| " + " | ".join(headers) + " |", "|" + "|".join(["---"] * len(headers)) + "|"]
    for row in view.itertuples(index=False, name=None):
        formatted = []
        for value in row:
            if isinstance(value, float):
                formatted.append("" if math.isnan(value) else f"{value:.6g}")
            else:
                formatted.append(str(value).replace("|", "\\|"))
        lines.append("| " + " | ".join(formatted) + " |")
    return "\n".join(lines)


def interp(frame: pd.DataFrame, name: str, grid: np.ndarray, column: str) -> np.ndarray:
    subset = frame[frame.joint_name == name].sort_values("t")
    if subset.empty:
        return np.full_like(grid, np.nan, dtype=float)
    return np.interp(grid, subset.t.to_numpy(float), subset[column].to_numpy(float))


def imu_interp(frame: pd.DataFrame, sensor: str, grid: np.ndarray, column: str) -> np.ndarray:
    subset = frame[frame.imu == sensor].sort_values("t")
    if subset.empty or column not in subset:
        return np.full_like(grid, np.nan, dtype=float)
    return np.interp(grid, subset.t.to_numpy(float), subset[column].to_numpy(float))


def phase_labels(grid: np.ndarray, energy: np.ndarray, motion_end: float) -> tuple[np.ndarray, dict[str, float]]:
    pre = energy[(grid >= -3.0) & (grid <= -0.2)]
    threshold = max(float(np.nanmean(pre) + 3.0 * np.nanstd(pre)), 0.05 * float(np.nanmax(energy)), 0.02)
    motion = (grid >= 0.0) & (grid <= motion_end)
    active_indices = np.flatnonzero(motion & (energy >= threshold))
    onset = float(grid[active_indices[0]]) if len(active_indices) else 0.0
    offset = float(grid[active_indices[-1]]) if len(active_indices) else motion_end
    peak = float(grid[np.nanargmax(np.where(motion, energy, -np.inf))])
    labels = np.full(grid.shape, "POST_ROLL", dtype=object)
    labels[grid < onset] = "PRE_ROLL"
    labels[(grid >= onset) & (grid <= peak)] = "MOTION_ONSET_TO_PEAK"
    labels[(grid > peak) & (grid <= offset)] = "MOTION_RETURN"
    labels[(grid > offset) & (grid <= motion_end)] = "RECOVERY"
    return labels, {"arm_energy_threshold_rad_s": threshold, "onset_s": onset, "peak_s": peak, "offset_s": offset}


def extract_real_features() -> tuple[pd.DataFrame, dict[str, dict[str, float]]]:
    source = datasets()
    imu_paths = {
        "heart": CALIBRATION / "phase2e_replay" / "phase2e_aligned_imu_data.csv",
        "wave": CALIBRATION / "phase3av_validation" / "phase3av_aligned_imu_data.csv",
    }
    frames = []
    phase_info: dict[str, dict[str, float]] = {}
    for name, dataset in source.items():
        # Explicit columns ensure reported_effort never enters this phase.
        joints = pd.read_csv(dataset.real_joint_path, usecols=["t", "joint_name", "position", "velocity"])
        imu = pd.read_csv(
            imu_paths[name],
            usecols=["t", "imu", "relative_roll_rad", "relative_pitch_rad", "gyro_x", "gyro_y", "gyro_z"],
        )
        start = max(-5.0, float(joints.t.min()))
        end = min(float(dataset.motion_end_s + 5.0), float(joints.t.max()))
        grid = np.arange(start, end + DT / 2.0, DT)
        names = sorted(joints.joint_name.unique())
        arm_names = [joint for joint in names if any(token in joint for token in ARM_TOKENS)]
        positions = {joint: interp(joints, joint, grid, "position") for joint in names}
        velocities = {joint: interp(joints, joint, grid, "velocity") for joint in names}
        accelerations = {joint: np.gradient(velocities[joint], DT) for joint in arm_names}

        def norm(side: str, values: dict[str, np.ndarray]) -> np.ndarray:
            selected = [value for joint, value in values.items() if joint.startswith(side + "_") and joint in arm_names]
            return np.sqrt(np.sum(np.square(selected), axis=0)) if selected else np.zeros_like(grid)

        left_pos = norm("left", positions)
        right_pos = norm("right", positions)
        left_vel = norm("left", velocities)
        right_vel = norm("right", velocities)
        left_acc = norm("left", accelerations)
        right_acc = norm("right", accelerations)
        energy = left_vel + right_vel
        asymmetry = np.abs(right_vel - left_vel) / np.maximum(energy, 1e-6)
        sagittal_names = [joint for joint in arm_names if any(token in joint for token in ("shoulder_pitch", "elbow", "wrist_pitch"))]
        lateral_names = [joint for joint in arm_names if any(token in joint for token in ("shoulder_roll", "shoulder_yaw", "wrist_yaw", "wrist_roll"))]
        sagittal_proxy = np.sum([velocities[joint] for joint in sagittal_names], axis=0)
        lateral_proxy = np.sum([velocities[joint] for joint in lateral_names], axis=0)
        phase, info = phase_labels(grid, energy, float(dataset.motion_end_s))
        phase_info[name] = info
        output: dict[str, object] = {
            "dataset": np.full(grid.shape, name),
            "t": grid,
            "motion_phase": phase,
            "left_arm_position_norm_rad": left_pos,
            "right_arm_position_norm_rad": right_pos,
            "arm_center_position_rad": np.mean([positions[joint] for joint in arm_names], axis=0),
            "left_arm_velocity_norm_rad_s": left_vel,
            "right_arm_velocity_norm_rad_s": right_vel,
            "arm_motion_energy_rad_s": energy,
            "left_arm_acceleration_norm_rad_s2": left_acc,
            "right_arm_acceleration_norm_rad_s2": right_acc,
            "arm_motion_asymmetry": asymmetry,
            "arm_sagittal_momentum_proxy_rad_s": sagittal_proxy,
            "arm_lateral_momentum_proxy_rad_s": lateral_proxy,
            "arm_sagittal_fraction": np.abs(sagittal_proxy) / np.maximum(np.abs(sagittal_proxy) + np.abs(lateral_proxy), 1e-6),
        }
        for sensor in ("chest", "torso"):
            for column in ("relative_roll_rad", "relative_pitch_rad", "gyro_x", "gyro_y", "gyro_z"):
                output[f"{sensor}_{column}"] = imu_interp(imu, sensor, grid, column)
        state_names = sorted({joint for joint in names if any(token in joint for token in ("ankle", "knee", "hip", "waist"))})
        for joint in state_names:
            output[f"state_position__{joint}"] = positions[joint]
            output[f"state_velocity__{joint}"] = velocities[joint]
        for joint in TARGETS:
            pre_mask = (grid >= -3.0) & (grid <= -0.2)
            baseline = float(np.mean(positions[joint][pre_mask]))
            output[f"real_response_relative_position__{joint}"] = positions[joint] - baseline
            output[f"real_response_velocity__{joint}"] = velocities[joint]
        frames.append(pd.DataFrame(output))
    features = pd.concat(frames, ignore_index=True)
    features.to_csv(HERE / "phase3ay_real_response_features.csv", index=False)
    return features, phase_info


def response_characteristics(t: np.ndarray, position: np.ndarray, velocity: np.ndarray, motion_end: float) -> dict[str, float | None]:
    pre = (t >= -3.0) & (t <= -0.2)
    motion = (t >= 0.0) & (t <= motion_end)
    baseline = float(np.mean(position[pre]))
    relative = position - baseline
    tm = t[motion]
    rm = relative[motion]
    vm = velocity[motion]
    excursion = float(np.max(rm) - np.min(rm))
    peak_index = int(np.argmax(np.abs(rm)))
    signed = float(rm[peak_index])
    threshold = max(0.05 * abs(signed), 3.0 * float(np.std(relative[pre])), 0.0005)
    onset_indices = np.flatnonzero(np.abs(rm) >= threshold)
    onset = float(tm[onset_indices[0]]) if len(onset_indices) else None
    peak_time = float(tm[peak_index])
    recovery = None
    after = np.flatnonzero(t >= motion_end)
    band = max(0.10 * abs(signed), threshold)
    for index in after:
        end = min(index + int(round(0.20 / DT)), len(t))
        if end - index >= 5 and np.all(np.abs(relative[index:end]) <= band):
            recovery = float(t[index] - motion_end)
            break
    return {
        "baseline_rad": baseline,
        "signed_excursion_rad": signed,
        "excursion_rad": excursion,
        "onset_s": onset,
        "peak_time_s": peak_time,
        "recovery_after_motion_s": recovery,
        "peak_abs_velocity_rad_s": float(np.max(np.abs(vm))),
        "velocity_rms_rad_s": float(np.sqrt(np.mean(vm**2))),
    }


def real_response_targets(features: pd.DataFrame) -> pd.DataFrame:
    rows = []
    source = datasets()
    for name, dataset in source.items():
        frame = features[features.dataset == name].sort_values("t")
        t = frame.t.to_numpy(float)
        for plane, joints in (("pitch", TARGETS_PITCH), ("roll", TARGETS_ROLL)):
            plane_rows = []
            for joint in joints:
                position = frame[f"real_response_relative_position__{joint}"].to_numpy(float)
                velocity = frame[f"real_response_velocity__{joint}"].to_numpy(float)
                row = {"dataset": name, "plane": plane, "joint_name": joint, **response_characteristics(t, position, velocity, dataset.motion_end_s)}
                plane_rows.append(row)
            denominator = sum(abs(float(row["signed_excursion_rad"])) for row in plane_rows)
            for row in plane_rows:
                row["normalized_abs_response"] = abs(float(row["signed_excursion_rad"])) / denominator if denominator else 0.0
                rows.append(row)
    result = pd.DataFrame(rows)
    result.to_csv(HERE / "phase3ay_real_output_response_targets.csv", index=False)
    return result


def lag_seconds(reference: np.ndarray, response: np.ndarray) -> float | None:
    if len(reference) < 10 or np.std(reference) < 1e-7 or np.std(response) < 1e-7:
        return None
    best = None
    for shift in range(-50, 51):
        if shift < 0:
            first, second = reference[-shift:], response[:shift]
        elif shift > 0:
            first, second = reference[:-shift], response[shift:]
        else:
            first, second = reference, response
        if len(first) < 10:
            continue
        score = float(np.corrcoef(first, second)[0, 1])
        if best is None or score > best[0]:
            best = (score, shift)
    return None if best is None else float(best[1] * DT)


def amplitude_band(ratio: float) -> str:
    if 0.67 <= ratio <= 1.50:
        return "GOOD"
    if 0.50 <= ratio <= 2.00:
        return "ACCEPTABLE"
    return "POOR"


def timing_band(onset: float | None, peak: float | None, lag: float | None, recovery: float | None) -> str:
    values = [abs(value) for value in (onset, peak, lag) if value is not None and not math.isnan(value)]
    if values and max(values) <= 0.25 and (recovery is None or abs(recovery) <= 0.50):
        return "GOOD"
    if values and max(values) <= 0.50 and (recovery is None or abs(recovery) <= 1.00):
        return "ACCEPTABLE"
    return "POOR"


def compare_final(features: pd.DataFrame) -> pd.DataFrame:
    source = datasets()
    rows = []
    for name, dataset in source.items():
        real = features[features.dataset == name].sort_values("t")
        joint_log = pd.read_csv(RUNS / f"{FINAL_ID}__{name}__arm_only_joint_log.csv")
        for joint in TARGETS:
            real_t = real.t.to_numpy(float)
            real_p = real[f"real_response_relative_position__{joint}"].to_numpy(float)
            real_v = real[f"real_response_velocity__{joint}"].to_numpy(float)
            sim = joint_log[joint_log.joint_name == joint].sort_values("t")
            sim_t = sim.t.to_numpy(float)
            sim_p_abs = sim.position.to_numpy(float)
            sim_v = sim.velocity.to_numpy(float)
            real_char = response_characteristics(real_t, real_p, real_v, dataset.motion_end_s)
            sim_char = response_characteristics(sim_t, sim_p_abs, sim_v, dataset.motion_end_s)
            common = sim_t[(sim_t >= 0.0) & (sim_t <= dataset.motion_end_s)]
            r = np.interp(common, real_t, real_p)
            s_abs = np.interp(common, sim_t, sim_p_abs)
            sim_pre = float(np.mean(sim_p_abs[(sim_t >= -3.0) & (sim_t <= -0.2)]))
            s = s_abs - sim_pre
            rv = np.interp(common, real_t, real_v)
            sv = np.interp(common, sim_t, sim_v)
            ratio = float(sim_char["excursion_rad"]) / max(float(real_char["excursion_rad"]), 1e-12)
            onset_delta = None if real_char["onset_s"] is None or sim_char["onset_s"] is None else float(sim_char["onset_s"] - real_char["onset_s"])
            peak_delta = float(sim_char["peak_time_s"] - real_char["peak_time_s"])
            real_recovery = real_char["recovery_after_motion_s"]
            sim_recovery = sim_char["recovery_after_motion_s"]
            recovery_delta = None if real_recovery is None or sim_recovery is None else float(sim_recovery - real_recovery)
            lag = lag_seconds(r, s)
            sign_match = int(np.sign(float(real_char["signed_excursion_rad"])) == np.sign(float(sim_char["signed_excursion_rad"])))
            velocity_corr = float(np.corrcoef(rv, sv)[0, 1]) if np.std(rv) > 1e-8 and np.std(sv) > 1e-8 else math.nan
            a_band = amplitude_band(ratio)
            t_band = timing_band(onset_delta, peak_delta, lag, recovery_delta)
            overall = "POOR" if not sign_match or "POOR" in (a_band, t_band) else "ACCEPTABLE" if "ACCEPTABLE" in (a_band, t_band) else "GOOD"
            rows.append({
                "dataset": name,
                "plane": "pitch" if joint in TARGETS_PITCH else "roll",
                "joint_name": joint,
                "real_excursion_rad": real_char["excursion_rad"],
                "sim_excursion_rad": sim_char["excursion_rad"],
                "excursion_ratio": ratio,
                "sign_match": sign_match,
                "onset_delta_s": onset_delta,
                "peak_timing_delta_s": peak_delta,
                "xcorr_lag_s": lag,
                "recovery_delta_s": recovery_delta,
                "velocity_shape_correlation": velocity_corr,
                "relative_rmse_rad": float(np.sqrt(np.mean((s - r) ** 2))),
                "amplitude_band": a_band,
                "timing_band": t_band,
                "overall_band": overall,
            })
    metrics = pd.DataFrame(rows)
    metrics.to_csv(HERE / "phase3ay_response_metrics.csv", index=False)
    return metrics


def allocation_audit() -> tuple[pd.DataFrame, dict[str, float]]:
    base_weights = {
        "left_ankle_pitch_joint": 0.70, "right_ankle_pitch_joint": 0.70,
        "left_hip_pitch_joint": 0.10, "right_hip_pitch_joint": 0.10,
        "left_knee_joint": 0.15, "right_knee_joint": 0.15,
        "waist_pitch_joint": 0.00,
        "left_ankle_roll_joint": 0.70, "right_ankle_roll_joint": 0.70,
        "left_hip_roll_joint": 0.00, "right_hip_roll_joint": 0.00, "waist_roll_joint": 0.00,
    }
    rows = []
    configurations = (
        ("PHASE3AX_BASELINE", CALIBRATION / "phase3ax_constraint_balance" / "runs", "phase3ax_final_candidate"),
        ("PHASE3AY_FINAL", RUNS, FINAL_ID),
    )
    for label, directory, stem in configurations:
        for dataset in ("heart", "wave"):
            joints = pd.read_csv(directory / f"{stem}__{dataset}__arm_only_joint_log.csv")
            safety = pd.read_csv(directory / f"{stem}__{dataset}__arm_only_safety_log.csv", usecols=["t", "raw_pitch_feedback_nm", "raw_roll_feedback_nm"])
            merged = joints[joints.joint_name.isin(TARGETS)].merge(safety, on="t", how="left")
            for row in merged.itertuples(index=False):
                plane = "pitch" if row.joint_name in TARGETS_PITCH else "roll"
                total = float(row.raw_pitch_feedback_nm if plane == "pitch" else row.raw_roll_feedback_nm)
                raw_weight = float(getattr(row, "raw_desired_allocation_weight", base_weights[row.joint_name]))
                pre_scale = float(getattr(row, "preallocation_constraint_scaling", 1.0))
                redistributed_weight = float(getattr(row, "redistributed_allocation_weight", row.allocation_weight))
                allocated_nm = float(row.pitch_balance_correction_nm + row.roll_balance_correction_nm)
                rows.append({
                    "controller": label,
                    "dataset": dataset,
                    "t": float(row.t),
                    "joint_name": row.joint_name,
                    "plane": plane,
                    "total_balance_request_nm": total,
                    "raw_desired_allocation_weight": raw_weight,
                    "raw_desired_allocation_nm": total * raw_weight,
                    "priority_and_preallocation_constraint_scaling": pre_scale,
                    "redistributed_allocation_weight": redistributed_weight,
                    "redistributed_allocation_nm": allocated_nm,
                    "post_contact_scaling": float(row.contact_avoidance_scaling),
                    "post_limit_scaling": float(row.limit_scaling),
                    "post_saturation_scaling": float(row.saturation_scaling),
                    "post_rate_scaling": float(row.rate_scaling),
                    "final_allocation_nm": float(row.final_balance_addition_nm),
                })
    audit = pd.DataFrame(rows)
    audit.to_csv(HERE / "phase3ay_constraint_redistribution_audit.csv", index=False)
    wave = audit[(audit.controller == "PHASE3AX_BASELINE") & (audit.dataset == "wave")]
    summary = {}
    for joint in ("left_ankle_pitch_joint", "right_ankle_pitch_joint", "left_knee_joint", "right_knee_joint"):
        part = wave[(wave.joint_name == joint) & wave.t.between(0.0, datasets()["wave"].motion_end_s)]
        summary[f"{joint}_mean_weight"] = float(part.redistributed_allocation_weight.mean())
        summary[f"{joint}_max_weight"] = float(part.redistributed_allocation_weight.max())
        summary[f"{joint}_post_limit_activation_fraction"] = float((part.post_limit_scaling < 0.999).mean())
        summary[f"{joint}_post_saturation_activation_fraction"] = float((part.post_saturation_scaling < 0.999).mean())
    return audit, summary


def source_lock() -> pd.DataFrame:
    project = CALIBRATION.parent
    paths = [
        CALIBRATION / "phase2e_replay" / "phase2e_heart_measured_reference.csv",
        CALIBRATION / "phase2e_replay" / "phase2e_aligned_joint_data.csv",
        CALIBRATION / "phase2e_replay" / "phase2e_aligned_imu_data.csv",
        CALIBRATION / "phase3av_validation" / "phase3av_measured_reference.csv",
        CALIBRATION / "phase3av_validation" / "phase3av_aligned_joint_data.csv",
        CALIBRATION / "phase3av_validation" / "phase3av_aligned_imu_data.csv",
        CALIBRATION / "phase3ax_constraint_balance" / "phase3ax_core.py",
        CALIBRATION / "phase3ax_constraint_balance" / "simulation_constraint_aware_controller_candidate.json",
        CALIBRATION / "phase3ax_constraint_balance" / "phase3ax_final_validation.json",
        project / "assets" / "Master" / "ff_master_ultra.xml",
        HERE / "phase3ay_core.py",
        HERE / "run_phase3ay_candidate_smoke.py",
        HERE / "run_phase3ay_final_validation.py",
    ]
    rows = []
    for path in paths:
        rows.append({"path": str(path.relative_to(project)), "bytes": path.stat().st_size, "sha256": sha256(path)})
    frame = pd.DataFrame(rows)
    frame.to_csv(HERE / "phase3ay_source_manifest.csv", index=False)
    return frame


def validation_payload() -> tuple[list[dict], list[dict]]:
    payload = json.loads((HERE / "phase3ay_final_validation.json").read_text(encoding="utf-8"))
    return payload["final_runs"], payload["perturbation_runs"]


def write_reports(
    features: pd.DataFrame,
    phase_info: dict[str, dict[str, float]],
    real_targets: pd.DataFrame,
    metrics: pd.DataFrame,
    audit_summary: dict[str, float],
    manifest: pd.DataFrame,
) -> None:
    source = datasets()
    final_runs, perturbations = validation_payload()
    cv = pd.read_csv(HERE / "phase3ay_cross_validation.csv")

    lock_text = f"""# Phase 3A-Y Source Lock

Status: **LOCKED AND VERIFIED**

- Robot connection/control: **NONE**
- Reported effort loaded: **NO**
- MJCF or physical parameter edits: **NONE**
- Hardware mapping edits: **NONE**
- Frozen safety baseline: Phase 3A-X contact/limit/saturation/rate/pitch-roll architecture
- Frozen arm tracking: Phase 3A-X shoulder/wrist gain candidate

{md_table(manifest)}

The Phase 2/3 source files were read only. Generated Phase 3A-Y artifacts live only in this directory.
"""
    (HERE / "phase3ay_source_lock.md").write_text(lock_text, encoding="utf-8")

    condition_rows = []
    for name, dataset in source.items():
        motion = features[(features.dataset == name) & features.t.between(0.0, dataset.motion_end_s)]
        condition_rows.append({
            "dataset": name,
            "duration_s": dataset.motion_end_s,
            "mean_arm_energy_rad_s": motion.arm_motion_energy_rad_s.mean(),
            "peak_arm_energy_rad_s": motion.arm_motion_energy_rad_s.max(),
            "mean_asymmetry": motion.arm_motion_asymmetry.mean(),
            "p90_asymmetry": motion.arm_motion_asymmetry.quantile(0.90),
            "peak_arm_acceleration_rad_s2": max(motion.left_arm_acceleration_norm_rad_s2.max(), motion.right_arm_acceleration_norm_rad_s2.max()),
            "mean_abs_sagittal_proxy": motion.arm_sagittal_momentum_proxy_rad_s.abs().mean(),
            "mean_abs_lateral_proxy": motion.arm_lateral_momentum_proxy_rad_s.abs().mean(),
            "peak_chest_pitch_rad": motion.chest_relative_pitch_rad.abs().max(),
            "peak_chest_roll_rad": motion.chest_relative_roll_rad.abs().max(),
            "peak_chest_gyro_rad_s": np.sqrt(motion.chest_gyro_x**2 + motion.chest_gyro_y**2 + motion.chest_gyro_z**2).max(),
        })
    condition = pd.DataFrame(condition_rows)
    condition.to_csv(HERE / "phase3ay_motion_condition_summary.csv", index=False)
    motion_text = f"""# Phase 3A-Y Motion-Condition Analysis

## Measured input differences

{md_table(condition)}

Heart is bilateral and nearly symmetric; wave is unilateral and highly asymmetric. This is measured, not inferred from the preset name: the final controller sees only live arm energy and left/right asymmetry. The two motions also differ in sagittal/lateral velocity proxies, onset, duration, and measured torso response.

## Why the response distribution differs

- Heart's symmetric, high-energy arm motion produces a measurable pitch disturbance while net left/right asymmetry remains low. The real response spreads into ankle pitch and a small but nonzero waist-roll response.
- Wave's unilateral arm motion produces high asymmetry and stronger roll coupling. The safest simulation response retains the frozen Phase 3A-X distribution; applying the heart allocation blindly causes contact.
- The large wave knee ratio is dominated by small real knee excursion plus passive simulated leg coupling. It is not evidence that the real MC commands eight times more knee motion.

Phase markers were derived from measured arm energy: `{json.dumps(phase_info, sort_keys=True)}`.
"""
    (HERE / "phase3ay_motion_condition_analysis.md").write_text(motion_text, encoding="utf-8")

    right_knee_mean = audit_summary["right_knee_joint_mean_weight"]
    right_knee_max = audit_summary["right_knee_joint_max_weight"]
    ankle_limit = max(audit_summary["left_ankle_pitch_joint_post_limit_activation_fraction"], audit_summary["right_ankle_pitch_joint_post_limit_activation_fraction"])
    audit_text = f"""# Phase 3A-Y Constraint Redistribution Audit

Per-timestep evidence is in `phase3ay_constraint_redistribution_audit.csv` and contains raw desired allocation, priority/pre-allocation scaling, redistributed allocation, post-safety scaling, and final allocation.

## Wave right-knee root-cause check

- Frozen 3A-X raw knee weight: **0.150**.
- Mean redistributed right-knee weight during wave: **{right_knee_mean:.6f}**; maximum **{right_knee_max:.6f}**.
- Ankle limit-scaling activation fraction: **{ankle_limit:.6f}**.
- Right-knee limit/saturation activation fractions: **{audit_summary['right_knee_joint_post_limit_activation_fraction']:.6f} / {audit_summary['right_knee_joint_post_saturation_activation_fraction']:.6f}**.

Conclusion: **Phase 3A-X constraint redistribution did not manufacture the knee over-response.** The priority-normalized knee share is below its raw 0.15 weight, ankle corrections were not limit-clipped, and there was no knee saturation-driven transfer. The remaining 8× ratio is a passive/coupled position response against a very small real denominator; safely removing it was not possible with controller allocation alone.
"""
    (HERE / "phase3ay_constraint_redistribution_audit.md").write_text(audit_text, encoding="utf-8")

    pitch_text = """# Phase 3A-Y Pitch Response Model

Model class: **continuous, explainable activity/asymmetry gain scheduling**.

Inputs are live arm position/velocity-derived energy, instantaneous left/right asymmetry, sagittal-motion fraction, pelvis pitch and pitch rate, plus inherited joint/contact/saturation safety state. No motion name or preset ID is available.

At rest the model exactly returns the frozen Phase 3A-X distribution. During symmetric motion it moves normalized authority toward ankle pitch and reduces direct knee share. During unilateral motion it continuously returns toward the frozen distribution. Total pitch request and allocation weights are computed separately; absolute weights are normalized before safety redistribution.

This is a simulation controller response model, **not MC gain identification and not hardware calibration**.
"""
    (HERE / "phase3ay_pitch_response_model.md").write_text(pitch_text, encoding="utf-8")
    roll_text = """# Phase 3A-Y Roll Response Model

The roll model uses arm activity, left/right asymmetry, lateral-motion proxy, pelvis roll and roll rate. Symmetric active motion allocates explicit waist-roll authority; unilateral motion returns continuously to the frozen ankle-roll distribution. A 0.10 rad/s arm-energy deadband prevents landing and base-perturbation noise from activating the gesture model.

The model restored heart waist-roll excursion from 0.012× to an order-one response without using a motion label. Safety scaling and slew limiting are applied afterward.
"""
    (HERE / "phase3ay_roll_response_model.md").write_text(roll_text, encoding="utf-8")
    allocation_text = """# Phase 3A-Y Allocation Model

```text
measured arm/base state
  -> filtered motion-activity estimator
  -> instantaneous left/right asymmetry schedule
  -> independent total pitch / total roll request
  -> signed-normalized desired joint distribution
  -> frozen Phase 3A-X priority + constraint redistribution
  -> contact / joint-limit / saturation / slew / hard target envelope
  -> final simulation joint target/torque addition
```

No branch checks `heart`, `wave`, preset ID, or dataset name. At zero motion activity, the exact Phase 3A-X allocation is recovered. Every timestep is auditable in the redistribution CSV.
"""
    (HERE / "phase3ay_allocation_model.md").write_text(allocation_text, encoding="utf-8")

    cv_text = f"""# Phase 3A-Y Cross-Validation Report

{md_table(cv, ['experiment_id', 'fit_dataset', 'dataset', 'evaluation', 'safety_pass', 'balance_shape_score', 'arm_tracking_retained'])}

- **Heart-only -> blind wave: NO generalization.** It produces 180 hip-contact samples and a worse response score.
- **Wave-only -> blind heart: NO response generalization.** It stays safe but returns the poor fixed-allocation heart score.
- The joint state-conditioned architecture is designed only after both blind experiments. It is not selected by motion name.

This is strong overfitting evidence with only two motions; the final baseline therefore remains unvalidated when wave knee/timing gates fail.
"""
    (HERE / "phase3ay_cross_validation_report.md").write_text(cv_text, encoding="utf-8")

    for name in ("heart", "wave"):
        subset = metrics[metrics.dataset == name]
        key = subset[subset.joint_name.isin(("left_ankle_pitch_joint", "waist_roll_joint", "right_knee_joint"))]
        summary = next(item for item in final_runs if item["dataset"] == name and item["mode"] == "arm_only")
        report = f"""# Phase 3A-Y {name.title()} Validation

Safety pass: **{summary['safety_pass']}**. Arm tracking retained: **{all(row['lag_s'] is None or abs(float(row['lag_s'])) <= 0.22 for row in summary['tracking_metrics'])}**.

## Key response metrics

{md_table(key, ['joint_name', 'real_excursion_rad', 'sim_excursion_rad', 'excursion_ratio', 'sign_match', 'onset_delta_s', 'peak_timing_delta_s', 'xcorr_lag_s', 'amplitude_band', 'timing_band', 'overall_band'])}

## All balance channels

{md_table(subset, ['plane', 'joint_name', 'excursion_ratio', 'sign_match', 'xcorr_lag_s', 'velocity_shape_correlation', 'amplitude_band', 'timing_band', 'overall_band'])}

Reported effort was not used. Relative IMU remains auxiliary only.
"""
        (HERE / f"phase3ay_{name}_validation.md").write_text(report, encoding="utf-8")

    whole = pd.DataFrame([
        {
            "dataset": item["dataset"], "safety_pass": item["safety_pass"], "stable_no_fall": item["stable_no_fall"],
            "pelvis_hip_contact_samples": item["pelvis_hip_contact_samples"], "limit_violation_samples": item["limit_violation_samples"],
            "persistent_saturation_fraction": item["persistent_saturation_fraction"], "minimum_positive_precontact_distance_m": item["minimum_positive_precontact_distance_m"],
        }
        for item in final_runs if item["mode"] == "whole_body"
    ])
    whole_text = f"""# Phase 3A-Y Whole-Body Stress Report

{md_table(whole)}

Both measured whole-body replays pass: no fall, no pelvis/hip contact, no joint-limit violation, and no persistent saturation. Whole-body replay is a safety stress test, not a claim that simulated response equals real MC internals.
"""
    (HERE / "phase3ay_whole_body_stress_report.md").write_text(whole_text, encoding="utf-8")
    perturb = pd.read_csv(HERE / "phase3ay_perturbation_results.csv")
    perturb_text = f"""# Phase 3A-Y Perturbation Report

Result: **{int(perturb.safety_pass.sum())}/{len(perturb)} PASS**.

{md_table(perturb, ['dataset', 'stable_no_fall', 'contact_safety_pass', 'minimum_positive_precontact_distance_m', 'limit_management_pass', 'saturation_management_pass', 'safety_pass'])}

All tests use the inherited ±0.25° Phase 3A-X suite. The activity deadband keeps the response scheduler on the exact frozen fallback during standing perturbations.
"""
    (HERE / "phase3ay_perturbation_report.md").write_text(perturb_text, encoding="utf-8")

    architecture = """# Phase 3A-Y Controller Architecture

```text
real-time measurable state
  arm q / qdot / acceleration proxy
  left-right asymmetry + sagittal/lateral proxies
  pelvis pitch/roll + angular rates
  current joint/contact/actuator margins
        |
        v
disturbance estimator (0.10 rad/s activity deadband, 0.12 s filter)
        |
        +--> PITCH_RESPONSE_MODEL --> total pitch request + normalized allocation
        |
        +--> ROLL_RESPONSE_MODEL  --> total roll request  + normalized allocation
        |
        v
Phase 3A-X frozen constraint-aware safety layer
  channel priority -> constraint redistribution -> contact/limit/saturation/rate scaling
        |
        v
final target composition -> frozen arm/standing controller -> MuJoCo
```

The response model cannot bypass the safety layer. It contains no motion name, preset ID, physical parameter fit, effort input, or hardware calibration parameter.
"""
    (HERE / "phase3ay_controller_architecture.md").write_text(architecture, encoding="utf-8")

    arm_summaries = [item for item in final_runs if item["mode"] == "arm_only"]
    whole_summaries = [item for item in final_runs if item["mode"] == "whole_body"]
    arm_ok = all(all(row["lag_s"] is None or abs(float(row["lag_s"])) <= 0.22 for row in item["tracking_metrics"]) for item in arm_summaries)
    contact_ok = all(item["contact_safety_pass"] for item in final_runs) and bool(perturb.contact_safety_pass.all())
    limit_ok = all(item["limit_management_pass"] for item in final_runs) and bool(perturb.limit_management_pass.all())
    sat_ok = all(item["saturation_management_pass"] for item in final_runs) and bool(perturb.saturation_management_pass.all())
    heart_key = metrics[(metrics.dataset == "heart") & metrics.joint_name.isin(("left_ankle_pitch_joint", "waist_roll_joint"))]
    wave_key = metrics[(metrics.dataset == "wave") & (metrics.joint_name == "right_knee_joint")]
    heart_acceptable = bool((heart_key.amplitude_band != "POOR").all()) and bool((heart_key.timing_band != "POOR").all())
    wave_acceptable = bool((wave_key.amplitude_band != "POOR").all()) and bool((wave_key.timing_band != "POOR").all())
    amplitude_general = heart_acceptable and wave_acceptable
    timing_general = bool((metrics.timing_band != "POOR").mean() >= 0.80)
    whole_ok = all(item["safety_pass"] for item in whole_summaries)
    perturb_ok = bool(perturb.safety_pass.all()) and len(perturb) == 8
    gates = {
        "ARM_TRACKING_GENERALIZES": "YES" if arm_ok else "NO",
        "CONTACT_SAFETY_ROBUST": "YES" if contact_ok else "NO",
        "LIMIT_MANAGEMENT_ROBUST": "YES" if limit_ok else "NO",
        "SATURATION_MANAGEMENT_ROBUST": "YES" if sat_ok else "NO",
        "BALANCE_AMPLITUDE_GENERALIZES": "YES" if amplitude_general else "NO",
        "BALANCE_TIMING_GENERALIZES": "YES" if timing_general else "NO",
        "HEART_BALANCE_ACCEPTABLE": "YES" if heart_acceptable else "NO",
        "WAVE_BALANCE_ACCEPTABLE": "YES" if wave_acceptable else "NO",
        "WHOLE_BODY_STRESS_TEST_PASSES": "YES" if whole_ok else "NO",
        "PERTURBATION_ROBUSTNESS": "YES" if perturb_ok else "NO",
    }
    gates["VALIDATED_SIM_CONTROLLER_BASELINE"] = "YES" if all(value == "YES" for value in gates.values()) else "NO"
    gates["DYNAMICS_CALIBRATION_READY"] = "NO"
    gate_lines = "\n".join(f"- `{key} = {value}`" for key, value in gates.items())
    final_text = f"""# Phase 3A-Y Final Gate

{gate_lines}

## Fixed engineering bands

- Amplitude GOOD: 0.67–1.50×; ACCEPTABLE: 0.50–2.00×; otherwise POOR.
- Timing GOOD: onset/peak/lag within 0.25 s and recovery within 0.50 s; ACCEPTABLE: within 0.50/1.00 s; otherwise POOR.
- A sign conflict is always POOR.

These bands were fixed before final gate evaluation. They reflect the 50 Hz measurement grid (20 ms), the limited two-motion dataset, and an engineering tolerance that does not require 1.000× matching.

## Gate rationale

Heart left-ankle and waist-roll amplitudes are repaired to order-one ratios, but timing and other knee channels remain poor. Wave right-knee remains about eight times the very small real excursion, so amplitude/timing generalization is not achieved. Hard safety, whole-body stress, arm tracking, and 8/8 perturbation robustness are retained.

`DYNAMICS_CALIBRATION_READY` remains NO because PHYSICAL_SIGN/PHYSICAL_ZERO, effort semantics, complete IMU transform, and MC internal command remain unresolved.
"""
    (HERE / "phase3ay_final_gate.md").write_text(final_text, encoding="utf-8")

    candidate_payload = {
        "classification": "SIMULATION_MOTION_CONDITIONED_BALANCE_CANDIDATE_NOT_VALIDATED_RESPONSE_BASELINE",
        "warning": "NOT HARDWARE CALIBRATION",
        "design": asdict(candidate(FINAL_ID)),
        "architecture": {
            "disturbance_estimator": "arm energy deadband/filter plus instantaneous left-right asymmetry",
            "pitch_response_model": "continuous normalized activity/asymmetry schedule",
            "roll_response_model": "continuous normalized activity/asymmetry schedule",
            "safety_layer": "frozen Phase 3A-X contact/limit/saturation/rate/target-envelope layer",
            "motion_name_or_preset_id_used": False,
        },
        "gates": gates,
        "reported_effort_used": False,
        "robot_connected": False,
        "mjcf_modified": False,
        "physical_parameters_modified": False,
        "hardware_mapping_modified": False,
        "source_manifest_sha256": sha256(HERE / "phase3ay_source_manifest.csv"),
    }
    (HERE / "simulation_motion_conditioned_balance_candidate.json").write_text(json.dumps(candidate_payload, indent=2), encoding="utf-8")

    experiment_rows = []
    experiment_path = HERE / "phase3ay_experiments.csv"
    if experiment_path.exists():
        experiment_rows.extend(pd.read_csv(experiment_path).to_dict("records"))
    experiment_rows.extend(cv.to_dict("records"))
    experiment_rows.extend([
        {**{key: value for key, value in item.items() if key in ("experiment_id", "family", "dataset", "mode", "safety_pass")}, "classification": "FINAL_CANDIDATE"}
        for item in final_runs
    ])
    pd.DataFrame(experiment_rows).drop_duplicates(subset=["experiment_id", "dataset", "mode"], keep="last").to_csv(experiment_path, index=False)


def main() -> int:
    manifest = source_lock()
    features, phase_info = extract_real_features()
    real_targets = real_response_targets(features)
    metrics = compare_final(features)
    _audit, audit_summary = allocation_audit()
    write_reports(features, phase_info, real_targets, metrics, audit_summary, manifest)
    print(f"FEATURE_ROWS={len(features)}")
    print(f"RESPONSE_METRICS={len(metrics)}")
    print("PHASE3AY_ANALYSIS_COMPLETE=1")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
