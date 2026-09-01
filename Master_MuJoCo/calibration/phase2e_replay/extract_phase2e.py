#!/usr/bin/env python3
"""Extract the accepted Phase 2D measured heart trajectory (offline only)."""

from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation, Slerp


HERE = Path(__file__).resolve().parent
CALIBRATION = HERE.parent
PROJECT = CALIBRATION.parent
INPUT = CALIBRATION / "logs" / "real" / "phase2d_heart_001"
PLOTS = HERE / "plots" / "real_joint_trajectories"
RATE_HZ = 50.0

RAW_FILES = [
    "raw_arm.csv",
    "raw_leg.csv",
    "raw_waist.csv",
    "raw_head.csv",
    "raw_chest_imu.csv",
    "raw_torso_imu.csv",
    "capture_metadata.json",
    "events.csv",
    "raw_serialized_evidence.txt",
]
JOINT_FILES = ["raw_arm.csv", "raw_leg.csv", "raw_waist.csv", "raw_head.csv"]
IMU_FILES = {"chest": "raw_chest_imu.csv", "torso": "raw_torso_imu.csv"}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def md_table(headers: list[str], rows: list[list[object]]) -> str:
    def cell(value: object) -> str:
        return str(value).replace("|", "\\|").replace("\n", " ")

    lines = [
        "| " + " | ".join(cell(value) for value in headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    lines.extend("| " + " | ".join(cell(value) for value in row) + " |" for row in rows)
    return "\n".join(lines)


def fmt(value: object, digits: int = 5) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "UNKNOWN"
    return f"{number:.{digits}f}" if math.isfinite(number) else "UNKNOWN"


def interpolate_series(times: np.ndarray, values: np.ndarray, timeline: np.ndarray) -> np.ndarray:
    mask = np.isfinite(times) & np.isfinite(values)
    times = times[mask]
    values = values[mask]
    order = np.argsort(times)
    times = times[order]
    values = values[order]
    unique, indices = np.unique(times, return_index=True)
    return np.interp(timeline, unique, values[indices])


def quaternion_aligned(frame: pd.DataFrame, timeline: np.ndarray, motion_start: float) -> pd.DataFrame:
    source_t = frame.elapsed_seconds.to_numpy(float)
    order = np.argsort(source_t)
    source_t = source_t[order]
    quaternions = frame[["orientation_x", "orientation_y", "orientation_z", "orientation_w"]].to_numpy(float)[order]
    valid = np.isfinite(quaternions).all(axis=1) & (np.linalg.norm(quaternions, axis=1) > 0.5)
    source_t = source_t[valid]
    quaternions = quaternions[valid]
    unique_t, unique_idx = np.unique(source_t, return_index=True)
    rotations = Rotation.from_quat(quaternions[unique_idx])
    aligned_rotations = Slerp(unique_t, rotations)(timeline)
    relative_t = timeline - motion_start
    pre = (relative_t >= -3.0) & (relative_t <= -0.2)
    baseline = aligned_rotations[pre].mean() if pre.any() else aligned_rotations[0]
    relative = baseline.inv() * aligned_rotations
    relative_rpy = relative.as_euler("xyz", degrees=False)
    absolute_quat = aligned_rotations.as_quat()
    result = pd.DataFrame(
        {
            "elapsed_seconds": timeline,
            "t": relative_t,
            "orientation_x": absolute_quat[:, 0],
            "orientation_y": absolute_quat[:, 1],
            "orientation_z": absolute_quat[:, 2],
            "orientation_w": absolute_quat[:, 3],
            "relative_roll_rad": relative_rpy[:, 0],
            "relative_pitch_rad": relative_rpy[:, 1],
            "relative_yaw_rad": relative_rpy[:, 2],
        }
    )
    for source, target in (
        ("gyro_x", "gyro_x"),
        ("gyro_y", "gyro_y"),
        ("gyro_z", "gyro_z"),
        ("accel_x", "accel_x"),
        ("accel_y", "accel_y"),
        ("accel_z", "accel_z"),
    ):
        result[target] = interpolate_series(
            frame.elapsed_seconds.to_numpy(float), frame[source].to_numpy(float), timeline
        )
    result["gyro_norm"] = np.linalg.norm(result[["gyro_x", "gyro_y", "gyro_z"]], axis=1)
    result["accel_norm"] = np.linalg.norm(result[["accel_x", "accel_y", "accel_z"]], axis=1)
    return result


def recovery_time(t: np.ndarray, position: np.ndarray, velocity: np.ndarray, motion_end: float, baseline: float, baseline_std: float) -> float | None:
    threshold = max(0.003, 3.0 * baseline_std)
    mask = (np.abs(position - baseline) <= threshold) & (np.abs(velocity) <= 0.05) & (t >= motion_end)
    samples = max(1, round(0.5 * RATE_HZ))
    run = 0
    for index, good in enumerate(mask):
        run = run + 1 if good else 0
        if run >= samples:
            return float(t[index - samples + 1] - motion_end)
    return None


def main() -> int:
    HERE.mkdir(parents=True, exist_ok=True)
    PLOTS.mkdir(parents=True, exist_ok=True)

    metadata = json.loads((INPUT / "capture_metadata.json").read_text(encoding="utf-8"))
    if not metadata.get("PHASE2D_REPLAY_READY"):
        raise SystemExit("Phase 2D capture is not replay-ready")
    detection = metadata["event_detection"]
    motion_start = float(detection["heart_start_elapsed_seconds"])
    motion_end = float(detection["heart_end_elapsed_seconds"])
    motion_duration = motion_end - motion_start

    manifest_rows = []
    for name in RAW_FILES:
        path = INPUT / name
        if not path.exists():
            raise FileNotFoundError(path)
        manifest_rows.append(
            {
                "relative_path": str(path.relative_to(PROJECT)),
                "size_bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
        )
    pd.DataFrame(manifest_rows).to_csv(HERE / "source_sha256_manifest.csv", index=False)

    joint_frames = []
    coverage = []
    for name in JOINT_FILES:
        frame = pd.read_csv(INPUT / name)
        frame["source_file"] = name
        joint_frames.append(frame)
        coverage.append((float(frame.elapsed_seconds.min()), float(frame.elapsed_seconds.max())))
    joints_raw = pd.concat(joint_frames, ignore_index=True)
    imu_raw = {}
    for label, name in IMU_FILES.items():
        frame = pd.read_csv(INPUT / name)
        imu_raw[label] = frame
        coverage.append((float(frame.elapsed_seconds.min()), float(frame.elapsed_seconds.max())))

    common_start = max(value[0] for value in coverage)
    common_end = min(value[1] for value in coverage)
    period = 1.0 / RATE_HZ
    timeline = np.arange(math.ceil(common_start * RATE_HZ) / RATE_HZ, common_end + 1e-9, period)
    relative_t = timeline - motion_start

    aligned_rows = []
    for joint_name, frame in joints_raw.groupby("joint_name", sort=False):
        frame = frame.sort_values("elapsed_seconds")
        for field in ("position", "velocity", "effort"):
            if field not in frame:
                frame[field] = np.nan
        position = interpolate_series(frame.elapsed_seconds.to_numpy(float), frame.position.to_numpy(float), timeline)
        velocity = interpolate_series(frame.elapsed_seconds.to_numpy(float), frame.velocity.to_numpy(float), timeline)
        effort = interpolate_series(frame.elapsed_seconds.to_numpy(float), frame.effort.to_numpy(float), timeline)
        aligned_rows.append(
            pd.DataFrame(
                {
                    "elapsed_seconds": timeline,
                    "t": relative_t,
                    "joint_name": joint_name,
                    "position": position,
                    "velocity": velocity,
                    "reported_effort": effort,
                }
            )
        )
    aligned = pd.concat(aligned_rows, ignore_index=True)
    aligned.to_csv(HERE / "phase2e_aligned_joint_data.csv", index=False)
    aligned[["t", "joint_name", "position", "velocity"]].to_csv(
        HERE / "phase2e_heart_measured_reference.csv", index=False
    )

    imu_frames = []
    for label, frame in imu_raw.items():
        converted = quaternion_aligned(frame, timeline, motion_start)
        converted.insert(2, "imu", label)
        imu_frames.append(converted)
    imu_aligned = pd.concat(imu_frames, ignore_index=True)
    imu_aligned.to_csv(HERE / "phase2e_aligned_imu_data.csv", index=False)

    motion = aligned[(aligned.t >= 0) & (aligned.t <= motion_duration)]
    pre = aligned[(aligned.t >= -3.0) & (aligned.t <= -0.2)]
    post = aligned[(aligned.t >= motion_duration + 1.0) & (aligned.t <= motion_duration + 3.0)]
    preliminary_excursion = motion.groupby("joint_name").position.agg(lambda value: float(value.max() - value.min()))
    primary_seed = [
        name for name, excursion in preliminary_excursion.items()
        if (name.startswith("left_") or name.startswith("right_"))
        and any(token in name for token in ("shoulder", "elbow", "wrist"))
        and excursion >= 0.75
    ]
    arm_activity = (
        motion[motion.joint_name.isin(primary_seed)]
        .pivot(index="t", columns="joint_name", values="velocity")
        .abs()
        .mean(axis=1)
    )

    metrics_rows = []
    for joint_name, group in aligned.groupby("joint_name", sort=False):
        group = group.sort_values("t")
        gm = group[(group.t >= 0) & (group.t <= motion_duration)]
        gp = pre[pre.joint_name == joint_name]
        ga = post[post.joint_name == joint_name]
        q_initial = float(gp.position.mean())
        baseline_std = float(gp.position.std(ddof=0))
        q_min = float(gm.position.min())
        q_max = float(gm.position.max())
        excursion = q_max - q_min
        peak_velocity = float(gm.velocity.abs().max())
        baseline_effort = float(gp.reported_effort.mean())
        peak_effort = float(gm.reported_effort.abs().max())
        velocity_noise = float(gp.velocity.abs().quantile(0.99))
        position_threshold = max(0.002, 0.03 * excursion, 3.0 * baseline_std)
        velocity_threshold = max(0.03, velocity_noise + 0.02)
        active = (np.abs(gm.position.to_numpy(float) - q_initial) > position_threshold) | (
            np.abs(gm.velocity.to_numpy(float)) > velocity_threshold
        )
        active_indices = np.flatnonzero(active)
        onset = float(gm.t.iloc[active_indices[0]]) if active_indices.size else np.nan
        end = float(gm.t.iloc[active_indices[-1]]) if active_indices.size else np.nan
        abs_velocity = gm.set_index("t").velocity.abs().reindex(arm_activity.index).interpolate().bfill().ffill()
        correlation = float(abs_velocity.corr(arm_activity)) if abs_velocity.std() > 1e-8 else np.nan
        peak_correlation = np.nan
        peak_correlation_lag = np.nan
        if abs_velocity.std() > 1e-8 and arm_activity.std() > 1e-8:
            x = abs_velocity.to_numpy(float)
            reference_activity = arm_activity.to_numpy(float)
            candidates = []
            for lag_samples in range(-round(RATE_HZ), round(RATE_HZ) + 1):
                if lag_samples > 0:
                    reference_slice, joint_slice = reference_activity[:-lag_samples], x[lag_samples:]
                elif lag_samples < 0:
                    reference_slice, joint_slice = reference_activity[-lag_samples:], x[:lag_samples]
                else:
                    reference_slice, joint_slice = reference_activity, x
                if len(reference_slice) >= 10:
                    candidates.append((float(np.corrcoef(reference_slice, joint_slice)[0, 1]), lag_samples / RATE_HZ))
            peak_correlation, peak_correlation_lag = max(candidates, key=lambda item: item[0])
        final_position = float(ga.position.mean()) if not ga.empty else float(group.position.iloc[-1])
        recovery = recovery_time(
            group.t.to_numpy(float), group.position.to_numpy(float), group.velocity.to_numpy(float),
            motion_duration, q_initial, baseline_std,
        )

        is_arm = any(token in joint_name for token in ("shoulder", "elbow", "wrist"))
        if is_arm and (excursion >= 0.75 or peak_velocity >= 1.0):
            classification = "GESTURE_PRIMARY"
        elif is_arm and excursion >= 0.05:
            classification = "GESTURE_SECONDARY"
        elif not is_arm and excursion >= 0.005 and peak_velocity >= 0.04 and (
            not math.isfinite(peak_correlation) or peak_correlation >= 0.15
        ):
            classification = "BALANCE_COMPENSATION_CANDIDATE"
        elif excursion < 0.005 and peak_velocity < 0.05:
            classification = "STATIC"
        else:
            classification = "UNKNOWN"

        metrics_rows.append(
            {
                "joint_name": joint_name,
                "q_initial": q_initial,
                "q_min": q_min,
                "q_max": q_max,
                "excursion": excursion,
                "peak_abs_dq": peak_velocity,
                "reported_effort_baseline": baseline_effort,
                "peak_abs_reported_effort": peak_effort,
                "motion_onset": onset,
                "motion_end": end,
                "final_minus_initial": final_position - q_initial,
                "position_baseline_std": baseline_std,
                "arm_activity_correlation": correlation,
                "arm_activity_peak_lagged_correlation": peak_correlation,
                "arm_activity_peak_correlation_lag_s": peak_correlation_lag,
                "recovery_after_global_motion_s": recovery,
                "classification": classification,
                "classification_basis": (
                    f"excursion={excursion:.6f} rad; peak|dq|={peak_velocity:.6f} rad/s; "
                    f"peak lagged arm activity correlation={peak_correlation:.4f} at {peak_correlation_lag:+.3f} s" if math.isfinite(peak_correlation)
                    else f"excursion={excursion:.6f} rad; peak|dq|={peak_velocity:.6f} rad/s; correlation unavailable"
                ),
            }
        )
    metrics = pd.DataFrame(metrics_rows).sort_values("joint_name")
    metrics.to_csv(HERE / "phase2e_joint_metrics.csv", index=False)

    symmetry_rows = []
    arm_pairs = sorted(
        name.removeprefix("left_") for name in metrics.joint_name
        if name.startswith("left_") and any(token in name for token in ("shoulder", "elbow", "wrist"))
    )
    motion_pivot = motion.pivot(index="t", columns="joint_name", values="position")
    for suffix in arm_pairs:
        left_name = "left_" + suffix
        right_name = "right_" + suffix
        if right_name not in motion_pivot:
            continue
        left = motion_pivot[left_name].to_numpy(float)
        right = motion_pivot[right_name].to_numpy(float)
        left_delta = left - left[0]
        right_delta = right - right[0]
        left_exc = float(np.ptp(left_delta))
        right_exc = float(np.ptp(right_delta))
        if min(left_exc, right_exc) < 0.01:
            relation = "INSUFFICIENT_EVIDENCE"
            same_corr = mirrored_corr = np.nan
        else:
            same_corr = float(np.corrcoef(left_delta, right_delta)[0, 1])
            mirrored_corr = float(np.corrcoef(left_delta, -right_delta)[0, 1])
            amplitude_ratio = min(left_exc, right_exc) / max(left_exc, right_exc)
            if mirrored_corr >= 0.90 and amplitude_ratio >= 0.80:
                relation = "MIRRORED"
            elif same_corr >= 0.90 and amplitude_ratio >= 0.80:
                relation = "SAME_SIGN"
            else:
                relation = "ASYMMETRIC"
        symmetry_rows.append(
            {
                "joint_pair": suffix,
                "left_excursion_rad": left_exc,
                "right_excursion_rad": right_exc,
                "correlation_same_sign": same_corr,
                "correlation_mirrored": mirrored_corr,
                "rms_q_left_plus_q_right": float(np.sqrt(np.mean((left + right) ** 2))),
                "rms_q_left_minus_q_right": float(np.sqrt(np.mean((left - right) ** 2))),
                "classification": relation,
            }
        )
    symmetry = pd.DataFrame(symmetry_rows)
    symmetry.to_csv(HERE / "phase2e_left_right_symmetry.csv", index=False)

    lock = {
        "input_capture": str(INPUT.relative_to(PROJECT)),
        "PHASE2D_REPLAY_READY": True,
        "motion_start_elapsed_seconds": motion_start,
        "motion_end_elapsed_seconds": motion_end,
        "motion_duration_seconds": motion_duration,
        "common_start_elapsed_seconds": common_start,
        "common_end_elapsed_seconds": common_end,
        "relative_t_start_seconds": common_start - motion_start,
        "relative_t_end_seconds": common_end - motion_start,
        "target_rate_hz": RATE_HZ,
        "joint_interpolation": "piecewise linear on receive-monotonic aligned elapsed_seconds",
        "imu_orientation_interpolation": "quaternion Slerp on receive-monotonic aligned elapsed_seconds",
        "raw_samples_modified": False,
        "source_manifest": "source_sha256_manifest.csv",
    }
    (HERE / "source_data_lock.json").write_text(json.dumps(lock, indent=2), encoding="utf-8")

    classification_rows = [
        [row.joint_name, row.classification, fmt(row.excursion), fmt(row.peak_abs_dq), fmt(row.arm_activity_peak_lagged_correlation, 3), fmt(row.arm_activity_peak_correlation_lag_s, 3), row.classification_basis]
        for row in metrics.itertuples()
    ]
    head_pitch = metrics[metrics.joint_name == "head_pitch_joint"].iloc[0]
    classification_report = f"""# Phase 2E joint classification

Classification is derived from measured excursion, velocity, timing, correlation with the dominant arm-gesture activity, and return behavior. It is not based on body group alone.

{md_table(['joint', 'classification', 'excursion rad', 'peak |dq| rad/s', 'peak lagged corr', 'lag s', 'basis'], classification_rows)}

## Head pitch

Real `head_pitch_joint` excursion is {head_pitch.excursion:.8f} rad with peak |dq| {head_pitch.peak_abs_dq:.8f} rad/s. The current fixed MuJoCo treatment is therefore retained and classified `NO_MATERIAL_DOF_MISMATCH_FOR_THIS_MOTION`; the physical joint's existence is still a structural model difference outside this static trajectory.
"""
    (HERE / "phase2e_joint_classification.md").write_text(classification_report, encoding="utf-8")

    symmetry_report = f"""# Phase 2E left/right measured-coordinate symmetry

All relationships below describe real measured coordinates only. They do not update hardware-to-MuJoCo sign, zero, or encoder offset.

{md_table(['pair', 'left excursion rad', 'right excursion rad', 'same corr', 'mirrored corr', 'RMS(left+right)', 'RMS(left-right)', 'classification'], [[row.joint_pair, fmt(row.left_excursion_rad), fmt(row.right_excursion_rad), fmt(row.correlation_same_sign, 3), fmt(row.correlation_mirrored, 3), fmt(row.rms_q_left_plus_q_right), fmt(row.rms_q_left_minus_q_right), row.classification] for row in symmetry.itertuples()])}
"""
    (HERE / "phase2e_left_right_symmetry.md").write_text(symmetry_report, encoding="utf-8")

    balance_metrics = metrics[
        metrics.joint_name.str.contains("hip|knee|ankle|waist", regex=True)
    ]
    imu_rows = []
    for imu_name, group in imu_aligned.groupby("imu"):
        gm = group[(group.t >= 0) & (group.t <= motion_duration)]
        imu_rows.append(
            [
                imu_name,
                fmt(np.rad2deg(np.max(np.abs(gm.relative_roll_rad))), 3),
                fmt(np.rad2deg(np.max(np.abs(gm.relative_pitch_rad))), 3),
                fmt(gm.gyro_norm.max(), 4),
                fmt(gm.accel_norm.max(), 4),
            ]
        )
    balance_report = f"""# Phase 2E measured balance-response candidates

The dominant arm motion is accompanied by the following leg/waist motion. `BALANCE_COMPENSATION_CANDIDATE` means timing and magnitude are consistent with compensation; it does **not** identify or reconstruct the MC control law.

{md_table(['joint', 'classification', 'excursion rad', 'peak |dq| rad/s', 'onset s', 'end s', 'recovery s', 'peak lagged corr', 'lag s'], [[row.joint_name, row.classification, fmt(row.excursion), fmt(row.peak_abs_dq), fmt(row.motion_onset, 3), fmt(row.motion_end, 3), fmt(row.recovery_after_global_motion_s, 3), fmt(row.arm_activity_peak_lagged_correlation, 3), fmt(row.arm_activity_peak_correlation_lag_s, 3)] for row in balance_metrics.itertuples()])}

## Relative IMU response

{md_table(['IMU', 'peak |relative roll| deg', 'peak |relative pitch| deg', 'peak gyro norm rad/s', 'peak accel norm m/s²'], imu_rows)}

The IMU frame relationship remains `UNKNOWN`; only relative changes, norms, shapes, and timing are used. No absolute quaternion component comparison or MC-law recovery is claimed.
"""
    (HERE / "phase2e_balance_response_report.md").write_text(balance_report, encoding="utf-8")

    groups = {
        "arms": metrics[metrics.joint_name.str.contains("shoulder|elbow|wrist")].joint_name.tolist(),
        "legs": metrics[metrics.joint_name.str.contains("hip|knee|ankle")].joint_name.tolist(),
        "waist_head": metrics[metrics.joint_name.str.contains("waist|head")].joint_name.tolist(),
    }
    for label, names in groups.items():
        fig, axis = plt.subplots(figsize=(14, 7))
        for joint_name in names:
            group = aligned[aligned.joint_name == joint_name]
            axis.plot(group.t, group.position, linewidth=1, label=joint_name)
        axis.axvline(0, color="black", linestyle="--")
        axis.axvline(motion_duration, color="black", linestyle=":")
        axis.set(title=f"Real measured {label} trajectory", xlabel="t from detected motion start (s)", ylabel="position (rad)")
        axis.grid(alpha=0.25)
        axis.legend(ncol=3, fontsize=7)
        fig.tight_layout()
        fig.savefig(PLOTS / f"{label}.png", dpi=150)
        plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(14, 9), sharex=True)
    for imu_name, group in imu_aligned.groupby("imu"):
        axes[0].plot(group.t, np.rad2deg(group.relative_roll_rad), label=f"{imu_name} roll")
        axes[0].plot(group.t, np.rad2deg(group.relative_pitch_rad), linestyle="--", label=f"{imu_name} pitch")
        axes[1].plot(group.t, group.gyro_norm, label=f"{imu_name} gyro norm")
    for axis in axes:
        axis.axvline(0, color="black", linestyle="--")
        axis.axvline(motion_duration, color="black", linestyle=":")
        axis.grid(alpha=0.25)
        axis.legend()
    axes[0].set_ylabel("relative angle (deg)")
    axes[1].set_ylabel("gyro norm (rad/s)")
    axes[1].set_xlabel("t from detected motion start (s)")
    fig.tight_layout()
    fig.savefig(PLOTS / "imu_relative_response.png", dpi=150)
    plt.close(fig)

    print(json.dumps({
        "joints": len(metrics),
        "motion_start": motion_start,
        "motion_end": motion_end,
        "common_t": [common_start - motion_start, common_end - motion_start],
        "class_counts": metrics.classification.value_counts().to_dict(),
        "primary_seed": primary_seed,
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
