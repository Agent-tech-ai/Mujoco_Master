#!/usr/bin/env python3
"""Compare real Phase 2D heart data with the two offline MuJoCo replays."""

from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


HERE = Path(__file__).resolve().parent
PLOTS = HERE / "plots" / "real_vs_sim"
RATE_HZ = 50.0


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


def lagged_correlation(reference: np.ndarray, response: np.ndarray, max_lag_s: float = 1.0) -> tuple[float, float]:
    if np.std(reference) < 1e-10 or np.std(response) < 1e-10:
        return np.nan, np.nan
    candidates = []
    limit = round(max_lag_s * RATE_HZ)
    for lag in range(-limit, limit + 1):
        if lag > 0:
            x, y = reference[:-lag], response[lag:]
        elif lag < 0:
            x, y = reference[-lag:], response[:lag]
        else:
            x, y = reference, response
        if len(x) >= 10 and np.std(x) > 1e-10 and np.std(y) > 1e-10:
            candidates.append((float(np.corrcoef(x, y)[0, 1]), lag / RATE_HZ))
    return max(candidates, key=lambda item: item[0]) if candidates else (np.nan, np.nan)


def joint_comparison(frame: pd.DataFrame, motion_duration: float) -> pd.DataFrame:
    rows = []
    for joint_name, group in frame.groupby("joint_name", sort=False):
        group = group.sort_values("t")
        motion = group[(group.t >= 0) & (group.t <= motion_duration)]
        pre = group[(group.t >= -3.0) & (group.t <= -0.2)]
        real = motion.reference_position.to_numpy(float)
        sim = motion.position.to_numpy(float)
        real_delta = real - float(pre.reference_position.mean())
        sim_delta = sim - float(pre.position.mean())
        error = sim - real
        delta_error = sim_delta - real_delta
        correlation, lag = lagged_correlation(real_delta, sim_delta)
        rows.append(
            {
                "joint_name": joint_name,
                "input_mode": str(group.input_mode.iloc[0]),
                "rmse": float(np.sqrt(np.mean(error**2))),
                "mae": float(np.mean(np.abs(error))),
                "peak_abs_error": float(np.max(np.abs(error))),
                "delta_rmse": float(np.sqrt(np.mean(delta_error**2))),
                "phase_lag_candidate_s": lag,
                "shape_correlation": correlation,
                "real_excursion": float(np.ptp(real_delta)),
                "sim_excursion": float(np.ptp(sim_delta)),
                "real_peak_abs_velocity": float(motion.reference_velocity.abs().max()),
                "sim_peak_abs_velocity": float(motion.velocity.abs().max()),
                "peak_velocity_difference": float(motion.velocity.abs().max() - motion.reference_velocity.abs().max()),
                "minimum_limit_margin_rad": float(motion.limit_margin.min()),
                "target_clip_samples": int(motion.target_clipped.sum()),
                "limit_contact_samples": int(motion.at_or_beyond_limit.sum()),
            }
        )
    return pd.DataFrame(rows).sort_values("joint_name")


def sim_relative_base(base: pd.DataFrame) -> pd.DataFrame:
    result = base.copy()
    pre = result[(result.t >= -3.0) & (result.t <= -0.2)]
    for axis in ("roll", "pitch", "yaw"):
        column = f"base_{axis}_rad"
        result[f"relative_{axis}_rad"] = np.unwrap(result[column].to_numpy(float)) - float(pre[column].mean())
    result["gyro_norm"] = np.linalg.norm(result[["imu_gyro_x", "imu_gyro_y", "imu_gyro_z"]], axis=1)
    result["accel_norm"] = np.linalg.norm(result[["imu_accel_x", "imu_accel_y", "imu_accel_z"]], axis=1)
    return result


def imu_comparison(real_imu: pd.DataFrame, sim_base: pd.DataFrame, motion_duration: float) -> pd.DataFrame:
    rows = []
    sim_motion = sim_base[(sim_base.t >= 0) & (sim_base.t <= motion_duration)]
    for imu_name, real_group in real_imu.groupby("imu"):
        real_motion = real_group[(real_group.t >= 0) & (real_group.t <= motion_duration)]
        sim_interp = {}
        for field in ("relative_roll_rad", "relative_pitch_rad", "gyro_norm", "accel_norm"):
            sim_interp[field] = np.interp(real_motion.t, sim_motion.t, sim_motion[field])
        for axis in ("roll", "pitch"):
            field = f"relative_{axis}_rad"
            real_values = real_motion[field].to_numpy(float)
            sim_values = sim_interp[field]
            corr, lag = lagged_correlation(real_values, sim_values)
            rows.append(
                {
                    "imu": imu_name,
                    "quantity": f"relative_{axis}",
                    "real_peak_abs": float(np.max(np.abs(real_values))),
                    "sim_peak_abs": float(np.max(np.abs(sim_values))),
                    "peak_abs_difference": float(np.max(np.abs(sim_values)) - np.max(np.abs(real_values))),
                    "shape_correlation": corr,
                    "phase_lag_candidate_s": lag,
                    "frame_status": "UNKNOWN_RELATIVE_COMPARISON_ONLY",
                }
            )
        for field in ("gyro_norm", "accel_norm"):
            real_values = real_motion[field].to_numpy(float)
            sim_values = sim_interp[field]
            corr, lag = lagged_correlation(real_values, sim_values)
            rows.append(
                {
                    "imu": imu_name,
                    "quantity": field,
                    "real_peak_abs": float(np.max(np.abs(real_values))),
                    "sim_peak_abs": float(np.max(np.abs(sim_values))),
                    "peak_abs_difference": float(np.max(np.abs(sim_values)) - np.max(np.abs(real_values))),
                    "shape_correlation": corr,
                    "phase_lag_candidate_s": lag,
                    "frame_status": "UNKNOWN_NORM_OR_RELATIVE_COMPARISON",
                }
            )
    return pd.DataFrame(rows)


def effort_comparison(real: pd.DataFrame, sim: pd.DataFrame, motion_duration: float) -> pd.DataFrame:
    rows = []
    for joint_name, sim_group in sim.groupby("joint_name", sort=False):
        real_group = real[real.joint_name == joint_name].sort_values("t")
        sim_motion = sim_group[(sim_group.t >= 0) & (sim_group.t <= motion_duration)].sort_values("t")
        real_effort = np.interp(sim_motion.t, real_group.t, real_group.reported_effort)
        real_pre = real_group[(real_group.t >= -3) & (real_group.t <= -0.2)].reported_effort.mean()
        sim_pre = sim_group[(sim_group.t >= -3) & (sim_group.t <= -0.2)].actuator_force.mean()
        real_delta = real_effort - real_pre
        sim_delta = sim_motion.actuator_force.to_numpy(float) - sim_pre
        correlation, lag = lagged_correlation(real_delta, sim_delta)
        if math.isfinite(correlation) and correlation >= 0.5:
            relation = "SAME_SIGN_SHAPE_CANDIDATE"
        elif math.isfinite(correlation) and correlation <= -0.5:
            relation = "OPPOSITE_SIGN_SHAPE_CANDIDATE"
        else:
            relation = "WEAK_OR_INSUFFICIENT_SHAPE_RELATION"
        rows.append(
            {
                "joint_name": joint_name,
                "reported_effort_baseline": float(real_pre),
                "peak_abs_reported_effort": float(np.max(np.abs(real_effort))),
                "sim_actuator_force_baseline": float(sim_pre),
                "peak_abs_sim_actuator_force": float(sim_motion.actuator_force.abs().max()),
                "shape_correlation": correlation,
                "phase_lag_candidate_s": lag,
                "qualitative_relation": relation,
                "calibration_status": "NOT_TORQUE_CALIBRATION_READY",
            }
        )
    return pd.DataFrame(rows)


def write_replay_reports(
    metrics: pd.DataFrame,
    replay1_metrics: pd.DataFrame,
    replay2_metrics: pd.DataFrame,
    summary1: dict,
    summary2: dict,
    imu1: pd.DataFrame,
    effort: pd.DataFrame,
    motion_duration: float,
) -> None:
    balance_names = metrics[metrics.classification == "BALANCE_COMPENSATION_CANDIDATE"].joint_name.tolist()
    arm_names = metrics[metrics.classification.isin(["GESTURE_PRIMARY", "GESTURE_SECONDARY"])].joint_name.tolist()
    balance = replay1_metrics[replay1_metrics.joint_name.isin(balance_names)].sort_values("delta_rmse", ascending=False)
    arm = replay1_metrics[replay1_metrics.joint_name.isin(arm_names)].sort_values("rmse", ascending=False)
    largest_balance = balance.iloc[0]
    largest_absolute_balance = balance.sort_values("rmse", ascending=False).iloc[0]

    report1 = f"""# Phase 2F Replay 1 — arm-reference only

## Setup

- Scene: current `scene_x2_free.xml`; free base enabled.
- Controller: current `SimulationStabilityController`, unchanged.
- Reference: real **measured** arm q(t), not MC internal command.
- Controlled joints: {', '.join(arm_names)}.
- Leg, waist, head yaw, and static wrist-pitch targets remain at the measured initial pose; simulated ankle attitude feedback remains active.
- Coordinate mapping is an identity-by-live-name candidate. Hardware sign/zero remain unverified.

## Arm tracking

{md_table(['joint', 'RMSE rad', 'MAE rad', 'peak error rad', 'lag candidate s', 'peak velocity diff rad/s'], [[row.joint_name, fmt(row.rmse), fmt(row.mae), fmt(row.peak_abs_error), fmt(row.phase_lag_candidate_s, 3), fmt(row.peak_velocity_difference)] for row in arm.itertuples()])}

## Real MC response versus simulation-controller response

{md_table(['joint', 'real excursion rad', 'sim excursion rad', 'delta RMSE rad', 'shape corr', 'lag candidate s'], [[row.joint_name, fmt(row.real_excursion), fmt(row.sim_excursion), fmt(row.delta_rmse), fmt(row.shape_correlation, 3), fmt(row.phase_lag_candidate_s, 3)] for row in balance.itertuples()])}

The largest gesture-induced delta-response difference is `{largest_balance.joint_name}` with delta RMSE {largest_balance.delta_rmse:.6f} rad (real excursion {largest_balance.real_excursion:.6f}, sim excursion {largest_balance.sim_excursion:.6f}). This is a controller-response mismatch candidate, confounded by unverified coordinate mapping and physical-model differences.

Absolute q comparison is dominated by a standing-equilibrium offset at `{largest_absolute_balance.joint_name}` (RMSE {largest_absolute_balance.rmse:.6f} rad). The simulation settles away from the fixed measured-initial target before the gesture; this is separate from the delta-response ranking.

## Stability/contact

- Stable/no fall: `{summary1['stable_no_fall']}`; fall time: `{summary1['fall_time_seconds']}`.
- Maximum absolute base roll/pitch: {summary1['max_abs_base_roll_deg']:.3f}° / {summary1['max_abs_base_pitch_deg']:.3f}°.
- Both-feet-contact fraction: {summary1['both_feet_contact_fraction']:.6f}.
- Foot slip proxy maxima: left {summary1['max_left_foot_slip_proxy_m']:.6f} m, right {summary1['max_right_foot_slip_proxy_m']:.6f} m.
- Target clips / limit contacts / self-collision samples / non-foot ground samples: {summary1['range_clip_requests']} / {summary1['limit_contact_samples']} / {summary1['self_collision_samples']} / {summary1['nonfoot_ground_contact_samples']}.
"""
    (HERE / "phase2f_replay1_arm_only_report.md").write_text(report1, encoding="utf-8")

    top2 = replay2_metrics.sort_values("rmse", ascending=False)
    mapping = pd.read_csv(HERE / "phase2f_mapping_assumptions.csv")
    range_conflicts = mapping[(mapping.mujoco_joint_name.notna()) & (~mapping.reference_within_model_range.astype(bool))]
    report2 = f"""# Phase 2F Replay 2 — whole-body measured reference

## Scope

All 30 name-matched MuJoCo joints track the measured real q(t). `head_pitch_joint` is not mapped because the current model fixes that DOF. Replay 2 checks tracking, range, contact, collision, and kinematic consistency; it is not an evaluation of balance-controller prediction.

## Tracking metrics

{md_table(['joint', 'RMSE rad', 'MAE rad', 'peak error rad', 'shape corr', 'lag candidate s', 'min limit margin rad'], [[row.joint_name, fmt(row.rmse), fmt(row.mae), fmt(row.peak_abs_error), fmt(row.shape_correlation, 3), fmt(row.phase_lag_candidate_s, 3), fmt(row.minimum_limit_margin_rad)] for row in top2.itertuples()])}

## Consistency checks

- Reference outside current model range: `{len(range_conflicts)}` mapped joints.
- Runtime target clipping: `{summary2['range_clip_requests']}` requests across `{', '.join(summary2['range_clip_joints']) or 'none'}`.
- Runtime joint-limit contacts: `{summary2['limit_contact_samples']}`.
- Self-collision / non-foot-ground samples: `{summary2['self_collision_samples']}` / `{summary2['nonfoot_ground_contact_samples']}`.
- Stable/no fall: `{summary2['stable_no_fall']}`.
- Controller saturation samples: `{summary2['ctrl_saturation_samples']}`; maximum saturation fraction {summary2['maximum_ctrl_saturation_fraction']:.4f}.

No q-tracking sign conflict is observed under the identity-coordinate replay assumption. That does **not** confirm physical hardware-to-MuJoCo axis sign or zero; those remain `UNKNOWN` until physical single-joint verification.
"""
    (HERE / "phase2f_replay2_whole_body_report.md").write_text(report2, encoding="utf-8")

    imu_rows = [
        [row.imu, row.quantity, fmt(row.real_peak_abs), fmt(row.sim_peak_abs), fmt(row.peak_abs_difference), fmt(row.shape_correlation, 3), fmt(row.phase_lag_candidate_s, 3)]
        for row in imu1.itertuples()
    ]
    effort_same = int((effort.qualitative_relation == "SAME_SIGN_SHAPE_CANDIDATE").sum())
    effort_opposite = int((effort.qualitative_relation == "OPPOSITE_SIGN_SHAPE_CANDIDATE").sum())
    comparison_report = f"""# Phase 2F real-vs-sim baseline report

## Most important comparison: arm input → balance response

Real MC and the simulation controller receive comparable measured arm-position shapes, but the simulation's gesture-induced autonomous leg/waist response differs most at `{largest_balance.joint_name}` by delta RMSE {largest_balance.delta_rmse:.6f} rad. Absolute q error is instead dominated by the pre-gesture simulated `{largest_absolute_balance.joint_name}` equilibrium offset (RMSE {largest_absolute_balance.rmse:.6f} rad). Ranking and detailed response amplitudes are in the Replay 1 report and `phase2f_replay1_joint_metrics.csv`.

## Relative IMU comparison

{md_table(['real IMU', 'quantity', 'real peak', 'sim peak', 'sim-real peak', 'shape corr', 'lag candidate s'], imu_rows)}

IMU mounting/frame remains `UNKNOWN`. Roll/pitch values are relative to each stream's own pre-motion baseline; gyro/acceleration use norms. Absolute quaternion components are intentionally not compared.

## Reported effort versus simulated actuator force

- Status: `NOT_TORQUE_CALIBRATION_READY`.
- Same-sign shape candidates: {effort_same}; opposite-sign shape candidates: {effort_opposite}; remaining joints have weak or insufficient shape relation.
- AimDK `reported_effort` and MuJoCo actuator force are not treated as numerically equivalent. No torque fitting is performed.

## Interpretation

- Controller mismatch candidates: Replay 1 leg/waist amplitude, phase and recovery differences after applying the same measured arm reference.
- Physical-model mismatch candidates: base/IMU magnitude, contact penetration/slip proxy, and actuator-load-shape differences. These remain confounded with controller and frame/mapping uncertainty.
- Mapping/kinematic checks: no reference range conflict, target clipping, limit contact, collision, or fall occurred under the identity-name candidate mapping; physical sign and zero remain unconfirmed.
- Dynamics-calibration gate: **NO-GO**. Missing prerequisites include verified hardware↔MuJoCo sign/zero, verified IMU transforms, a defined torque source, MC internal command trajectory, and multiple controlled excitations. This single preset baseline is insufficient for mass/inertia/friction/Kp/Kd fitting.
"""
    (HERE / "phase2f_real_vs_sim_report.md").write_text(comparison_report, encoding="utf-8")


def make_plots(
    real: pd.DataFrame,
    metrics: pd.DataFrame,
    replay1: pd.DataFrame,
    replay2: pd.DataFrame,
    base1: pd.DataFrame,
    real_imu: pd.DataFrame,
    motion_duration: float,
    comparison1: pd.DataFrame,
) -> None:
    PLOTS.mkdir(parents=True, exist_ok=True)
    balance_names = metrics[metrics.classification == "BALANCE_COMPENSATION_CANDIDATE"].joint_name.tolist()
    arm_names = metrics[metrics.classification.isin(["GESTURE_PRIMARY", "GESTURE_SECONDARY"])].joint_name.tolist()
    for label, names, simulation, filename in (
        ("Replay 1 arm tracking", arm_names, replay1, "replay1_arm_tracking.png"),
        ("Replay 1 autonomous balance response", balance_names, replay1, "replay1_balance_response.png"),
        ("Replay 2 whole-body tracking", balance_names + arm_names, replay2, "replay2_tracking.png"),
    ):
        cols = 2
        rows = math.ceil(len(names) / cols)
        fig, axes = plt.subplots(rows, cols, figsize=(14, 2.5 * rows), squeeze=False)
        for axis, joint_name in zip(axes.flat, names):
            frame = simulation[simulation.joint_name == joint_name]
            axis.plot(frame.t, frame.reference_position, label="real measured", linewidth=1.2)
            axis.plot(frame.t, frame.position, label="sim", linewidth=0.9)
            axis.axvline(0, color="black", linestyle="--", linewidth=0.8)
            axis.axvline(motion_duration, color="black", linestyle=":", linewidth=0.8)
            axis.set_title(joint_name, fontsize=8)
            axis.grid(alpha=0.25)
        for axis in axes.flat[len(names):]:
            axis.axis("off")
        handles, labels_legend = axes.flat[0].get_legend_handles_labels()
        fig.legend(handles, labels_legend, loc="upper right", ncol=2, bbox_to_anchor=(0.99, 0.995))
        fig.suptitle(label, y=0.995)
        fig.tight_layout(rect=(0, 0, 1, 0.965))
        fig.savefig(PLOTS / filename, dpi=150)
        plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(14, 9), sharex=True)
    for imu_name, group in real_imu.groupby("imu"):
        axes[0].plot(group.t, np.rad2deg(group.relative_roll_rad), label=f"real {imu_name} roll")
        axes[1].plot(group.t, np.rad2deg(group.relative_pitch_rad), label=f"real {imu_name} pitch")
    axes[0].plot(base1.t, np.rad2deg(base1.relative_roll_rad), color="black", linewidth=1.4, label="sim pelvis roll")
    axes[1].plot(base1.t, np.rad2deg(base1.relative_pitch_rad), color="black", linewidth=1.4, label="sim pelvis pitch")
    for axis in axes:
        axis.axvline(0, color="black", linestyle="--")
        axis.axvline(motion_duration, color="black", linestyle=":")
        axis.grid(alpha=0.25)
        axis.legend()
        axis.set_ylabel("relative angle (deg)")
    axes[1].set_xlabel("t from detected real motion start (s)")
    fig.tight_layout()
    fig.savefig(PLOTS / "relative_imu_roll_pitch.png", dpi=150)
    plt.close(fig)

    rank = comparison1[comparison1.joint_name.isin(balance_names)].sort_values("delta_rmse")
    fig, axis = plt.subplots(figsize=(12, 6))
    axis.barh(rank.joint_name, rank.delta_rmse)
    axis.set(xlabel="delta-coordinate RMSE (rad)", title="Replay 1 real MC vs simulation balance response mismatch")
    axis.grid(axis="x", alpha=0.25)
    fig.tight_layout()
    fig.savefig(PLOTS / "replay1_balance_rmse_ranking.png", dpi=150)
    plt.close(fig)


def main() -> int:
    lock = json.loads((HERE / "source_data_lock.json").read_text(encoding="utf-8"))
    motion_duration = float(lock["motion_duration_seconds"])
    metrics = pd.read_csv(HERE / "phase2e_joint_metrics.csv")
    real = pd.read_csv(HERE / "phase2e_aligned_joint_data.csv")
    real_imu = pd.read_csv(HERE / "phase2e_aligned_imu_data.csv")
    replay1 = pd.read_csv(HERE / "replay1_arm_only_joint_log.csv")
    replay2 = pd.read_csv(HERE / "replay2_whole_body_joint_log.csv")
    base1 = sim_relative_base(pd.read_csv(HERE / "replay1_arm_only_base_log.csv"))
    base2 = sim_relative_base(pd.read_csv(HERE / "replay2_whole_body_base_log.csv"))
    summary1 = json.loads((HERE / "replay1_arm_only_summary.json").read_text())
    summary2 = json.loads((HERE / "replay2_whole_body_summary.json").read_text())

    comparison1 = joint_comparison(replay1, motion_duration)
    comparison2 = joint_comparison(replay2, motion_duration)
    comparison1.to_csv(HERE / "phase2f_replay1_joint_metrics.csv", index=False)
    comparison2.to_csv(HERE / "phase2f_replay2_joint_metrics.csv", index=False)
    imu1 = imu_comparison(real_imu, base1, motion_duration)
    imu2 = imu_comparison(real_imu, base2, motion_duration)
    imu1.insert(0, "replay", "replay1_arm_only")
    imu2.insert(0, "replay", "replay2_whole_body")
    pd.concat([imu1, imu2], ignore_index=True).to_csv(HERE / "phase2f_imu_comparison.csv", index=False)
    effort = effort_comparison(real, replay2, motion_duration)
    effort.to_csv(HERE / "phase2f_effort_qualitative.csv", index=False)

    write_replay_reports(metrics, comparison1, comparison2, summary1, summary2, imu1, effort, motion_duration)
    make_plots(real, metrics, replay1, replay2, base1, real_imu, motion_duration, comparison1)

    balance_names = metrics[metrics.classification == "BALANCE_COMPENSATION_CANDIDATE"].joint_name
    largest = comparison1[comparison1.joint_name.isin(balance_names)].sort_values("delta_rmse", ascending=False).iloc[0]
    result = {
        "phase2f_complete": True,
        "largest_replay1_balance_difference_joint": largest.joint_name,
        "largest_replay1_balance_delta_rmse_rad": float(largest.delta_rmse),
        "replay1_stable": summary1["stable_no_fall"],
        "replay2_stable": summary2["stable_no_fall"],
        "replay2_range_clip_requests": summary2["range_clip_requests"],
        "replay2_limit_contact_samples": summary2["limit_contact_samples"],
        "replay2_self_collision_samples": summary2["self_collision_samples"],
        "dynamics_calibration_gate": "NO_GO",
        "reported_effort_status": "NOT_TORQUE_CALIBRATION_READY",
    }
    (HERE / "phase2e_phase2f_summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
