#!/usr/bin/env python3
"""Analyze frozen Phase 3A-V legacy/candidate replays without retuning."""

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
PLOTS = HERE / "plots"
RATE_HZ = 50.0
BALANCE_JOINTS = [
    "left_ankle_pitch_joint", "right_ankle_pitch_joint",
    "left_hip_pitch_joint", "right_hip_pitch_joint",
    "left_knee_joint", "right_knee_joint",
    "waist_pitch_joint", "waist_roll_joint",
]


def fmt(value: object, digits: int = 5) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "UNKNOWN"
    return f"{number:.{digits}f}" if math.isfinite(number) else "UNKNOWN"


def table(headers: list[str], rows: list[list[object]]) -> str:
    clean = lambda value: str(value).replace("|", "\\|").replace("\n", " ")
    lines = ["| " + " | ".join(map(clean, headers)) + " |", "| " + " | ".join(["---"] * len(headers)) + " |"]
    lines += ["| " + " | ".join(clean(value) for value in row) + " |" for row in rows]
    return "\n".join(lines)


def lag_and_corr(reference: np.ndarray, response: np.ndarray, limit_s: float = 1.0) -> tuple[float, float]:
    reference = reference - np.mean(reference)
    response = response - np.mean(response)
    candidates = []
    for lag in range(-round(limit_s * RATE_HZ), round(limit_s * RATE_HZ) + 1):
        if lag > 0:
            left, right = reference[:-lag], response[lag:]
        elif lag < 0:
            left, right = reference[-lag:], response[:lag]
        else:
            left, right = reference, response
        if len(left) >= 10 and np.std(left) > 1e-10 and np.std(right) > 1e-10:
            candidates.append((float(np.corrcoef(left, right)[0, 1]), lag / RATE_HZ))
    corr, lag = max(candidates, default=(np.nan, np.nan), key=lambda value: value[0])
    return lag, corr


def recovery_time(t: np.ndarray, values: np.ndarray, motion_end: float) -> float:
    motion = values[(t >= 0.0) & (t <= motion_end)]
    if not len(motion):
        return np.nan
    threshold = max(0.10 * float(np.max(np.abs(motion))), 1e-4)
    hold = round(0.5 * RATE_HZ)
    for index in np.flatnonzero(t >= motion_end):
        if index + hold <= len(values) and np.all(np.abs(values[index:index + hold]) <= threshold):
            return float(t[index] - motion_end)
    return np.nan


def arm_metrics(log: pd.DataFrame, joint_metrics: pd.DataFrame, motion_end: float, experiment: str) -> pd.DataFrame:
    rows = []
    arm = joint_metrics[(joint_metrics.joint_group == "arm") & (joint_metrics.classification != "STATIC")]
    for source in arm.itertuples(index=False):
        frame = log[(log.joint_name == source.joint_name) & log.t.between(0.0, motion_end)].sort_values("t")
        post = log[(log.joint_name == source.joint_name) & log.t.between(motion_end + 0.5, motion_end + 3.0)]
        if len(frame) < 10:
            continue
        reference = frame.reference_position.to_numpy(float)
        response = frame.position.to_numpy(float)
        error = response - reference
        lag, corr = lag_and_corr(reference, response)
        ref_velocity = frame.reference_velocity.to_numpy(float)
        velocity = frame.velocity.to_numpy(float)
        overshoot = max(0.0, float(response.max() - reference.max()), float(reference.min() - response.min()))
        rows.append({
            "experiment": experiment,
            "joint_name": source.joint_name,
            "real_excursion_rad": source.excursion,
            "rmse_rad": float(np.sqrt(np.mean(error * error))),
            "mae_rad": float(np.mean(np.abs(error))),
            "phase_lag_s": lag,
            "shape_correlation": corr,
            "peak_error_rad": float(np.max(np.abs(error))),
            "peak_velocity_difference_rad_s": float(np.max(np.abs(velocity - ref_velocity))),
            "overshoot_rad": overshoot,
            "settling_error_rad": float(np.mean(np.abs(post.position - post.reference_position))) if len(post) else np.nan,
        })
    return pd.DataFrame(rows)


def centered(frame: pd.DataFrame, value: str) -> tuple[np.ndarray, np.ndarray]:
    frame = frame.sort_values("t")
    t = frame.t.to_numpy(float)
    y = frame[value].to_numpy(float)
    pre = (t >= -3.0) & (t <= -0.2)
    return t, y - float(np.mean(y[pre]))


def balance_metrics(log: pd.DataFrame, real: pd.DataFrame, motion_end: float, experiment: str) -> pd.DataFrame:
    rows = []
    for joint in BALANCE_JOINTS:
        sim = log[log.joint_name == joint].sort_values("t")
        measured = real[real.joint_name == joint].sort_values("t")
        if len(sim) < 10 or len(measured) < 10:
            continue
        t, sim_relative = centered(sim, "position")
        real_t, real_relative_native = centered(measured, "position")
        real_relative = np.interp(t, real_t, real_relative_native)
        motion = (t >= 0.0) & (t <= motion_end)
        r, s = real_relative[motion], sim_relative[motion]
        lag, corr = lag_and_corr(r, s)
        real_exc, sim_exc = float(np.ptp(r)), float(np.ptp(s))
        rows.append({
            "experiment": experiment,
            "joint_name": joint,
            "real_excursion_rad": real_exc,
            "sim_excursion_rad": sim_exc,
            "excursion_ratio": sim_exc / real_exc if real_exc > 1e-8 else np.nan,
            "relative_rmse_rad": float(np.sqrt(np.mean((s - r) ** 2))),
            "phase_lag_s": lag,
            "shape_correlation": corr,
            "peak_timing_difference_s": float(t[motion][np.argmax(np.abs(s))] - t[motion][np.argmax(np.abs(r))]),
            "real_recovery_s": recovery_time(t, real_relative, motion_end),
            "sim_recovery_s": recovery_time(t, sim_relative, motion_end),
            "comparison_scope": "RELATIVE_POSITION_ONLY_UNVERIFIED_SIGN_ZERO",
        })
    return pd.DataFrame(rows)


def equilibrium_metrics(log: pd.DataFrame, real: pd.DataFrame, experiment: str) -> pd.DataFrame:
    rows = []
    for joint in BALANCE_JOINTS:
        sim = log[(log.joint_name == joint) & log.t.between(-3.0, -0.2)]
        measured = real[(real.joint_name == joint) & real.t.between(-3.0, -0.2)]
        if len(sim) and len(measured):
            rows.append({
                "experiment": experiment,
                "joint_name": joint,
                "real_initial_mean_rad": float(measured.position.mean()),
                "sim_target_mean_rad": float(sim.target_position.mean()),
                "sim_settled_mean_rad": float(sim.position.mean()),
                "settled_minus_real_rad": float(sim.position.mean() - measured.position.mean()),
                "classification": "SIMULATION_REFERENCE_ALIGNMENT_NOT_HARDWARE_ZERO_CALIBRATION",
            })
    return pd.DataFrame(rows)


def imu_metrics(base: pd.DataFrame, real_imu: pd.DataFrame, motion_end: float, experiment: str) -> pd.DataFrame:
    rows = []
    sim_t = base.t.to_numpy(float)
    for imu_name, measured in real_imu.groupby("imu"):
        measured = measured.sort_values("t")
        for real_field, sim_field, quantity in (
            ("relative_roll_rad", "base_roll_rad", "relative_roll"),
            ("relative_pitch_rad", "base_pitch_rad", "relative_pitch"),
            ("gyro_norm", "gyro_norm", "gyro_norm"),
        ):
            real_on_sim = np.interp(sim_t, measured.t, measured[real_field])
            sim_values = base[sim_field].to_numpy(float)
            pre = (sim_t >= -3.0) & (sim_t <= -0.2)
            sim_values = sim_values - float(np.mean(sim_values[pre]))
            real_on_sim = real_on_sim - float(np.mean(real_on_sim[pre]))
            motion = (sim_t >= 0.0) & (sim_t <= motion_end)
            r, s = real_on_sim[motion], sim_values[motion]
            lag, corr = lag_and_corr(r, s)
            rows.append({
                "experiment": experiment,
                "real_imu": imu_name,
                "quantity": quantity,
                "real_excursion": float(np.ptp(r)),
                "sim_excursion": float(np.ptp(s)),
                "excursion_ratio": float(np.ptp(s) / np.ptp(r)) if np.ptp(r) > 1e-8 else np.nan,
                "relative_rmse": float(np.sqrt(np.mean((s - r) ** 2))),
                "phase_lag_s": lag,
                "shape_correlation": corr,
                "comparison_scope": "RELATIVE_AUXILIARY_ONLY_IMU_TRANSFORM_PARTIAL",
            })
    return pd.DataFrame(rows)


def family_decision(arm: pd.DataFrame, family: str) -> tuple[str, str]:
    selected = arm[arm.joint_name.str.contains(family)]
    if selected.empty or float(selected.real_excursion_rad.max()) < 0.02:
        return "INSUFFICIENT_EXCITATION", "No joint in this family exceeded 0.02 rad measured excursion."
    legacy = selected[selected.experiment == "phase3av_legacy_arm_only"].set_index("joint_name")
    candidate = selected[selected.experiment == "phase3av_candidate_arm_only"].set_index("joint_name")
    common = legacy.index.intersection(candidate.index)
    outcomes = []
    for joint in common:
        old, new = legacy.loc[joint], candidate.loc[joint]
        outcomes.append(bool(abs(new.phase_lag_s) <= abs(old.phase_lag_s) + 0.02 and new.rmse_rad <= old.rmse_rad * 1.05 and new.overshoot_rad <= old.overshoot_rad + 0.02))
    if outcomes and all(outcomes):
        return "GENERALIZES", f"All {len(outcomes)} sufficiently excited mapped joints pass lag/RMSE/overshoot checks."
    if outcomes and any(outcomes):
        return "PARTIAL_GENERALIZATION", f"{sum(outcomes)}/{len(outcomes)} joints pass."
    return "FAILS_VALIDATION", "No sufficiently excited mapped joint passes all checks."


def plot_arm(reference: pd.DataFrame, legacy: pd.DataFrame, candidate: pd.DataFrame, metrics: pd.DataFrame, motion_end: float) -> None:
    joints = metrics.loc[(metrics.joint_group == "arm") & (metrics.classification != "STATIC")].sort_values("excursion", ascending=False).joint_name.head(8).tolist()
    if not joints:
        return
    fig, axes = plt.subplots(math.ceil(len(joints) / 2), 2, figsize=(13, 3.0 * math.ceil(len(joints) / 2)), squeeze=False, sharex=True)
    for ax, joint in zip(axes.ravel(), joints):
        for frame, label, style in ((reference, "real measured", "k-"), (legacy, "legacy", "C1--"), (candidate, "candidate", "C0-")):
            part = frame[frame.joint_name == joint]
            field = "position" if frame is reference else "position"
            ax.plot(part.t, part[field], style, lw=1.2, label=label)
        ax.axvspan(0.0, motion_end, color="0.9", zorder=-1)
        ax.set_title(joint.removesuffix("_joint"))
        ax.set_ylabel("rad")
        ax.grid(alpha=0.25)
    for ax in axes.ravel()[len(joints):]:
        ax.axis("off")
    axes[0, 0].legend(fontsize=8)
    axes[-1, 0].set_xlabel("t relative to motion onset (s)")
    fig.tight_layout()
    fig.savefig(PLOTS / "before_after" / "arm_tracking_validation.png", dpi=160)
    plt.close(fig)
    for frame, label, subdir in (
        (reference, "real measured", "real"),
        (legacy, "legacy replay", "legacy"),
        (candidate, "frozen candidate replay", "candidate"),
    ):
        fig, ax = plt.subplots(figsize=(11, 5))
        for joint in joints:
            part = frame[frame.joint_name == joint]
            ax.plot(part.t, part.position, lw=1.1, label=joint.removesuffix("_joint"))
        ax.axvspan(0.0, motion_end, color="0.9", zorder=-1)
        ax.set(xlabel="t relative to motion onset (s)", ylabel="position (rad)", title=f"Phase 3A-V {label} active-arm trajectories")
        ax.grid(alpha=0.25)
        ax.legend(fontsize=7, ncol=2)
        fig.tight_layout()
        fig.savefig(PLOTS / subdir / "active_arm_positions.png", dpi=160)
        plt.close(fig)


def plot_balance(real: pd.DataFrame, legacy: pd.DataFrame, candidate: pd.DataFrame, motion_end: float) -> None:
    fig, axes = plt.subplots(4, 2, figsize=(13, 12), sharex=True)
    for ax, joint in zip(axes.ravel(), BALANCE_JOINTS):
        for frame, label, style in ((real, "real relative", "k-"), (legacy, "legacy relative", "C1--"), (candidate, "candidate relative", "C0-")):
            part = frame[frame.joint_name == joint].sort_values("t")
            if part.empty:
                continue
            t, y = centered(part, "position")
            ax.plot(t, y, style, lw=1.2, label=label)
        ax.axvspan(0.0, motion_end, color="0.9", zorder=-1)
        ax.set_title(joint.removesuffix("_joint"))
        ax.set_ylabel("relative rad")
        ax.grid(alpha=0.25)
    axes[0, 0].legend(fontsize=8)
    axes[-1, 0].set_xlabel("t relative to motion onset (s)")
    axes[-1, 1].set_xlabel("t relative to motion onset (s)")
    fig.tight_layout()
    fig.savefig(PLOTS / "before_after" / "balance_response_validation.png", dpi=160)
    plt.close(fig)


def main() -> int:
    metadata = json.loads((HERE / "phase3av_capture_metadata.json").read_text(encoding="utf-8"))
    if not metadata.get("data_ready"):
        raise SystemExit("PHASE3AV_VALIDATION_DATA_READY = NO")
    replay_lock = json.loads((HERE / "phase3av_replay_lock.json").read_text(encoding="utf-8"))
    motion_end = float(metadata["motion_duration_seconds"])
    joint_source = pd.read_csv(HERE / "phase3av_joint_metrics.csv")
    reference = pd.read_csv(HERE / "phase3av_measured_reference.csv", usecols=["t", "joint_name", "joint_group", "position", "velocity"])
    real = pd.read_csv(HERE / "phase3av_aligned_joint_data.csv")
    real_imu = pd.read_csv(HERE / "phase3av_aligned_imu_data.csv")
    independence = json.loads((HERE / "phase3av_independence.json").read_text(encoding="utf-8"))

    logs = {}
    bases = {}
    summaries = {}
    for name in ("phase3av_legacy_arm_only", "phase3av_candidate_arm_only", "phase3av_legacy_whole_body", "phase3av_candidate_whole_body"):
        logs[name] = pd.read_csv(HERE / f"{name}_joint_log.csv")
        bases[name] = pd.read_csv(HERE / f"{name}_base_log.csv")
        summaries[name] = json.loads((HERE / f"{name}_summary.json").read_text(encoding="utf-8"))

    arm = pd.concat([arm_metrics(logs[name], joint_source, motion_end, name) for name in ("phase3av_legacy_arm_only", "phase3av_candidate_arm_only")], ignore_index=True)
    balance = pd.concat([balance_metrics(logs[name], real, motion_end, name) for name in ("phase3av_legacy_arm_only", "phase3av_candidate_arm_only")], ignore_index=True)
    equilibrium = pd.concat([equilibrium_metrics(logs[name], real, name) for name in ("phase3av_legacy_arm_only", "phase3av_candidate_arm_only")], ignore_index=True)
    imu = pd.concat([imu_metrics(bases[name], real_imu, motion_end, name) for name in ("phase3av_legacy_arm_only", "phase3av_candidate_arm_only")], ignore_index=True)
    arm.to_csv(HERE / "phase3av_arm_tracking_metrics.csv", index=False)
    balance.to_csv(HERE / "phase3av_balance_metrics.csv", index=False)
    equilibrium.to_csv(HERE / "phase3av_equilibrium_metrics.csv", index=False)
    imu.to_csv(HERE / "phase3av_relative_imu_metrics.csv", index=False)

    shoulder_roll_decision, shoulder_roll_basis = family_decision(arm, "shoulder_roll")
    shoulder_yaw_decision, shoulder_yaw_basis = family_decision(arm, "shoulder_yaw")
    wrist_yaw_decision, wrist_yaw_basis = family_decision(arm, "wrist_yaw")
    wrist_roll_decision, wrist_roll_basis = family_decision(arm, "wrist_roll")
    legacy_arm = arm[arm.experiment == "phase3av_legacy_arm_only"]
    candidate_arm = arm[arm.experiment == "phase3av_candidate_arm_only"]
    legacy_balance = balance[balance.experiment == "phase3av_legacy_arm_only"]
    candidate_balance = balance[balance.experiment == "phase3av_candidate_arm_only"]
    legacy_eq = equilibrium[equilibrium.experiment == "phase3av_legacy_arm_only"]
    candidate_eq = equilibrium[equilibrium.experiment == "phase3av_candidate_arm_only"]
    knee_legacy = legacy_eq[legacy_eq.joint_name.str.contains("knee")].settled_minus_real_rad.abs().mean()
    knee_candidate = candidate_eq[candidate_eq.joint_name.str.contains("knee")].settled_minus_real_rad.abs().mean()
    standing_generalizes = bool(knee_candidate <= knee_legacy and candidate_eq.settled_minus_real_rad.abs().mean() <= legacy_eq.settled_minus_real_rad.abs().mean() * 1.10)

    excited_balance = candidate_balance[candidate_balance.real_excursion_rad >= 0.005]
    legacy_excited = legacy_balance.set_index("joint_name").reindex(excited_balance.joint_name).reset_index()
    balance_rmse_improves = bool(len(excited_balance) and excited_balance.relative_rmse_rad.mean() <= legacy_excited.relative_rmse_rad.mean() * 1.10)
    ankle = excited_balance[excited_balance.joint_name.str.contains("ankle_pitch")]
    over_corrected = bool(len(ankle) and (ankle.excursion_ratio < 0.70).any())
    candidate_summary = summaries["phase3av_candidate_arm_only"]
    legacy_summary = summaries["phase3av_legacy_arm_only"]
    safety = bool(
        candidate_summary["stable_no_fall"]
        and candidate_summary["self_collision_samples"] == 0
        and candidate_summary["nonfoot_ground_contact_samples"] == 0
        and candidate_summary["target_clip_samples"] == 0
        and candidate_summary["minimum_limit_margin_rad"] > 0.0
        and candidate_summary["persistent_saturation_sample_fraction"] <= 0.01
    )
    arm_better = bool(len(candidate_arm) and candidate_arm.rmse_rad.mean() < legacy_arm.rmse_rad.mean() and candidate_arm.phase_lag_s.abs().mean() <= legacy_arm.phase_lag_s.abs().mean() + 0.02)
    independent_ok = independence["decision"] == "SUFFICIENTLY_INDEPENDENT_VALIDATION_MOTION"
    validated = bool(independent_ok and safety and arm_better and standing_generalizes and balance_rmse_improves and not over_corrected)

    plot_arm(reference, logs["phase3av_legacy_arm_only"], logs["phase3av_candidate_arm_only"], joint_source, motion_end)
    plot_balance(real, logs["phase3av_legacy_arm_only"], logs["phase3av_candidate_arm_only"], motion_end)

    arm_rows = []
    for joint in sorted(set(legacy_arm.joint_name) | set(candidate_arm.joint_name)):
        old = legacy_arm[legacy_arm.joint_name == joint]
        new = candidate_arm[candidate_arm.joint_name == joint]
        if old.empty or new.empty:
            continue
        old, new = old.iloc[0], new.iloc[0]
        arm_rows.append([joint, fmt(old.real_excursion_rad), fmt(old.rmse_rad), fmt(new.rmse_rad), fmt(old.phase_lag_s, 3), fmt(new.phase_lag_s, 3), fmt(old.overshoot_rad), fmt(new.overshoot_rad)])
    balance_rows = []
    for joint in BALANCE_JOINTS:
        old = legacy_balance[legacy_balance.joint_name == joint]
        new = candidate_balance[candidate_balance.joint_name == joint]
        if old.empty or new.empty:
            continue
        old, new = old.iloc[0], new.iloc[0]
        balance_rows.append([joint, fmt(old.real_excursion_rad), fmt(old.excursion_ratio, 3), fmt(new.excursion_ratio, 3), fmt(old.relative_rmse_rad), fmt(new.relative_rmse_rad), fmt(new.phase_lag_s, 3), fmt(new.sim_recovery_s, 3)])
    eq_rows = []
    for joint in BALANCE_JOINTS:
        old = legacy_eq[legacy_eq.joint_name == joint]
        new = candidate_eq[candidate_eq.joint_name == joint]
        if old.empty or new.empty:
            continue
        eq_rows.append([joint, fmt(old.iloc[0].settled_minus_real_rad), fmt(new.iloc[0].settled_minus_real_rad)])

    (HERE / "phase3av_legacy_replay_report.md").write_text(f"""# Phase 3A-V legacy replay

Configuration: frozen pre-Phase-3A simulation controller (`shoulder/wrist=1x`, `balance=1x`, no standing-reference alignment).

- stable/no fall: `{legacy_summary['stable_no_fall']}`
- collision/non-foot contact: `{legacy_summary['self_collision_samples']}/{legacy_summary['nonfoot_ground_contact_samples']}`
- target clips: `{legacy_summary['target_clip_samples']}`
- persistent saturation fraction: `{fmt(legacy_summary['persistent_saturation_sample_fraction'])}`
- mean active-arm RMSE/|lag|: `{fmt(legacy_arm.rmse_rad.mean())} rad / {fmt(legacy_arm.phase_lag_s.abs().mean(), 3)} s`
""", encoding="utf-8")

    (HERE / "phase3av_candidate_replay_report.md").write_text(f"""# Phase 3A-V frozen candidate replay

Configuration is loaded without modification from the locked Phase 3A candidate.

- stable/no fall: `{candidate_summary['stable_no_fall']}`
- collision/non-foot contact: `{candidate_summary['self_collision_samples']}/{candidate_summary['nonfoot_ground_contact_samples']}`
- target clips: `{candidate_summary['target_clip_samples']}`
- persistent saturation fraction: `{fmt(candidate_summary['persistent_saturation_sample_fraction'])}`
- mean active-arm RMSE/|lag|: `{fmt(candidate_arm.rmse_rad.mean())} rad / {fmt(candidate_arm.phase_lag_s.abs().mean(), 3)} s`
- source lock verification: `{replay_lock['source_verification']}`

**NOT HARDWARE CALIBRATION.**
""", encoding="utf-8")

    (HERE / "phase3av_arm_tracking_validation.md").write_text(f"""# Phase 3A-V arm tracking blind validation

{table(['joint', 'real excursion', 'legacy RMSE', 'candidate RMSE', 'legacy lag s', 'candidate lag s', 'legacy overshoot', 'candidate overshoot'], arm_rows)}

- shoulder-roll bandwidth: **{shoulder_roll_decision}** — {shoulder_roll_basis}
- shoulder-yaw bandwidth: **{shoulder_yaw_decision}** — {shoulder_yaw_basis}
- wrist-yaw bandwidth: **{wrist_yaw_decision}** — {wrist_yaw_basis}
- wrist-roll bandwidth: **{wrist_roll_decision}** — {wrist_roll_basis}
- overall candidate vs legacy arm response: **{'IMPROVES' if arm_better else 'DOES_NOT_IMPROVE'}**

The input is `MEASURED_REAL_TRAJECTORY`, not an observable MC internal command. No fixed global time advance or parameter optimization is used.
""", encoding="utf-8")

    (HERE / "phase3av_balance_validation.md").write_text(f"""# Phase 3A-V balance and standing validation

## Standing equilibrium

{table(['joint', 'legacy settled-real rad', 'candidate settled-real rad'], eq_rows)}

- bilateral knee mean absolute mismatch: `{fmt(knee_legacy)} -> {fmt(knee_candidate)} rad`
- standing-reference decision: **{'GENERALIZES' if standing_generalizes else 'FAILS_VALIDATION'}**

## Arm-only autonomous balance response

{table(['joint', 'real excursion', 'legacy ratio', 'candidate ratio', 'legacy RMSE', 'candidate RMSE', 'candidate lag s', 'candidate recovery s'], balance_rows)}

- 0.7x balance response RMSE gate: **{'GENERALIZES_OR_PARTIAL' if balance_rmse_improves else 'FAILS_VALIDATION'}**
- ankle under-response check: **{'BALANCE_GAIN_OVER_CORRECTED' if over_corrected else 'NO_OVER_CORRECTION_DETECTED'}**
- free-base safety gate: **{'PASS' if safety else 'FAIL'}**

IMU results in `phase3av_relative_imu_metrics.csv` use only relative roll/pitch and gyro shape because `IMU_TRANSFORM = PARTIAL`. Reported effort is not used.
""", encoding="utf-8")

    component_rows = [
        ["shoulder/wrist bandwidth", "shoulder_roll=" + shoulder_roll_decision + "; shoulder_yaw=" + shoulder_yaw_decision + "; wrist_yaw=" + wrist_yaw_decision + "; wrist_roll=" + wrist_roll_decision],
        ["standing-reference alignment", "GENERALIZES" if standing_generalizes else "FAILS_VALIDATION"],
        ["0.7x balance gain", "BALANCE_GAIN_OVER_CORRECTED" if over_corrected else ("GENERALIZES" if balance_rmse_improves else "FAILS_VALIDATION")],
    ]
    (HERE / "phase3av_generalization_report.md").write_text(f"""# Phase 3A-V generalization report

- selected motion: `wave(right)`, native MC preset 1002 / area 2
- independence: **{independence['decision']}**
- legacy vs candidate overall arm tracking: **{'CANDIDATE_BETTER' if arm_better else 'LEGACY_NOT_WORSE'}**

{table(['frozen Phase 3A change', 'blind-validation result'], component_rows)}

No failed component was retuned. Any residual mismatch remains validation evidence and, where applicable, a `PHYSICAL_MODEL_MISMATCH_CANDIDATE` for a later phase.
""", encoding="utf-8")

    whole_safety = all(
        summaries[name]["stable_no_fall"]
        and summaries[name]["self_collision_samples"] == 0
        and summaries[name]["nonfoot_ground_contact_samples"] == 0
        and summaries[name]["target_clip_samples"] == 0
        and summaries[name]["minimum_limit_margin_rad"] > 0.0
        and summaries[name]["persistent_saturation_sample_fraction"] <= 0.01
        for name in ("phase3av_legacy_whole_body", "phase3av_candidate_whole_body")
    )
    final = f"""# Phase 3A-V final gate

1. Second real motion: **wave(right)**, preset 1002 / area 2.
2. Independent from heart: **{independence['decision']}**.
3. Shoulder-roll improvement: **{shoulder_roll_decision}**; shoulder yaw: **{shoulder_yaw_decision}**.
4. Wrist-yaw improvement: **{wrist_yaw_decision}**.
5. Standing knee equilibrium: **{'GENERALIZES' if standing_generalizes else 'FAILS_VALIDATION'}**, `{fmt(knee_legacy)} -> {fmt(knee_candidate)} rad`.
6. 0.7x balance gain: **{'BALANCE_GAIN_OVER_CORRECTED' if over_corrected else ('GENERALIZES' if balance_rmse_improves else 'FAILS_VALIDATION')}**.
7. Legacy vs candidate: **{'CANDIDATE_BETTER' if arm_better else 'CANDIDATE_NOT_BETTER'}** on active-arm aggregate.
8. Free-base stable: **{'YES' if candidate_summary['stable_no_fall'] else 'NO'}**.
9. New collision/limit/saturation: **{'NO' if safety and whole_safety else 'YES_OR_UNRESOLVED'}**.
10. Independently validated candidates: see component table in `phase3av_generalization_report.md`.
11. Rejected/downgraded candidates: any component marked `FAILS_VALIDATION`, `PARTIAL_GENERALIZATION`, or `BALANCE_GAIN_OVER_CORRECTED`.
12. **VALIDATED_SIM_CONTROLLER_BASELINE = {'YES' if validated else 'NO'}**.

`DYNAMICS_CALIBRATION_READY = NO`. This validation does not enable torque calibration or physical system identification.
"""
    (HERE / "phase3av_final_gate.md").write_text(final, encoding="utf-8")
    print(json.dumps({"VALIDATED_SIM_CONTROLLER_BASELINE": "YES" if validated else "NO", "independence": independence["decision"], "safety": safety}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
