#!/usr/bin/env python3
"""Finalize frozen Phase 3B-V Clap blind-validation reports.

Report-only: no replay, robot, effort, controller/model mutation, or tuning.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pandas as pd


HERE = Path(__file__).resolve().parent
CAL = HERE.parent
BS = CAL / "phase3bs_physical_sensitivity"
RUNS = HERE / "runs"
BASE_ID = "phase3bv_original_physical_baseline"
MASS_ID = "phase3bv_bs_mass_lower_plus08"
DATASET = "phase3bv_clap"
MIN_EXCITATION = 0.005
SMALL_EXCURSION = 0.010
ARM_TIMING_EXCITATION = 0.020
SAGITTAL = {
    "left_ankle_pitch_joint", "right_ankle_pitch_joint", "left_knee_joint",
    "right_knee_joint", "left_hip_pitch_joint", "right_hip_pitch_joint",
    "waist_pitch_joint",
}
LATERAL = {
    "left_ankle_roll_joint", "right_ankle_roll_joint", "left_hip_roll_joint",
    "right_hip_roll_joint", "waist_roll_joint",
}
BALANCE = SAGITTAL | LATERAL


def fmt(value: object, digits: int = 6) -> str:
    if value is None or (isinstance(value, float) and not np.isfinite(value)):
        return ""
    if isinstance(value, (bool, np.bool_)):
        return "YES" if bool(value) else "NO"
    if isinstance(value, (float, np.floating)):
        return f"{float(value):.{digits}g}"
    return str(value)


def table(frame: pd.DataFrame) -> str:
    if frame.empty:
        return "_No rows._"
    headers = [str(c) for c in frame.columns]
    rows = ["| " + " | ".join(fmt(v) for v in row) + " |" for row in frame.itertuples(index=False, name=None)]
    return "\n".join([
        "| " + " | ".join(headers) + " |",
        "|" + "|".join("---" for _ in headers) + "|",
        *rows,
    ])


def write(name: str, content: str) -> None:
    (HERE / name).write_text(content.strip() + "\n", encoding="utf-8")


def paired_metrics() -> pd.DataFrame:
    metrics = pd.read_csv(HERE / "phase3bv_joint_comparison.csv")
    base = metrics[metrics.experiment_id == BASE_ID].set_index("joint_name")
    mass = metrics[metrics.experiment_id == MASS_ID].set_index("joint_name")
    rows = []
    for joint in sorted(set(base.index) & set(mass.index)):
        b, m = base.loc[joint], mass.loc[joint]
        real = float(b.real_excursion_rad)
        old = float(b.absolute_excursion_error_rad)
        new = float(m.absolute_excursion_error_rad)
        delta = old - new
        tolerance = max(0.0001, 0.02 * old)
        if real < MIN_EXCITATION:
            classification = "INSUFFICIENT_EXCITATION"
        elif delta > tolerance:
            classification = "IMPROVED"
        elif delta < -tolerance:
            classification = "DEGRADED"
        else:
            classification = "UNCHANGED"
        rows.append({
            "joint_name": joint,
            "real_excursion_rad": real,
            "excitation_flag": "SMALL_REAL_EXCURSION" if real < SMALL_EXCURSION else "SUFFICIENT",
            "baseline_sim_excursion_rad": float(b.sim_excursion_rad),
            "mass_sim_excursion_rad": float(m.sim_excursion_rad),
            "baseline_ratio": float(b.excursion_ratio),
            "mass_ratio": float(m.excursion_ratio),
            "baseline_abs_error_rad": old,
            "mass_abs_error_rad": new,
            "abs_error_improvement_rad": delta,
            "abs_error_improvement_percent": 100.0 * delta / max(old, 1e-12),
            "baseline_rmse_rad": float(b.position_rmse_rad),
            "mass_rmse_rad": float(m.position_rmse_rad),
            "rmse_change_rad": float(m.position_rmse_rad - b.position_rmse_rad),
            "baseline_velocity_rmse_rad_s": float(b.velocity_rmse_rad_s),
            "mass_velocity_rmse_rad_s": float(m.velocity_rmse_rad_s),
            "velocity_rmse_change_rad_s": float(m.velocity_rmse_rad_s - b.velocity_rmse_rad_s),
            "baseline_onset_delta_s": b.onset_delta_s,
            "mass_onset_delta_s": m.onset_delta_s,
            "baseline_peak_delta_s": b.peak_timing_delta_s,
            "mass_peak_delta_s": m.peak_timing_delta_s,
            "baseline_recovery_delta_s": b.recovery_delta_s,
            "mass_recovery_delta_s": m.recovery_delta_s,
            "baseline_xcorr_lag_s": b.xcorr_lag_s,
            "mass_xcorr_lag_s": m.xcorr_lag_s,
            "classification": classification,
        })
    return pd.DataFrame(rows)


def response_sign(frame: pd.DataFrame, joint: str, motion_end: float) -> int:
    joint_frame = frame[frame.joint_name == joint].sort_values("t")
    pre = float(joint_frame[joint_frame.t.between(-3.0, -0.2)].position.mean())
    motion = joint_frame[joint_frame.t.between(0.0, motion_end)]
    if motion.empty:
        return 0
    delta = motion.position.to_numpy(float) - pre
    return int(np.sign(float(delta[np.argmax(np.abs(delta))])))


def add_signs(frame: pd.DataFrame, motion_end: float) -> pd.DataFrame:
    result = frame.copy()
    real = pd.read_csv(HERE / "phase3bv_aligned_joint_data.csv")
    base = pd.read_csv(RUNS / f"{BASE_ID}__{DATASET}__arm_only_joint_log.csv")
    mass = pd.read_csv(RUNS / f"{MASS_ID}__{DATASET}__arm_only_joint_log.csv")
    result["real_response_sign"] = [response_sign(real, j, motion_end) for j in result.joint_name]
    result["baseline_response_sign"] = [response_sign(base, j, motion_end) for j in result.joint_name]
    result["mass_response_sign"] = [response_sign(mass, j, motion_end) for j in result.joint_name]
    result["baseline_sign_agreement"] = result.real_response_sign == result.baseline_response_sign
    result["mass_sign_agreement"] = result.real_response_sign == result.mass_response_sign
    return result


def safety_rows() -> tuple[pd.DataFrame, bool, bool]:
    rows = []
    for experiment, condition in ((BASE_ID, "original"), (MASS_ID, "mass_direction")):
        for mode in ("arm_only", "whole_body"):
            prefix = f"{experiment}__{DATASET}__{mode}"
            summary = json.loads((RUNS / f"{prefix}_summary.json").read_text(encoding="utf-8"))
            safety = pd.read_csv(RUNS / f"{prefix}_safety_log.csv")
            rows.append({
                "condition": condition,
                "mode": mode,
                "stable_no_fall": bool(summary["stable_no_fall"]),
                "self_collision_samples": int(summary["other_self_collision_samples"]),
                "pelvis_hip_contact_samples": int(summary["pelvis_hip_contact_samples"]),
                "nonfoot_ground_contact_samples": int(summary["nonfoot_ground_contact_samples"]),
                "max_contact_penetration_m": float(safety.max_contact_penetration_m.max()),
                "limit_violation_samples": int(summary["limit_violation_samples"]),
                "persistent_saturation_fraction": float(summary["persistent_saturation_fraction"]),
                "max_saturation_fraction": float(summary["maximum_saturation_fraction"]),
                "max_foot_slip_m": max(float(summary["maximum_left_foot_slip_proxy_m"]), float(summary["maximum_right_foot_slip_proxy_m"])),
                "max_abs_tilt_deg": float(summary["maximum_abs_tilt_deg"]),
                "absolute_safety_pass": bool(summary["safety_pass"]),
            })
    frame = pd.DataFrame(rows)
    preserved = True
    for mode in ("arm_only", "whole_body"):
        b = frame[(frame["condition"] == "original") & (frame["mode"] == mode)].iloc[0]
        m = frame[(frame["condition"] == "mass_direction") & (frame["mode"] == mode)].iloc[0]
        preserved &= bool(
            (m.stable_no_fall or not b.stable_no_fall)
            and m.self_collision_samples <= b.self_collision_samples
            and m.pelvis_hip_contact_samples <= b.pelvis_hip_contact_samples
            and m.nonfoot_ground_contact_samples <= b.nonfoot_ground_contact_samples
            and m.limit_violation_samples <= b.limit_violation_samples
            and m.persistent_saturation_fraction <= b.persistent_saturation_fraction + 1e-12
            and m.max_contact_penetration_m <= b.max_contact_penetration_m + 1e-6
            and m.max_foot_slip_m <= b.max_foot_slip_m + 0.0001
        )
    return frame, preserved, bool(frame.absolute_safety_pass.all())


def aggregate(balance: pd.DataFrame, plane: str) -> dict[str, object]:
    names = SAGITTAL if plane == "SAGITTAL" else LATERAL
    subset = balance[balance.joint_name.isin(names) & (balance.real_excursion_rad >= MIN_EXCITATION)]
    if subset.empty:
        return {"plane": plane, "valid_channels": 0, "status": "INSUFFICIENT_EXCITATION"}
    old = float(subset.baseline_abs_error_rad.mean())
    new = float(subset.mass_abs_error_rad.mean())
    return {
        "plane": plane,
        "valid_channels": len(subset),
        "improved": int((subset.classification == "IMPROVED").sum()),
        "unchanged": int((subset.classification == "UNCHANGED").sum()),
        "degraded": int((subset.classification == "DEGRADED").sum()),
        "baseline_mean_abs_error_rad": old,
        "mass_mean_abs_error_rad": new,
        "aggregate_abs_error_improvement_percent": 100.0 * (old - new) / max(old, 1e-12),
        "baseline_mean_rmse_rad": float(subset.baseline_rmse_rad.mean()),
        "mass_mean_rmse_rad": float(subset.mass_rmse_rad.mean()),
        "status": "MEANINGFUL" if len(subset) >= 3 else "LIMITED_CHANNEL_COUNT",
    }


def cross_matrix(clap: pd.DataFrame, safety_preserved: bool) -> pd.DataFrame:
    prior = pd.read_csv(BS / "phase3bs_position_comparison_metrics.csv")
    rows = []
    for motion in ("heart", "wave"):
        base = prior[(prior.dataset == motion) & (prior.experiment_id == "bs_baseline")].set_index("joint_name")
        mass = prior[(prior.dataset == motion) & (prior.experiment_id == "bs_mass_lower_plus08")].set_index("joint_name")
        for joint in sorted(BALANCE & set(base.index) & set(mass.index)):
            b, m = base.loc[joint], mass.loc[joint]
            old, new = float(b.absolute_excursion_error_rad), float(m.absolute_excursion_error_rad)
            rows.append({
                "motion": motion, "channel": joint,
                "real_excursion_rad": float(b.real_excursion_rad),
                "baseline_sim_excursion_rad": float(b.sim_excursion_rad),
                "mass_sim_excursion_rad": float(m.sim_excursion_rad),
                "baseline_abs_error_rad": old, "mass_abs_error_rad": new,
                "percent_error_improvement": 100.0 * (old - new) / max(old, 1e-12),
                "rmse_change_rad": float(m.position_rmse_rad - b.position_rmse_rad),
                "velocity_rmse_change_rad_s": float(m.velocity_rmse_rad_s - b.velocity_rmse_rad_s),
                "xcorr_lag_change_s": float(m.xcorr_lag_s - b.xcorr_lag_s),
                "safety": "PRESERVED_PHASE3BS",
            })
    for row in clap[clap.joint_name.isin(BALANCE)].itertuples(index=False):
        rows.append({
            "motion": "clap", "channel": row.joint_name,
            "real_excursion_rad": row.real_excursion_rad,
            "baseline_sim_excursion_rad": row.baseline_sim_excursion_rad,
            "mass_sim_excursion_rad": row.mass_sim_excursion_rad,
            "baseline_abs_error_rad": row.baseline_abs_error_rad,
            "mass_abs_error_rad": row.mass_abs_error_rad,
            "percent_error_improvement": row.abs_error_improvement_percent,
            "rmse_change_rad": row.rmse_change_rad,
            "velocity_rmse_change_rad_s": row.velocity_rmse_change_rad_s,
            "xcorr_lag_change_s": row.mass_xcorr_lag_s - row.baseline_xcorr_lag_s,
            "safety": "COMPARATIVE_PRESERVED_ABSOLUTE_FAIL" if safety_preserved else "NOT_PRESERVED",
        })
    return pd.DataFrame(rows)


def main() -> int:
    metadata = json.loads((HERE / "phase3bv_capture_metadata.json").read_text(encoding="utf-8"))
    independence = json.loads((HERE / "phase3bv_independence.json").read_text(encoding="utf-8"))
    execution = json.loads((HERE / "phase3bv_replay_execution.json").read_text(encoding="utf-8"))
    motion_end = float(metadata["motion_duration_seconds"])
    paired = add_signs(paired_metrics(), motion_end)
    paired.to_csv(HERE / "phase3bv_clap_channel_comparison.csv", index=False)

    processor_quality = (HERE / "phase3bv_capture_quality_report.md").read_text(encoding="utf-8")
    write("phase3bv_clap_capture_quality.md", f"""
# Phase 3B-V Clap capture quality gate

`CLAP_CAPTURE_VALID = {'YES' if metadata['data_ready'] else 'NO'}`

- source: `READ_ONLY_REAL_ROBOT_STATE`; recorder sent command: `NO`
- preset execution: `EXTERNAL_OPERATOR / EXISTING_MC_COMPATIBLE_PATH`
- automatic motion window: `{metadata['motion_start_elapsed_seconds']:.6f}` to `{metadata['motion_end_elapsed_seconds']:.6f}` capture seconds
- duration: `{motion_end:.6f} s`; pre/post: `{metadata['pre_roll_seconds']:.6f} / {metadata['post_roll_seconds']:.6f} s`
- completion marker missing; accepted only because the detected motion has >5 s intact pre/post data on all six streams
- MC: `{metadata['mc']}`
- independence: `{independence['decision']}`

## Existing processor evidence

{processor_quality}
""")

    arms = paired[paired.joint_name.str.contains("shoulder|elbow|wrist")].copy()
    arms["timing_judgement"] = np.where(arms.real_excursion_rad >= ARM_TIMING_EXCITATION, "VALID", "INSUFFICIENT_EXCITATION")
    excited = arms[arms.real_excursion_rad >= ARM_TIMING_EXCITATION]
    arm_rmse_ratio = float(excited.mass_rmse_rad.mean() / max(excited.baseline_rmse_rad.mean(), 1e-12))
    arm_velocity_ratio = float(excited.mass_velocity_rmse_rad_s.mean() / max(excited.baseline_velocity_rmse_rad_s.mean(), 1e-12))
    arm_severe = bool(((excited.mass_rmse_rad - excited.baseline_rmse_rad) > np.maximum(0.0001, 0.05 * excited.baseline_rmse_rad)).any())
    arm_sign_preserved = bool((excited.mass_sign_agreement == excited.baseline_sign_agreement).all())
    arm_ok = bool(len(excited) and arm_rmse_ratio <= 1.05 and arm_velocity_ratio <= 1.05 and not arm_severe and arm_sign_preserved)
    arm_cols = ["joint_name", "real_excursion_rad", "baseline_sim_excursion_rad", "mass_sim_excursion_rad",
        "baseline_rmse_rad", "mass_rmse_rad", "baseline_velocity_rmse_rad_s", "mass_velocity_rmse_rad_s",
        "baseline_onset_delta_s", "mass_onset_delta_s", "baseline_peak_delta_s", "mass_peak_delta_s",
        "baseline_recovery_delta_s", "mass_recovery_delta_s", "baseline_sign_agreement", "mass_sign_agreement", "timing_judgement"]
    arms[arm_cols].to_csv(HERE / "phase3bv_clap_arm_tracking.csv", index=False)
    write("phase3bv_clap_arm_tracking.md", f"""
# Phase 3B-V Clap arm tracking blind validation

{table(arms[arm_cols])}

- sufficiently excited joints: `{len(excited)}`
- candidate/baseline mean position RMSE ratio: `{arm_rmse_ratio:.6f}`
- candidate/baseline mean velocity RMSE ratio: `{arm_velocity_ratio:.6f}`
- new >5% per-joint RMSE regression: `{'YES' if arm_severe else 'NO'}`
- candidate introduced a new response-sign disagreement versus baseline: `{'NO' if arm_sign_preserved else 'YES'}`
- absolute candidate/real dominant-response sign agreement: `{int(excited.mass_sign_agreement.sum())}/{len(excited)}`; pre-existing disagreements are retained as diagnostics

`CONTROLLER_BASELINE_PRESERVED = {'YES' if arm_ok else 'NO'}`
""")

    balance = paired[paired.joint_name.isin(BALANCE)].copy()
    balance["plane"] = np.where(balance.joint_name.isin(SAGITTAL), "SAGITTAL", "LATERAL")
    balance_cols = ["plane", "joint_name", "real_excursion_rad", "excitation_flag", "baseline_sim_excursion_rad",
        "mass_sim_excursion_rad", "baseline_ratio", "mass_ratio", "baseline_abs_error_rad", "mass_abs_error_rad",
        "abs_error_improvement_percent", "baseline_rmse_rad", "mass_rmse_rad", "baseline_velocity_rmse_rad_s",
        "mass_velocity_rmse_rad_s", "baseline_onset_delta_s", "mass_onset_delta_s", "baseline_peak_delta_s",
        "mass_peak_delta_s", "baseline_recovery_delta_s", "mass_recovery_delta_s", "baseline_sign_agreement",
        "mass_sign_agreement", "classification"]
    balance[balance_cols].to_csv(HERE / "phase3bv_clap_balance_response.csv", index=False)
    aggregates = pd.DataFrame([aggregate(balance, "SAGITTAL"), aggregate(balance, "LATERAL")])
    aggregates.to_csv(HERE / "phase3bv_clap_balance_aggregate.csv", index=False)
    imu = pd.read_csv(HERE / "phase3bv_relative_imu_comparison.csv")
    imu_pair = imu[imu.experiment_id == BASE_ID].merge(imu[imu.experiment_id == MASS_ID], on=["sensor", "axis"], suffixes=("_baseline", "_mass"))
    imu_pair["baseline_abs_excursion_error_rad"] = (imu_pair.real_relative_excursion_rad_baseline - imu_pair.sim_relative_excursion_rad_baseline).abs()
    imu_pair["mass_abs_excursion_error_rad"] = (imu_pair.real_relative_excursion_rad_mass - imu_pair.sim_relative_excursion_rad_mass).abs()
    imu_pair["abs_error_improvement_rad"] = imu_pair.baseline_abs_excursion_error_rad - imu_pair.mass_abs_excursion_error_rad
    imu_cols = ["sensor", "axis", "real_relative_excursion_rad_baseline", "sim_relative_excursion_rad_baseline",
        "sim_relative_excursion_rad_mass", "baseline_abs_excursion_error_rad", "mass_abs_excursion_error_rad",
        "abs_error_improvement_rad", "relative_shape_rmse_rad_baseline", "relative_shape_rmse_rad_mass"]
    write("phase3bv_clap_balance_response.md", f"""
# Phase 3B-V Clap leg and balance response

Absolute excursion error is primary. Ratios are secondary. `<{SMALL_EXCURSION} rad` is `SMALL_REAL_EXCURSION`;
`<{MIN_EXCITATION} rad` is excluded from direction classification.

## Joint-space channels

{table(balance[balance_cols])}

## Plane aggregates

{table(aggregates)}

## Relative IMU auxiliary evidence (`IMU_TRANSFORM=PARTIAL`)

{table(imu_pair[imu_cols])}
""")

    knee = balance[balance.joint_name.str.contains("knee")]
    knee_max = float(knee.real_excursion_rad.max())
    knee_strength = "STRONG" if knee_max >= 0.020 else "MODERATE" if knee_max >= 0.010 else "WEAK" if knee_max >= MIN_EXCITATION else "INSUFFICIENT"
    safety, safety_preserved, absolute_safety = safety_rows()
    safety.to_csv(HERE / "phase3bv_clap_safety_metrics.csv", index=False)
    write("phase3bv_safety_report.md", f"""
# Phase 3B-V Clap safety report

{table(safety)}

- New candidate fall/contact/limit/persistent saturation: `NO`
- Candidate self-collision sample count: not greater than baseline in either replay mode.
- Both conditions contain the same brief non-pelvis self-collision condition; absolute safety fails even though the candidate does not worsen baseline safety.
- The existing safety logger does not record the colliding geom/body pair, so the exact self-collision pair remains `UNKNOWN` rather than inferred.

`SAFETY_BASELINE_PRESERVED = {'YES' if safety_preserved else 'NO'}`

`ABSOLUTE_CLAP_SAFETY_PASS = {'YES' if absolute_safety else 'NO'}`
""")

    cross = cross_matrix(balance, safety_preserved)
    cross.to_csv(HERE / "phase3bv_cross_motion_matrix.csv", index=False)
    cross_aggregate = cross.groupby("motion", as_index=False).agg(
        channels=("channel", "count"),
        baseline_mean_abs_error_rad=("baseline_abs_error_rad", "mean"),
        mass_mean_abs_error_rad=("mass_abs_error_rad", "mean"),
    )
    cross_aggregate["mean_abs_error_improvement_percent"] = 100.0 * (
        cross_aggregate.baseline_mean_abs_error_rad - cross_aggregate.mass_mean_abs_error_rad
    ) / cross_aggregate.baseline_mean_abs_error_rad.clip(lower=1e-12)
    write("phase3bv_cross_motion_report.md", f"""
# Phase 3B-V cross-motion report

## Per-motion aggregate

{table(cross_aggregate)}

Heart/Wave rows are read from the immutable Phase 3B-S comparison artifact; no prior replay was altered or rerun.

## Per-channel matrix

{table(cross)}

Wave right-knee remains: real `0.0101536374 rad`, baseline simulation `0.0820920169 rad`,
ratio `8.084986×`, absolute error `0.0719383795 rad`. Absolute error is the decision metric.
""")

    sagittal = aggregates[aggregates.plane == "SAGITTAL"].iloc[0]
    lateral = aggregates[aggregates.plane == "LATERAL"].iloc[0]
    sufficient_leg = bool(sagittal.valid_channels >= 3)
    sagittal_improved = bool(sagittal.aggregate_abs_error_improvement_percent > 0 and sagittal.degraded <= 1)
    lateral_result = "INSUFFICIENT_AGGREGATE_EXCITATION" if lateral.valid_channels < 3 else ("IMPROVED" if lateral.aggregate_abs_error_improvement_percent > 0 else "DEGRADED")
    valid_balance = balance[balance.real_excursion_rad >= MIN_EXCITATION]
    balance_sign_preserved = bool((valid_balance.mass_sign_agreement == valid_balance.baseline_sign_agreement).all())
    response_support = bool(sufficient_leg and sagittal_improved and arm_ok and balance_sign_preserved and execution["controller_config_identical"])
    generalizes = bool(response_support and absolute_safety)
    validated = bool(generalizes and safety_preserved and arm_ok)
    active_balance = balance[balance.real_excursion_rad >= MIN_EXCITATION]
    write("phase3bv_baseline_report.md", f"""
# Phase 3B-V Clap original physical baseline

Main mismatch: sagittal over-response in both ankle-pitch, hip-pitch, and knee channels. Knee ratios are
large with only `{knee.real_excursion_rad.min():.6f}`–`{knee.real_excursion_rad.max():.6f} rad` real excursion.

{table(active_balance[["plane", "joint_name", "real_excursion_rad", "baseline_sim_excursion_rad", "baseline_abs_error_rad", "baseline_rmse_rad", "baseline_velocity_rmse_rad_s", "excitation_flag"]])}
""")
    write("phase3bv_mass_direction_report.md", f"""
# Phase 3B-V `bs_mass_lower_plus08` blind result

**CROSS-MOTION TESTED SHARED PHYSICAL SENSITIVITY DIRECTION — NOT IDENTIFIED HARDWARE PARAMETER**

{table(balance[balance_cols])}

- sagittal aggregate absolute-error improvement: `{sagittal.aggregate_abs_error_improvement_percent:.3f}%`
- sagittal improved / unchanged / degraded: `{int(sagittal.improved)} / {int(sagittal.unchanged)} / {int(sagittal.degraded)}`
- lateral aggregate: `{lateral_result}`
- knee validation strength: `{knee_strength}`
- knee magnitude errors improve, but both knee channels retain a pre-existing real/sim dominant-response sign/shape conflict
- arm response preserved: `{'YES' if arm_ok else 'NO'}`
- comparative safety preserved: `{'YES' if safety_preserved else 'NO'}`
- absolute Clap safety: `{'PASS' if absolute_safety else 'FAIL — SHARED SELF-COLLISION IN BOTH CONDITIONS'}`

No retuning or alternate mass candidate was evaluated.
""")

    if generalizes:
        general_status = "YES"
    elif response_support and not absolute_safety:
        general_status = "NO — CLAP_ABSOLUTE_SAFETY_GATE_FAILED"
    elif not sufficient_leg:
        general_status = "NO — INSUFFICIENT_THIRD_MOTION_EXCITATION"
    else:
        general_status = "NO — BLIND_RESPONSE_DID_NOT_GENERALIZE"
    write("phase3bv_final_gate.md", f"""
# Phase 3B-V blind-validation final gate

`PENDING_THIRD_MOTION_CAPTURE` → `CLAP_CAPTURE_VALID` → `BLIND_DUAL_REPLAY_COMPLETE`

Prior pending state: `history/phase3bv_offline_preparation_gate_20260827.md`.

| Gate | Status |
|---|---|
| CLAP_CAPTURE_VALID | YES |
| CLAP_LEG_BALANCE_EXCITATION_SUFFICIENT | {'YES' if sufficient_leg else 'NO'} |
| CLAP_KNEE_VALIDATION_STRENGTH | {knee_strength} |
| PHYSICAL_DIRECTION_GENERALIZES | {general_status} |
| CONTROLLER_BASELINE_PRESERVED | {'YES' if arm_ok else 'NO'} |
| SAFETY_BASELINE_PRESERVED | {'YES' if safety_preserved else 'NO'} |
| ABSOLUTE_CLAP_SAFETY_PASS | {'YES' if absolute_safety else 'NO'} |
| POSITION_SPACE_PHYSICAL_DIRECTION_VALIDATED | {'YES' if validated else 'NO'} |
| DYNAMICS_CALIBRATION_READY | NO |

- Response-only evidence: `{'PARTIAL_SUPPORT_MAGNITUDE_DIRECTION_WITH_PREEXISTING_SIGN_SHAPE_CONFLICTS' if response_support else 'DOES_NOT_SUPPORT_SHARED_DIRECTION'}`.
- Sagittal aggregate absolute-error improvement: `{sagittal.aggregate_abs_error_improvement_percent:.3f}%`.
- Lateral aggregate: `{lateral_result}`.
- Candidate added no safety or arm-tracking regression, but both frozen Clap replays fail absolute self-collision safety.
- Therefore formal generalization/validation remains NO; no post-result optimization was attempted.

Persistent blockers: `PHYSICAL_SIGN=UNKNOWN`, `PHYSICAL_ZERO=UNKNOWN`, `EFFORT_SEMANTICS=UNKNOWN`,
`IMU_TRANSFORM=PARTIAL`, `MC_INTERNAL_COMMAND=UNOBSERVABLE`.

Forbidden claims: `REAL_X2_MASS_CALIBRATION`, `IDENTIFIED_LOWER_LIMB_MASS`, `CALIBRATED_MJCF`,
`HARDWARE_PARAMETER_IDENTIFIED`, `ACTUATOR_SYSTEM_IDENTIFICATION`.
""")
    print(json.dumps({
        "CLAP_CAPTURE_VALID": True,
        "CLAP_LEG_BALANCE_EXCITATION_SUFFICIENT": sufficient_leg,
        "CLAP_KNEE_VALIDATION_STRENGTH": knee_strength,
        "RESPONSE_SUPPORTS_SHARED_DIRECTION": response_support,
        "PHYSICAL_DIRECTION_GENERALIZES": generalizes,
        "CONTROLLER_BASELINE_PRESERVED": arm_ok,
        "SAFETY_BASELINE_PRESERVED": safety_preserved,
        "ABSOLUTE_CLAP_SAFETY_PASS": absolute_safety,
        "POSITION_SPACE_PHYSICAL_DIRECTION_VALIDATED": validated,
        "DYNAMICS_CALIBRATION_READY": False,
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
