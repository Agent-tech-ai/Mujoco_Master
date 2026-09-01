#!/usr/bin/env python3
"""Analyze Phase 2G evidence and generate readiness reports."""

from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation


HERE = Path(__file__).resolve().parent
CAL = HERE.parent
P2E = CAL / "phase2e_replay"
REAL = CAL / "logs" / "real" / "phase2d_heart_001"
PLOTS = HERE / "plots"
RATE_HZ = 50.0
LOCK = json.loads((P2E / "source_data_lock.json").read_text(encoding="utf-8"))
MOTION_END = float(LOCK["motion_duration_seconds"])
REFERENCE = pd.read_csv(P2E / "phase2e_heart_measured_reference.csv")
REAL_JOINT = pd.read_csv(P2E / "phase2e_aligned_joint_data.csv")
REAL_IMU = pd.read_csv(P2E / "phase2e_aligned_imu_data.csv")


LOG_PATHS = {
    "free_baseline": P2E / "replay1_arm_only_joint_log.csv",
    "fixed_base_baseline": HERE / "fixed_base_baseline_joint_log.csv",
    "fixed_base_50hz_zoh": HERE / "fixed_base_50hz_zoh_joint_log.csv",
    "free_reference_advance_030": HERE / "free_reference_advance_030_joint_log.csv",
    "free_balance_gain_scale_060": HERE / "free_balance_gain_scale_060_joint_log.csv",
    "free_equilibrium_target_compensation": HERE / "free_equilibrium_target_compensation_joint_log.csv",
}
BASE_PATHS = {
    "free_baseline": P2E / "replay1_arm_only_base_log.csv",
    **{
        name: HERE / f"{name}_base_log.csv"
        for name in LOG_PATHS
        if name != "free_baseline"
    },
}


def fmt(value: object, digits: int = 5) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "UNKNOWN"
    return f"{number:.{digits}f}" if math.isfinite(number) else "UNKNOWN"


def table(headers: list[str], rows: list[list[object]]) -> str:
    def cell(value: object) -> str:
        return str(value).replace("|", "\\|").replace("\n", " ")

    lines = [
        "| " + " | ".join(map(cell, headers)) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    lines.extend("| " + " | ".join(map(cell, row)) + " |" for row in rows)
    return "\n".join(lines)


def lagged_correlation(reference: np.ndarray, response: np.ndarray, max_lag_s: float = 1.0) -> tuple[float, float]:
    candidates: list[tuple[float, float]] = []
    for lag in range(-round(max_lag_s * RATE_HZ), round(max_lag_s * RATE_HZ) + 1):
        if lag > 0:
            x, y = reference[:-lag], response[lag:]
        elif lag < 0:
            x, y = reference[-lag:], response[:lag]
        else:
            x, y = reference, response
        if len(x) >= 10 and np.std(x) > 1e-10 and np.std(y) > 1e-10:
            candidates.append((float(np.corrcoef(x, y)[0, 1]), lag / RATE_HZ))
    return max(candidates, default=(np.nan, np.nan), key=lambda item: item[0])


def relative_sim_base(frame: pd.DataFrame) -> pd.DataFrame:
    result = frame.copy()
    pre = result[(result.t >= -3.0) & (result.t <= -0.2)]
    for axis in ("roll", "pitch"):
        source = f"base_{axis}_rad"
        result[f"relative_{axis}_rad"] = np.unwrap(result[source]) - float(pre[source].mean())
    result["gyro_norm"] = np.linalg.norm(result[["imu_gyro_x", "imu_gyro_y", "imu_gyro_z"]], axis=1)
    return result


def recovery_time(t: np.ndarray, values: np.ndarray) -> float:
    post = np.flatnonzero(t >= MOTION_END)
    if len(post) == 0:
        return np.nan
    peak = float(np.max(np.abs(values[(t >= 0) & (t <= MOTION_END)])))
    threshold = max(0.1 * peak, 1e-4)
    hold = round(0.5 * RATE_HZ)
    for index in post:
        if index + hold <= len(values) and np.all(np.abs(values[index : index + hold]) <= threshold):
            return float(t[index] - MOTION_END)
    return np.nan


def imu_transform_observation() -> pd.DataFrame:
    pre = REAL_IMU[(REAL_IMU.t >= -3.0) & (REAL_IMU.t <= -0.2)]
    chest = pre[pre.imu == "chest"].sort_values("t").reset_index(drop=True)
    torso = pre[pre.imu == "torso"].sort_values("t").reset_index(drop=True)
    common = np.intersect1d(chest.t.to_numpy(), torso.t.to_numpy())
    chest = chest[chest.t.isin(common)].sort_values("t")
    torso = torso[torso.t.isin(common)].sort_values("t")
    qcols = ["orientation_x", "orientation_y", "orientation_z", "orientation_w"]
    relative = Rotation.from_quat(chest[qcols].to_numpy()).inv() * Rotation.from_quat(torso[qcols].to_numpy())
    mean = relative.mean()
    dispersion = (mean.inv() * relative).magnitude()
    euler = mean.as_euler("xyz", degrees=True)
    quat = mean.as_quat()
    rows = [
        {
            "observation": "torso_orientation_relative_to_chest_pre_roll",
            "samples": len(common),
            "mean_qx": quat[0],
            "mean_qy": quat[1],
            "mean_qz": quat[2],
            "mean_qw": quat[3],
            "mean_roll_deg": euler[0],
            "mean_pitch_deg": euler[1],
            "mean_yaw_deg": euler[2],
            "angular_dispersion_std_deg": np.rad2deg(np.std(dispersion)),
            "angular_dispersion_max_deg": np.rad2deg(np.max(dispersion)),
            "status": "PARTIAL_TRANSFORM_OBSERVATION_ONLY",
        }
    ]
    return pd.DataFrame(rows)


def imu_comparisons() -> pd.DataFrame:
    rows: list[dict[str, object]] = []
    for experiment in ("free_baseline", "free_reference_advance_030", "free_balance_gain_scale_060", "free_equilibrium_target_compensation"):
        sim = relative_sim_base(pd.read_csv(BASE_PATHS[experiment]))
        sim_motion = sim[(sim.t >= 0) & (sim.t <= MOTION_END)]
        for imu_name in ("chest", "torso"):
            real = REAL_IMU[(REAL_IMU.imu == imu_name) & (REAL_IMU.t >= 0) & (REAL_IMU.t <= MOTION_END)]
            for quantity in ("relative_roll_rad", "relative_pitch_rad", "gyro_norm"):
                real_values = real[quantity].to_numpy(float)
                sim_values = np.interp(real.t, sim_motion.t, sim_motion[quantity])
                corr, lag = lagged_correlation(real_values, sim_values)
                rows.append(
                    {
                        "experiment": experiment,
                        "real_imu": imu_name,
                        "quantity": quantity,
                        "rmse": float(np.sqrt(np.mean((sim_values - real_values) ** 2))),
                        "real_peak_abs": float(np.max(np.abs(real_values))),
                        "sim_peak_abs": float(np.max(np.abs(sim_values))),
                        "phase_lag_candidate_s": lag,
                        "shape_correlation": corr,
                        "frame_status": "PARTIAL_TRANSFORM_RELATIVE_BASELINE_ONLY" if quantity != "gyro_norm" else "ROTATION_INVARIANT_NORM_ONLY",
                    }
                )
    return pd.DataFrame(rows)


def equilibrium_table() -> pd.DataFrame:
    baseline = pd.read_csv(LOG_PATHS["free_baseline"])
    candidate = pd.read_csv(LOG_PATHS["free_equilibrium_target_compensation"])
    rows: list[dict[str, object]] = []
    for name, group in baseline.groupby("joint_name", sort=True):
        pre = group[(group.t >= -3.0) & (group.t <= -0.2)]
        initial = group.iloc[0]
        cand = candidate[(candidate.joint_name == name) & (candidate.t >= -3.0) & (candidate.t <= -0.2)]
        real_q = float(pre.reference_position.mean())
        target_q = float(pre.target_position.mean())
        settled_q = float(pre.position.mean())
        controller_delta = settled_q - target_q
        rows.append(
            {
                "joint_name": name,
                "real_initial_q_rad": real_q,
                "sim_initial_q_rad": float(initial.position),
                "sim_controller_target_rad": target_q,
                "sim_settled_q_rad": settled_q,
                "target_minus_real_rad": target_q - real_q,
                "settled_minus_target_rad": controller_delta,
                "settled_minus_real_rad": settled_q - real_q,
                "alignment_candidate_target_rad": float(cand.target_position.mean()),
                "alignment_candidate_settled_rad": float(cand.position.mean()),
                "alignment_candidate_settled_minus_real_rad": float(cand.position.mean() - real_q),
                "equilibrium_classification": "CONTROLLER_EQUILIBRIUM_MISMATCH" if abs(controller_delta) >= 0.01 else "WITHIN_0P01_RAD",
                "zero_classification": "POSSIBLE_ZERO_MISMATCH_UNRESOLVED_NOT_ATTRIBUTED",
            }
        )
    return pd.DataFrame(rows)


def tracking_metrics() -> pd.DataFrame:
    rows: list[dict[str, object]] = []
    for experiment, path in LOG_PATHS.items():
        frame = pd.read_csv(path)
        for name, group in frame.groupby("joint_name", sort=True):
            if str(group.input_mode.iloc[0]) not in ("MEASURED_REAL_TRAJECTORY",):
                continue
            motion = group[(group.t >= 0) & (group.t <= MOTION_END)].sort_values("t")
            real = REFERENCE[REFERENCE.joint_name == name].sort_values("t")
            reference = np.interp(motion.t, real.t, real.position)
            response = motion.position.to_numpy(float)
            ref_delta = reference - reference[0]
            response_delta = response - response[0]
            corr, lag = lagged_correlation(ref_delta, response_delta)
            rows.append(
                {
                    "experiment": experiment,
                    "joint_name": name,
                    "rmse_rad": float(np.sqrt(np.mean((response - reference) ** 2))),
                    "delta_rmse_rad": float(np.sqrt(np.mean((response_delta - ref_delta) ** 2))),
                    "phase_lag_candidate_s": lag,
                    "shape_correlation": corr,
                    "maximum_ctrl_saturation_fraction": float(motion.ctrl_saturation_fraction.max()),
                    "peak_abs_velocity_rad_s": float(motion.velocity.abs().max()),
                }
            )
    return pd.DataFrame(rows)


BALANCE_JOINTS = [
    "left_ankle_pitch_joint", "right_ankle_pitch_joint",
    "left_hip_pitch_joint", "right_hip_pitch_joint",
    "left_knee_joint", "right_knee_joint",
    "waist_pitch_joint", "waist_roll_joint",
]


def balance_metrics() -> pd.DataFrame:
    rows: list[dict[str, object]] = []
    experiments = ("free_baseline", "free_reference_advance_030", "free_balance_gain_scale_060", "free_equilibrium_target_compensation")
    for experiment in experiments:
        sim = pd.read_csv(LOG_PATHS[experiment])
        for name in BALANCE_JOINTS:
            real_group = REAL_JOINT[REAL_JOINT.joint_name == name].sort_values("t")
            sim_group = sim[sim.joint_name == name].sort_values("t")
            real_pre = float(real_group[(real_group.t >= -3) & (real_group.t <= -0.2)].position.mean())
            sim_pre = float(sim_group[(sim_group.t >= -3) & (sim_group.t <= -0.2)].position.mean())
            real_motion = real_group[(real_group.t >= 0) & (real_group.t <= MOTION_END)]
            sim_values = np.interp(real_motion.t, sim_group.t, sim_group.position) - sim_pre
            real_values = real_motion.position.to_numpy(float) - real_pre
            corr, lag = lagged_correlation(real_values, sim_values)
            real_peak_t = float(real_motion.t.iloc[int(np.argmax(np.abs(real_values)))])
            sim_peak_t = float(real_motion.t.iloc[int(np.argmax(np.abs(sim_values)))])
            real_all = real_group.position.to_numpy(float) - real_pre
            sim_all = sim_group.position.to_numpy(float) - sim_pre
            rows.append(
                {
                    "experiment": experiment,
                    "quantity": name,
                    "kind": "joint",
                    "real_excursion": float(np.ptp(real_values)),
                    "sim_excursion": float(np.ptp(sim_values)),
                    "excursion_ratio": float(np.ptp(sim_values) / np.ptp(real_values)) if np.ptp(real_values) > 1e-9 else np.nan,
                    "delta_rmse": float(np.sqrt(np.mean((sim_values - real_values) ** 2))),
                    "peak_timing_difference_s": sim_peak_t - real_peak_t,
                    "phase_lag_candidate_s": lag,
                    "shape_correlation": corr,
                    "real_recovery_s": recovery_time(real_group.t.to_numpy(), real_all),
                    "sim_recovery_s": recovery_time(sim_group.t.to_numpy(), sim_all),
                    "frame_status": "JOINT_MAPPING_IDENTITY_CANDIDATE_UNVERIFIED_SIGN_ZERO",
                }
            )

        sim_base = relative_sim_base(pd.read_csv(BASE_PATHS[experiment]))
        real_torso = REAL_IMU[REAL_IMU.imu == "torso"].sort_values("t")
        for quantity in ("relative_roll_rad", "relative_pitch_rad", "gyro_norm"):
            real_motion = real_torso[(real_torso.t >= 0) & (real_torso.t <= MOTION_END)]
            real_values = real_motion[quantity].to_numpy(float)
            sim_values = np.interp(real_motion.t, sim_base.t, sim_base[quantity])
            corr, lag = lagged_correlation(real_values, sim_values)
            rows.append(
                {
                    "experiment": experiment,
                    "quantity": f"torso_{quantity}",
                    "kind": "imu",
                    "real_excursion": float(np.ptp(real_values)),
                    "sim_excursion": float(np.ptp(sim_values)),
                    "excursion_ratio": float(np.ptp(sim_values) / np.ptp(real_values)) if np.ptp(real_values) > 1e-9 else np.nan,
                    "delta_rmse": float(np.sqrt(np.mean((sim_values - real_values) ** 2))),
                    "peak_timing_difference_s": float(real_motion.t.iloc[int(np.argmax(np.abs(sim_values)))]) - float(real_motion.t.iloc[int(np.argmax(np.abs(real_values)))]),
                    "phase_lag_candidate_s": lag,
                    "shape_correlation": corr,
                    "real_recovery_s": np.nan,
                    "sim_recovery_s": np.nan,
                    "frame_status": "ROTATION_INVARIANT_NORM_ONLY" if quantity == "gyro_norm" else "PARTIAL_TRANSFORM_RELATIVE_BASELINE_ONLY",
                }
            )
    return pd.DataFrame(rows)


def experiment_summary(tracking: pd.DataFrame, balance: pd.DataFrame, equilibrium: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, object]] = []
    for experiment in LOG_PATHS:
        t = tracking[tracking.experiment == experiment]
        # Aggregate only joint-angle RMSE here; IMU gyro has different units.
        b = balance[(balance.experiment == experiment) & (balance.kind == "joint")]
        summary_path = (P2E / "replay1_arm_only_summary.json") if experiment == "free_baseline" else (HERE / f"{experiment}_summary.json")
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        rows.append(
            {
                "experiment": experiment,
                "classification": "EXISTING_BASELINE" if experiment == "free_baseline" else summary.get("classification", "DIAGNOSTIC_BASELINE"),
                "changed_category": {
                    "free_baseline": "none",
                    "fixed_base_baseline": "base constraint",
                    "fixed_base_50hz_zoh": "reference interpolation",
                    "free_reference_advance_030": "reference timing",
                    "free_balance_gain_scale_060": "simulation balance gains",
                    "free_equilibrium_target_compensation": "standing equilibrium targets",
                }[experiment],
                "mean_arm_rmse_rad": float(t.rmse_rad.mean()) if len(t) else np.nan,
                "median_arm_phase_lag_s": float(t.phase_lag_candidate_s.median()) if len(t) else np.nan,
                "mean_balance_delta_rmse": float(b.delta_rmse.mean()) if len(b) else np.nan,
                "stable_no_fall": summary.get("stable_no_fall"),
                "maximum_ctrl_saturation_fraction": summary.get("maximum_ctrl_saturation_fraction"),
                "physical_parameters_modified": False,
                "status": "NOT HARDWARE CALIBRATION",
            }
        )
    return pd.DataFrame(rows)


def make_plots(tracking: pd.DataFrame, balance: pd.DataFrame, equilibrium: pd.DataFrame) -> None:
    PLOTS.mkdir(parents=True, exist_ok=True)
    joints = ["left_shoulder_roll_joint", "right_shoulder_roll_joint", "left_wrist_yaw_joint", "right_wrist_yaw_joint"]
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), sharex=True)
    for axis, name in zip(axes.flat, joints):
        real = REFERENCE[REFERENCE.joint_name == name]
        axis.plot(real.t, real.position, label="real measured", color="black", linewidth=1.3)
        for experiment, style in (("free_baseline", "-"), ("fixed_base_baseline", "--"), ("free_reference_advance_030", ":")):
            frame = pd.read_csv(LOG_PATHS[experiment])
            group = frame[frame.joint_name == name]
            axis.plot(group.t, group.position, style, label=experiment, linewidth=1.0)
        axis.set_title(name)
        axis.set_xlim(-1, MOTION_END + 1)
        axis.grid(alpha=0.25)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=4)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(PLOTS / "tracking_fixed_free_timing.png", dpi=150)
    plt.close(fig)

    selected = balance[balance.quantity.isin(["left_ankle_pitch_joint", "right_ankle_pitch_joint", "left_knee_joint", "right_knee_joint", "torso_relative_pitch_rad"])]
    pivot = selected.pivot(index="quantity", columns="experiment", values="delta_rmse")
    pivot[[column for column in ("free_baseline", "free_balance_gain_scale_060", "free_equilibrium_target_compensation") if column in pivot]].plot(kind="bar", figsize=(13, 6))
    plt.ylabel("relative-response RMSE")
    plt.grid(axis="y", alpha=0.25)
    plt.xticks(rotation=25, ha="right")
    plt.tight_layout()
    plt.savefig(PLOTS / "balance_alignment_before_after.png", dpi=150)
    plt.close()

    top = equilibrium.reindex(equilibrium.settled_minus_real_rad.abs().sort_values(ascending=False).index).head(12)
    x = np.arange(len(top))
    fig, axis = plt.subplots(figsize=(14, 6))
    axis.bar(x - 0.2, top.settled_minus_real_rad, width=0.4, label="baseline settled - real")
    axis.bar(x + 0.2, top.alignment_candidate_settled_minus_real_rad, width=0.4, label="target-comp candidate - real")
    axis.set_xticks(x, top.joint_name, rotation=35, ha="right")
    axis.axhline(0, color="black", linewidth=0.8)
    axis.set_ylabel("rad")
    axis.legend()
    axis.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(PLOTS / "standing_equilibrium_offsets.png", dpi=150)
    plt.close(fig)


def write_reports(imu_obs: pd.DataFrame, imu_cmp: pd.DataFrame, equilibrium: pd.DataFrame, tracking: pd.DataFrame, balance: pd.DataFrame, experiments: pd.DataFrame) -> None:
    obs = imu_obs.iloc[0]
    imu_report = f"""# Phase 2G IMU transform investigation

Overall status: **PARTIAL_TRANSFORM**. No raw log was modified and no transform is promoted to confirmed.

| Classification | Result |
|---|---|
| CONFIRMED_TRANSFORM | none |
| PARTIAL_TRANSFORM | message frame labels, upstream URDF mounting candidates, and a stable observed chest/torso relative rotation |
| UNKNOWN | deployed TF chain, driver rotation convention, and sensor-to-comparison-frame transform |

## CONFIRMED facts

- Phase 2D topics are `/aima/hal/imu/chest/state` and `/aima/hal/imu/torso/state`, both `sensor_msgs/msg/Imu`.
- All 1470 chest and 1469 torso messages in the accepted capture report `frame_id=base_link`.
- The Phase 2D graph contains `/tf_static` (`tf2_msgs/msg/TFMessage`), but the subscription-only recorder did not capture its messages. Phase 2A also discovered TF topics, without a transform snapshot.
- The supplied upstream `assets/Master/ff_master_fist.urdf` has candidate fixed joints: pelvis -> `imu_in_pelvis_link`, xyz `(0.0239465,-0.0002287,0.0417100)`, rpy `(0,0,0)`; torso_link -> `imu_in_torso_link`, xyz `(-0.0248787,0.0019853,0.1059381)`, rpy `(0,0,0)`.
- That URDF uses `pelvis` as its root and contains no `base_link`, so it does not resolve what the live message label means.

## PARTIAL_TRANSFORM evidence

During the stationary pre-roll, the observed torso orientation relative to chest was mean Euler xyz **({fmt(obs.mean_roll_deg,3)}, {fmt(obs.mean_pitch_deg,3)}, {fmt(obs.mean_yaw_deg,3)}) deg**, with angular dispersion std {fmt(obs.angular_dispersion_std_deg,4)} deg (n={int(obs.samples)}). This stable offset is observational evidence, not a mounting calibration. It conflicts with treating both absolute quaternions as already interchangeable merely because both headers say `base_link`.

The upstream URDF is not proven to be the deployed robot description and it only specifies IMU-link mounting relative to different parent links; waist pose is also between pelvis and torso. No captured deployed TF tree, robot_description hash, driver convention, or static transform links the message orientation convention to the MuJoCo comparison frame.

## UNKNOWN

- Whether the driver rotates orientation, gyro and acceleration into `base_link`, or only labels the message.
- The deployed chest/torso sensor mounting rotations and TF authorities.
- Gravity inclusion remains consistent with the ~9.8 m/s² stationary acceleration norm, but driver filtering/convention is not source-confirmed.

Because the transform is not confirmed, no real-IMU conversion tool was created. `phase2g_imu_relative_comparison.csv` recomputes only independently baselined relative roll/pitch plus rotation-invariant gyro norm; it is not an axis-aligned IMU calibration.
"""
    (CAL / "phase2g_imu_transform_report.md").write_text(imu_report, encoding="utf-8")

    top = equilibrium.reindex(equilibrium.settled_minus_real_rad.abs().sort_values(ascending=False).index).head(12)
    eq_rows = [[r.joint_name, fmt(r.real_initial_q_rad), fmt(r.sim_initial_q_rad), fmt(r.sim_controller_target_rad), fmt(r.sim_settled_q_rad), fmt(r.settled_minus_target_rad), fmt(r.settled_minus_real_rad), r.equilibrium_classification] for r in top.itertuples()]
    knees = equilibrium[equilibrium.joint_name.isin(["left_knee_joint", "right_knee_joint"])]
    eq_report = f"""# Phase 2G standing equilibrium mismatch

The knee offsets are classified as **CONTROLLER_EQUILIBRIUM_MISMATCH**, not hardware zero. In the baseline, each controller target is equal (within interpolation noise) to the real pre-roll reference, while the settled simulated joint differs from that target.

{table(["joint", "real initial", "sim initial", "controller target", "sim settled", "settled-target", "settled-real", "class"], eq_rows)}

For left/right knee, baseline settled-minus-real is {fmt(knees.iloc[0].settled_minus_real_rad)} / {fmt(knees.iloc[1].settled_minus_real_rad)} rad. A simulation-only one-shot standing-target compensation reduced the absolute knee residuals to {fmt(knees.iloc[0].alignment_candidate_settled_minus_real_rad)} / {fmt(knees.iloc[1].alignment_candidate_settled_minus_real_rad)} rad, but did not eliminate settled-minus-target behavior and worsened some ankle equilibria. It is therefore only `SIM_CONTROLLER_ALIGNMENT_CANDIDATE` and is not adopted as a calibrated state.

`POSSIBLE_ZERO_MISMATCH` remains unresolved for every joint. Real encoder zero cannot be inferred from agreement or disagreement with an unverified identity replay mapping.
"""
    (CAL / "phase2g_standing_equilibrium_report.md").write_text(eq_report, encoding="utf-8")

    key = tracking[tracking.joint_name.isin(["left_shoulder_roll_joint", "right_shoulder_roll_joint", "left_wrist_yaw_joint", "right_wrist_yaw_joint"])]
    key = key[key.experiment.isin(["free_baseline", "fixed_base_baseline", "fixed_base_50hz_zoh", "free_reference_advance_030"])]
    delay_rows = [[r.experiment, r.joint_name, fmt(r.phase_lag_candidate_s,2), fmt(r.rmse_rad), fmt(r.maximum_ctrl_saturation_fraction,3)] for r in key.itertuples()]
    delay_report = f"""# Phase 2G replay tracking-delay decomposition

{table(["experiment", "joint", "lag (s)", "RMSE (rad)", "max saturation fraction"], delay_rows)}

## Finding

Fixed-base reproduces the free-base lag: shoulder roll remains about 0.24 s and wrist yaw about 0.38 s, with no difference at the 20 ms analysis resolution. The 50 Hz zero-order-hold test adds only one 20 ms sample (0.26/0.40 s), as expected for sample-and-hold, and does not explain the original 0.24/0.38 s lag. This prioritizes the simulation controller/actuator-following pipeline over free-base balance coupling or reference update rate.

The baseline uses piecewise-linear 50 Hz data evaluated every 1 ms physics/control step. `SimulationStabilityController` has no explicit command filter or velocity limiter. Control saturation is not active (all tested fractions remain well below 1). The remaining mechanism is the simulated joint PD/inertia/damping/friction response; wrist kp=12 and shoulder kp=38 are simulation controller settings, not hardware gains.

A common 0.30 s reference schedule advance changes lag against the original real timeline to approximately -0.06 s for shoulder roll and +0.08 s for wrist yaw and substantially reduces RMSE, but cannot align both joint families simultaneously. It is a `SIM_CONTROLLER_ALIGNMENT_CANDIDATE`, not a physical delay estimate and not hardware calibration.
"""
    (CAL / "phase2g_tracking_delay_report.md").write_text(delay_report, encoding="utf-8")

    exp_rows = [[r.experiment, r.changed_category, fmt(r.mean_arm_rmse_rad), fmt(r.median_arm_phase_lag_s,2), fmt(r.mean_balance_delta_rmse), r.stable_no_fall, r.status] for r in experiments.itertuples()]
    alignment_report = f"""# Phase 2G simulation controller alignment experiments

Every row is **NOT HARDWARE CALIBRATION**. No mass, inertia, friction, gear, torque limit, MJCF dynamics, or hardware mapping was changed; each experiment changes one simulation controller/reference category.

{table(["experiment", "single changed category", "mean arm RMSE", "median arm lag", "mean balance RMSE", "stable", "status"], exp_rows)}

- `free_reference_advance_030`: useful timing candidate, but one common advance over-corrects shoulders and under-corrects wrists.
- `free_balance_gain_scale_060`: reduces excessive ankle/hip/torso pitch response, but under-shoots left ankle excursion and does not resolve knee mismatch.
- `free_equilibrium_target_compensation`: improves knee absolute equilibrium but worsens ankle equilibrium; it exposes controller equilibrium, not robot zero.
- No candidates were combined or promoted to the current model/controller.
"""
    (CAL / "phase2g_sim_controller_alignment_report.md").write_text(alignment_report, encoding="utf-8")

    ankle = balance[balance.quantity.isin(["left_ankle_pitch_joint", "right_ankle_pitch_joint"])]
    ankle = ankle[ankle.experiment.isin(["free_baseline", "free_balance_gain_scale_060"])]
    bal_rows = [[r.experiment, r.quantity, fmt(r.real_excursion), fmt(r.sim_excursion), fmt(r.excursion_ratio,3), fmt(r.delta_rmse), fmt(r.peak_timing_difference_s,2), fmt(r.phase_lag_candidate_s,2), fmt(r.real_recovery_s,2), fmt(r.sim_recovery_s,2)] for r in ankle.itertuples()]
    balance_report = f"""# Phase 2G balance-response comparison rerun

{table(["experiment", "quantity", "real excursion", "sim excursion", "ratio", "RMSE", "peak Δt", "phase lag", "real recovery", "sim recovery"], bal_rows)}

The earlier left ankle result is reproduced: real 0.04621 rad versus baseline sim 0.07313 rad (ratio 1.582). Scaling only the simulation balance gains to 0.60 produces 0.02934 rad (ratio 0.635) and reduces relative-response RMSE from 0.02748 to 0.01602 rad. The direction of improvement shows that controller alignment explains a material portion of the excessive baseline response, but the candidate over-corrects excursion and does not validate physical dynamics.

Right ankle improves from ratio 2.254 to 0.873. Hip pitch and torso pitch metrics also improve, while knee excursion remains excessive and waist roll remains under-responsive. Full metrics, including peak timing, recovery, phase lag, torso relative roll/pitch and gyro norm, are in `phase2g_balance_metrics.csv`. Torso axes remain `PARTIAL_TRANSFORM`; gyro is compared as a norm only.
"""
    (CAL / "phase2g_balance_response_rerun.md").write_text(balance_report, encoding="utf-8")

    sign_plan = """# Phase 2G hardware sign / zero verification plan

Status: **PLAN ONLY — NO MAPPING UPGRADE**.

## Existing evidence

- Live `JointState.name` values provide a name-level mapping.
- Historical heart motion provides dynamic left/right mirror evidence for J2/J7 and other moving arm joints, but it does not define the MuJoCo physical positive axis or encoder zero.
- `STAND_DEFAULT` is checked before/after preset 1007. Neither the inspected wrapper nor the captured MC state exposes a numeric joint specification for this pose.
- Heart endpoints are measured/API-preset outcomes, not documented mechanical calibration targets.

## Passive evidence sequence

1. Obtain manufacturer or deployed MC source specifying numeric joint targets for a named `home`, `neutral`, `calibration`, or `STAND_DEFAULT` pose, including coordinate convention and firmware applicability.
2. Hash and record that source; capture JointState while an operator has independently placed/confirmed the robot in exactly that already-supported pose. Codex sends no motion.
3. For each fitted joint, compare specified physical angle to measured position. A single nonzero known pose can provide an offset candidate; at least two distinct known poses or a physical direction observation are required to confirm sign and scale.
4. Confirm left/right convention separately; symmetry alone is not a global-axis definition.
5. Promote sign/zero only per joint with source plus physical-pose evidence. Keep encoder offset separate from an MC balance/posture bias.

## Required physical verification

An operator must identify the physical landmark/fixture for zero, confirm pose tolerance, E-stop and passive observation conditions, and verify the deployed firmware uses the cited coordinate definition. Until then, sign/zero remain UNKNOWN or FIELD_TEST_EVIDENCE only.
"""
    (CAL / "phase2g_sign_zero_verification_plan.md").write_text(sign_plan, encoding="utf-8")

    effort_report = """# Phase 2G JointState effort semantics

Final classification: **UNKNOWN**.

| Evidence layer | Finding | What it does not prove |
|---|---|---|
| `aimdk_msgs/msg/JointState` schema | `effort` is `double` | origin or estimator |
| AimDK documentation already captured | labeled torque, unit N·m | measured vs estimated vs commanded vs current-derived |
| live graph | state publisher is EtherCAT HAL (`/hal_ethercat_x21436`) | assignment source |
| FF SDK/application source | consumes the value as reported/measured effort and applies safety thresholds | HAL meaning; downstream naming is not provenance |
| Phase 2D response | nonzero, time-varying output correlated to motion for some joints | torque source or sign convention |

No inspected official documentation or publisher/HAL source shows the assignment to `JointState.effort`. Therefore it cannot be classified as `MEASURED_TORQUE`, `ESTIMATED_TORQUE`, `COMMANDED_TORQUE`, `CURRENT_DERIVED_TORQUE`, or `OTHER`. It remains excluded from torque calibration; `phase2f_effort_qualitative.csv` is response-shape evidence only.
"""
    (CAL / "phase2g_effort_semantics_report.md").write_text(effort_report, encoding="utf-8")

    mc_report = """# Phase 2G MC command observability

Final status: **MC_INTERNAL_COMMAND = UNOBSERVABLE** with current evidence.

- Readable and captured: `/aima/mc/common/state`, including `input_source=app_proxy`, `STAND_DEFAULT`, FSM/body/status fields. It did not expose per-joint q(t) during heart.
- Readable by the validated wrapper when task ID is known: `GetMcPresetMotionState`. It exposes preset task state, not the internal joint trajectory.
- Present in the Phase 2D graph but not captured/decoded as command targets: `/aima/mc/body_pose`, `/aima/mc/manipulation`, `/aima/mc/rl/debug`. Topic names and types are insufficient to assert they contain heart joint targets.
- HAL `/aima/hal/joint/*/command` topics exist, but no evidence proves a passive subscription is an authoritative MC internal target, nor distinguishes competing publishers/arbitration. They were not used as Phase 2D command truth.
- No debug mode was enabled and no state-changing service/action was called.

Preset execution state is partially observable; the MC per-joint reference is not. Later work may be called `OUTPUT_RESPONSE_CALIBRATION` if it uses known preset/output measurements. It must not be called `ACTUATOR_SYSTEM_IDENTIFICATION` without an observable, time-aligned actuator input.
"""
    (CAL / "phase2g_mc_command_observability.md").write_text(mc_report, encoding="utf-8")

    readiness_rows = [
        ["joint name mapping", "READY", "live JointState.name decoded; exact-name candidate exists"],
        ["joint physical sign", "PARTIAL", "mirror FIELD_TEST_EVIDENCE; MuJoCo physical sign unconfirmed"],
        ["joint zero / encoder offset", "BLOCKED", "no documented known physical pose plus measurement"],
        ["IMU transform", "PARTIAL", "frame_id and upstream URDF known; deployed TF/convention unconfirmed"],
        ["real position", "READY", "decoded 47 Hz Phase 2D capture"],
        ["real velocity", "READY", "decoded 47 Hz Phase 2D capture"],
        ["reported effort semantics", "BLOCKED", "N·m label only; HAL assignment source absent"],
        ["MC internal command", "BLOCKED", "preset state visible; joint reference UNOBSERVABLE"],
        ["sim controller alignment", "PARTIAL", "cause separation improved; candidates not adopted"],
        ["contact baseline", "PARTIAL", "sim contact stable; no real foot contact/force baseline"],
        ["balance baseline", "PARTIAL", "real/sim relative response exists; IMU axes not transformed"],
    ]
    readiness = f"""# Phase 2G calibration readiness

{table(["item", "status", "basis"], readiness_rows)}

## Gate

**DYNAMICS_CALIBRATION_READY = NO**

## Minimum conditions for Phase 3

1. Confirm a common IMU comparison frame from deployed TF/driver documentation or a validated static transform.
2. Confirm physical sign and zero for every joint selected for fitting; do not infer these from replay error.
3. Select and validate a simulation controller timing/alignment policy so artificial tracking lag is either materially reduced or explicitly modeled.
4. Treat standing target/balance equilibrium separately from encoder zero; adopt no global target compensation until ankle/knee trade-offs are resolved.
5. If effort is used for torque calibration, confirm its HAL assignment semantics and sign. Otherwise exclude torque fitting.
6. If MC internal q-command remains unavailable, limit Phase 3 to `OUTPUT_RESPONSE_CALIBRATION`; do not claim `ACTUATOR_SYSTEM_IDENTIFICATION`.

All Phase 2G candidates are `SIM_CONTROLLER_ALIGNMENT_CANDIDATE` and **NOT HARDWARE CALIBRATION**. No calibrated MJCF was created.
"""
    (CAL / "phase2g_calibration_readiness.md").write_text(readiness, encoding="utf-8")


def main() -> int:
    PLOTS.mkdir(parents=True, exist_ok=True)
    imu_obs = imu_transform_observation()
    imu_cmp = imu_comparisons()
    equilibrium = equilibrium_table()
    tracking = tracking_metrics()
    balance = balance_metrics()
    experiments = experiment_summary(tracking, balance, equilibrium)
    imu_obs.to_csv(HERE / "phase2g_imu_transform_observations.csv", index=False)
    imu_cmp.to_csv(HERE / "phase2g_imu_relative_comparison.csv", index=False)
    equilibrium.to_csv(HERE / "phase2g_equilibrium_delta.csv", index=False)
    tracking.to_csv(HERE / "phase2g_tracking_metrics.csv", index=False)
    balance.to_csv(HERE / "phase2g_balance_metrics.csv", index=False)
    experiments.to_csv(HERE / "phase2g_experiment_summary.csv", index=False)
    candidate = {
        "status": "SIM_CONTROLLER_ALIGNMENT_CANDIDATE",
        "warning": "NOT HARDWARE CALIBRATION",
        "physical_parameters_modified": False,
        "candidates": [
            {"category": "reference timing", "value": {"common_reference_advance_s": 0.30}, "result": "reduces wrist/shoulder lag but cannot align both exactly"},
            {"category": "simulation balance gains", "value": {"gain_scale": 0.60}, "result": "reduces ankle/hip/torso pitch over-response; over-corrects left ankle"},
            {"category": "standing equilibrium targets", "value": "per-joint one-shot baseline correction in experiment log", "result": "reduces knee absolute offset; worsens ankle equilibrium"},
        ],
        "adopted": False,
    }
    (CAL / "simulation_controller_alignment_candidate.json").write_text(json.dumps(candidate, indent=2), encoding="utf-8")
    pd.DataFrame(candidate["candidates"]).to_csv(CAL / "simulation_controller_alignment_candidate.csv", index=False)
    make_plots(tracking, balance, equilibrium)
    write_reports(imu_obs, imu_cmp, equilibrium, tracking, balance, experiments)
    print(json.dumps({
        "imu_transform": "PARTIAL_TRANSFORM",
        "mc_internal_command": "UNOBSERVABLE",
        "effort_semantics": "UNKNOWN",
        "dynamics_calibration_ready": "NO",
        "reports_written": 9,
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
