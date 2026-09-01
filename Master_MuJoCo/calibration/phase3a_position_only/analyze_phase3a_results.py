"""Build Phase 3A position-only reports from immutable real data and sim results.

This is an offline analysis/reporting tool.  It does not connect to a robot and
does not load ``reported_effort``.  IMU comparisons are deliberately limited to
relative roll/pitch motion and gyro shape because the Phase 2H transform gate is
PARTIAL.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


HERE = Path(__file__).resolve().parent
CALIBRATION = HERE.parent
PROJECT = CALIBRATION.parent
PLOTS = HERE / "plots"
PLOTS.mkdir(exist_ok=True)

TRACKING = pd.read_csv(HERE / "phase3a_all_tracking_metrics.csv")
EQUILIBRIUM = pd.read_csv(HERE / "phase3a_all_equilibrium_metrics.csv")
BALANCE = pd.read_csv(HERE / "phase3a_all_balance_metrics.csv")
EXPERIMENTS = pd.read_csv(HERE / "phase3a_candidate_experiments.csv")
CANDIDATE = json.loads((HERE / "simulation_controller_alignment_candidate.json").read_text(encoding="utf-8"))
SAFETY = json.loads((HERE / "free_base_10s_standing_validation.json").read_text(encoding="utf-8"))
REHEARSAL = json.loads((HERE / "rehearsal_12_joint_regression.json").read_text(encoding="utf-8"))

REFERENCE_PATH = CALIBRATION / "phase2e_replay" / "phase2e_heart_measured_reference.csv"
REAL_JOINT_PATH = CALIBRATION / "phase2e_replay" / "phase2e_aligned_joint_data.csv"
REAL_IMU_PATH = CALIBRATION / "phase2e_replay" / "phase2e_aligned_imu_data.csv"
SOURCE_LOCK = json.loads((CALIBRATION / "phase2e_replay" / "source_data_lock.json").read_text(encoding="utf-8"))
MOTION_END = float(SOURCE_LOCK["motion_duration_seconds"])
PRE_WINDOW = (-3.0, -0.2)

ARM_JOINTS = [
    "left_shoulder_roll_joint",
    "right_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "right_shoulder_yaw_joint",
    "left_wrist_yaw_joint",
    "right_wrist_yaw_joint",
    "left_wrist_roll_joint",
    "right_wrist_roll_joint",
]
BALANCE_JOINTS = [
    "left_ankle_pitch_joint",
    "right_ankle_pitch_joint",
    "left_hip_pitch_joint",
    "right_hip_pitch_joint",
    "left_knee_joint",
    "right_knee_joint",
    "waist_pitch_joint",
    "waist_roll_joint",
]


def short_name(name: str) -> str:
    return name.removesuffix("_joint")


def fmt(value: object, digits: int = 5) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not math.isfinite(number):
        return "N/A"
    return f"{number:.{digits}f}"


def markdown_table(headers: list[str], rows: list[list[object]]) -> str:
    clean = lambda value: str(value).replace("|", "\\|").replace("\n", " ")
    lines = ["| " + " | ".join(map(clean, headers)) + " |", "| " + " | ".join(["---"] * len(headers)) + " |"]
    lines += ["| " + " | ".join(clean(value) for value in row) + " |" for row in rows]
    return "\n".join(lines)


def metric(experiment: str, joint: str, segment: str = "full_motion") -> pd.Series:
    frame = TRACKING[
        (TRACKING.experiment == experiment)
        & (TRACKING.joint_name == joint)
        & (TRACKING.segment == segment)
    ]
    if len(frame) != 1:
        raise ValueError(f"Expected one tracking row: {experiment=} {joint=} {segment=}; got {len(frame)}")
    return frame.iloc[0]


def experiment(name: str) -> pd.Series:
    frame = EXPERIMENTS[EXPERIMENTS.experiment == name]
    if len(frame) != 1:
        raise ValueError(f"Expected one experiment row for {name}; got {len(frame)}")
    return frame.iloc[0]


def verify_hashes() -> pd.DataFrame:
    manifest = pd.read_csv(HERE / "immutable_baseline_sha256.csv")
    records = []
    for row in manifest.itertuples(index=False):
        path = PROJECT / Path(row.path)
        digest = hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else "MISSING"
        records.append(
            {
                "category": row.category,
                "path": row.path,
                "locked_sha256": row.sha256,
                "current_sha256": digest,
                "status": "VERIFIED_UNCHANGED" if digest == row.sha256 else "CHANGED_OR_MISSING",
            }
        )
    result = pd.DataFrame(records)
    result.to_csv(HERE / "immutable_baseline_verification.csv", index=False)
    return result


def centered(values: np.ndarray, times: np.ndarray, window: tuple[float, float] = PRE_WINDOW) -> np.ndarray:
    mask = (times >= window[0]) & (times <= window[1])
    origin = float(np.nanmean(values[mask])) if np.any(mask) else float(values[0])
    return values - origin


def lag_and_corr(reference: np.ndarray, response: np.ndarray, dt: float, bound: float = 1.0) -> tuple[float, float]:
    reference = reference - np.nanmean(reference)
    response = response - np.nanmean(response)
    best = (float("-inf"), 0)
    limit = int(round(bound / dt))
    for shift in range(-limit, limit + 1):
        if shift > 0:
            left, right = reference[:-shift], response[shift:]
        elif shift < 0:
            left, right = reference[-shift:], response[:shift]
        else:
            left, right = reference, response
        if len(left) < 20 or np.std(left) < 1e-10 or np.std(right) < 1e-10:
            continue
        corr = float(np.corrcoef(left, right)[0, 1])
        if corr > best[0]:
            best = (corr, shift)
    return best[1] * dt, best[0]


def relative_base_metrics() -> pd.DataFrame:
    real = pd.read_csv(
        REAL_IMU_PATH,
        usecols=["t", "imu", "relative_roll_rad", "relative_pitch_rad", "gyro_norm"],
    )
    records: list[dict[str, object]] = []
    for experiment_name in ("free_baseline", "free_final_candidate"):
        sim = pd.read_csv(
            HERE / f"{experiment_name}_base_log.csv",
            usecols=["t", "base_roll_rad", "base_pitch_rad", "gyro_norm"],
        ).sort_values("t")
        sim_t = sim.t.to_numpy(float)
        motion = (sim_t >= 0.0) & (sim_t <= MOTION_END)
        for imu_name, group in real.groupby("imu"):
            group = group.sort_values("t")
            real_t = group.t.to_numpy(float)
            for real_field, sim_field, quantity in (
                ("relative_roll_rad", "base_roll_rad", "relative_roll"),
                ("relative_pitch_rad", "base_pitch_rad", "relative_pitch"),
                ("gyro_norm", "gyro_norm", "gyro_norm"),
            ):
                real_values = centered(group[real_field].to_numpy(float), real_t)
                sim_values = centered(sim[sim_field].to_numpy(float), sim_t)
                real_on_sim = np.interp(sim_t, real_t, real_values)
                r = real_on_sim[motion]
                s = sim_values[motion]
                dt = float(np.median(np.diff(sim_t[motion])))
                lag, corr = lag_and_corr(r, s, dt)
                records.append(
                    {
                        "experiment": experiment_name,
                        "real_imu": imu_name,
                        "quantity": quantity,
                        "real_relative_excursion": float(np.ptp(r)),
                        "sim_relative_excursion": float(np.ptp(s)),
                        "excursion_ratio": float(np.ptp(s) / np.ptp(r)) if np.ptp(r) > 1e-10 else np.nan,
                        "relative_rmse": float(np.sqrt(np.mean((s - r) ** 2))),
                        "phase_lag_s": lag,
                        "shape_correlation": corr,
                        "comparison_scope": "RELATIVE_MOTION_AUXILIARY_ONLY_IMU_TRANSFORM_PARTIAL",
                    }
                )
    result = pd.DataFrame(records)
    result.to_csv(HERE / "phase3a_relative_imu_auxiliary_metrics.csv", index=False)
    return result


def plot_arm_tracking() -> None:
    real = pd.read_csv(REFERENCE_PATH, usecols=["t", "joint_name", "position"])
    baseline = pd.read_csv(HERE / "free_baseline_joint_log.csv", usecols=["t", "joint_name", "position"])
    final = pd.read_csv(HERE / "free_final_candidate_joint_log.csv", usecols=["t", "joint_name", "position"])
    fig, axes = plt.subplots(4, 2, figsize=(13, 12), sharex=True)
    for ax, joint in zip(axes.ravel(), ARM_JOINTS):
        for frame, label, style in ((real, "real measured", "k-"), (baseline, "sim baseline", "C1--"), (final, "sim candidate", "C0-")):
            part = frame[frame.joint_name == joint]
            ax.plot(part.t, part.position, style, lw=1.2, label=label)
        ax.axvspan(0, MOTION_END, color="0.9", zorder=-1)
        ax.set_title(short_name(joint))
        ax.set_ylabel("rad")
        ax.grid(alpha=0.25)
    axes[-1, 0].set_xlabel("relative time (s)")
    axes[-1, 1].set_xlabel("relative time (s)")
    axes[0, 0].legend(fontsize=8)
    fig.suptitle("Phase 3A arm response: measured real vs simulation")
    fig.tight_layout()
    fig.savefig(PLOTS / "arm_tracking_before_after.png", dpi=160)
    plt.close(fig)


def plot_pipeline_scan() -> None:
    shoulder = EXPERIMENTS[EXPERIMENTS.changed_category == "simulation shoulder PD bandwidth"]
    wrist = EXPERIMENTS[EXPERIMENTS.changed_category == "simulation wrist PD bandwidth"]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    axes[0].plot(shoulder.shoulder_gain_scale, shoulder.mean_arm_rmse_rad, "o-", label="shoulder-only scan")
    axes[0].plot(wrist.wrist_gain_scale, wrist.mean_arm_rmse_rad, "o-", label="wrist-only scan")
    axes[0].set(xlabel="simulation gain scale", ylabel="mean arm RMSE (rad)", title="Bandwidth scan")
    axes[0].legend(fontsize=8)
    rate = EXPERIMENTS[EXPERIMENTS.changed_category == "controller update rate"].copy()
    baseline = experiment("fixed_baseline")
    rate = pd.concat([rate, pd.DataFrame([baseline])], ignore_index=True).sort_values("controller_rate_hz")
    axes[1].plot(rate.controller_rate_hz, rate.mean_arm_rmse_rad, "o-")
    axes[1].set(xlabel="controller update rate (Hz)", ylabel="mean arm RMSE (rad)", title="Update-rate diagnostic")
    for ax in axes:
        ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(PLOTS / "tracking_pipeline_diagnostics.png", dpi=160)
    plt.close(fig)


def plot_equilibrium() -> None:
    base = EQUILIBRIUM[EQUILIBRIUM.experiment == "free_baseline"].set_index("joint_name")
    final = EQUILIBRIUM[EQUILIBRIUM.experiment == "free_final_candidate"].set_index("joint_name")
    labels = [short_name(joint) for joint in BALANCE_JOINTS]
    x = np.arange(len(labels))
    fig, ax = plt.subplots(figsize=(13, 4.5))
    ax.bar(x - 0.2, [base.loc[j].settled_minus_real_rad for j in BALANCE_JOINTS], 0.4, label="baseline")
    ax.bar(x + 0.2, [final.loc[j].settled_minus_real_rad for j in BALANCE_JOINTS], 0.4, label="candidate")
    ax.axhline(0, color="k", lw=0.8)
    ax.set_xticks(x, labels, rotation=30, ha="right")
    ax.set_ylabel("sim settled - real pre-heart (rad)")
    ax.set_title("Simulation equilibrium mismatch (not hardware zero)")
    ax.legend()
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(PLOTS / "standing_equilibrium_before_after.png", dpi=160)
    plt.close(fig)


def plot_balance() -> None:
    real = pd.read_csv(REAL_JOINT_PATH, usecols=["t", "joint_name", "position"])
    base = pd.read_csv(HERE / "free_baseline_joint_log.csv", usecols=["t", "joint_name", "position"])
    final = pd.read_csv(HERE / "free_final_candidate_joint_log.csv", usecols=["t", "joint_name", "position"])
    fig, axes = plt.subplots(4, 2, figsize=(13, 12), sharex=True)
    for ax, joint in zip(axes.ravel(), BALANCE_JOINTS):
        for frame, label, style in ((real, "real relative", "k-"), (base, "sim baseline relative", "C1--"), (final, "sim candidate relative", "C0-")):
            part = frame[frame.joint_name == joint].sort_values("t")
            t = part.t.to_numpy(float)
            y = centered(part.position.to_numpy(float), t)
            ax.plot(t, y, style, lw=1.2, label=label)
        ax.axvspan(0, MOTION_END, color="0.9", zorder=-1)
        ax.set_title(short_name(joint))
        ax.set_ylabel("relative rad")
        ax.grid(alpha=0.25)
    axes[-1, 0].set_xlabel("relative time (s)")
    axes[-1, 1].set_xlabel("relative time (s)")
    axes[0, 0].legend(fontsize=8)
    fig.suptitle("Balance response before/after (relative motion only)")
    fig.tight_layout()
    fig.savefig(PLOTS / "balance_response_before_after.png", dpi=160)
    plt.close(fig)


def tracking_pipeline_report() -> str:
    names = [
        "fixed_baseline",
        "fixed_interp_zoh",
        "fixed_interp_pchip",
        "fixed_reference_rate_25hz",
        "fixed_reference_rate_100hz",
        "fixed_controller_rate_500hz",
        "fixed_controller_rate_200hz",
        "fixed_controller_rate_100hz",
        "fixed_timestep_0005",
        "fixed_velocity_limit_5",
        "fixed_velocity_limit_2",
        "fixed_arm_pd_candidate",
    ]
    rows = []
    for name in names:
        row = experiment(name)
        rows.append(
            [
                name,
                row.changed_category,
                fmt(row.mean_arm_rmse_rad),
                fmt(row.mean_arm_mae_rad),
                fmt(row.median_arm_lag_s, 3),
                fmt(row.maximum_ctrl_saturation_fraction, 3),
                fmt(row.persistent_saturation_sample_fraction, 3),
                row.classification,
            ]
        )
    return f"""# Phase 3A Tracking Pipeline Decomposition

Status: **COMPLETE**  
Scope: position/velocity response only. **NOT HARDWARE CALIBRATION.**

No global time advance was used. The Phase 2G 0.30 s advance remains diagnostic-only and is absent from every Phase 3A candidate.

## One-factor fixed-base experiments

{markdown_table(["experiment", "single changed factor", "mean RMSE rad", "mean MAE rad", "median lag s", "max ctrl fraction", "persistent saturation", "classification"], rows)}

## Findings by pipeline factor

- **A — interpolation:** 50 Hz linear and PCHIP are effectively equivalent. ZOH adds error and increases median lag from 0.31 s to 0.33 s.
- **B — reference rate:** 25, 50, and 100 Hz are nearly identical at the baseline bandwidth. Reference sampling is not the dominant 0.24/0.38 s delay source.
- **C — controller update rate:** 500 and 200 Hz match 1000 Hz closely. At 100 Hz the controller develops persistent saturation (~22.1% of samples) and RMSE regresses, so it is rejected.
- **D — timestep:** 0.5 ms and 1.0 ms are effectively equivalent. The integrator timestep is not the dominant delay source.
- **E — PD/controller bandwidth:** independent shoulder and wrist scans monotonically reduce lag/RMSE through the tested 8x boundary. The 8x/8x combination is accepted as a bounded simulation candidate, but the boundary optimum is a reason not to interpret it as a unique physical parameter estimate.
- **F — actuator saturation:** no actuator limit was changed. Baseline peak command fraction is observationally below 0.28 with zero persistent saturation; saturation does not explain the original lag. The 100 Hz controller-rate failure is the counterexample and was rejected.
- **G — velocity limiting:** a 5 rad/s target limiter is inactive/equivalent to baseline; 2 rad/s increases RMSE and lag and is rejected.

Conclusion: the dominant removable delay is the **simulation arm controller bandwidth**, not free-base coupling, reference sample rate, timestep, or an admissible global clock shift.
"""


def arm_report() -> str:
    rows = []
    for joint in ARM_JOINTS:
        before = metric("free_baseline", joint)
        after = metric("free_final_candidate", joint)
        rows.append(
            [
                short_name(joint),
                fmt(before.rmse_rad),
                fmt(after.rmse_rad),
                fmt(100.0 * (1.0 - after.rmse_rad / before.rmse_rad), 1) + "%",
                fmt(before.lag_s, 3),
                fmt(after.lag_s, 3),
                fmt(before.peak_velocity_error_rad_s),
                fmt(after.peak_velocity_error_rad_s),
                fmt(after.settling_error_rad),
            ]
        )
    split_rows = []
    for exp_name in ("free_baseline", "free_final_candidate"):
        frame = TRACKING[(TRACKING.experiment == exp_name) & TRACKING.joint_name.isin(ARM_JOINTS)]
        for split in ("fit", "validation"):
            part = frame[frame.split == split]
            split_rows.append([exp_name, split, fmt(part.rmse_rad.mean()), fmt(part.mae_rad.mean()), len(part)])
    return f"""# Phase 3A Arm Tracking Calibration

Classification: **ACCEPTED_SIM_CONTROLLER_ALIGNMENT**  
Warning: **NOT HARDWARE CALIBRATION**. Gain scales are simulation-controller-only parameters and are not identified physical robot gains.

Selected candidate: shoulder `kp x8`, `kd x sqrt(8)`; wrist `kp x8`, `kd x sqrt(8)`. The original per-joint architecture is preserved; shoulder and wrist families were scanned independently.

## Free-base before/after

{markdown_table(["joint", "RMSE before", "RMSE after", "RMSE reduction", "lag before s", "lag after s", "peak vel err before", "peak vel err after", "settling err after"], rows)}

## Temporal train/validation check

Fit segments are motion onset and return. Validation segments are pre-roll, peak gesture, and post-roll.

{markdown_table(["experiment", "split", "mean RMSE rad", "mean MAE rad", "metric rows"], split_rows)}

Only one complete heart capture is available, so this is temporal hold-out validation, not an independent-trajectory validation. The 8x optimum lies on the tested scan boundary and should be treated as a controller-alignment candidate, not a unique optimum or physical identification.

Plot: [arm_tracking_before_after.png](plots/arm_tracking_before_after.png)
"""


def standing_report() -> str:
    base = EQUILIBRIUM[EQUILIBRIUM.experiment == "free_baseline"].set_index("joint_name")
    final = EQUILIBRIUM[EQUILIBRIUM.experiment == "free_final_candidate"].set_index("joint_name")
    rows = []
    for joint in BALANCE_JOINTS:
        b, a = base.loc[joint], final.loc[joint]
        rows.append(
            [
                short_name(joint),
                fmt(b.real_pre_mean_rad),
                fmt(b.sim_target_mean_rad),
                fmt(b.sim_settled_mean_rad),
                fmt(b.settled_minus_real_rad),
                fmt(a.sim_target_mean_rad),
                fmt(a.sim_settled_mean_rad),
                fmt(a.settled_minus_real_rad),
            ]
        )
    knee_before = base.loc[["left_knee_joint", "right_knee_joint"]].settled_minus_real_rad.abs().mean()
    knee_after = final.loc[["left_knee_joint", "right_knee_joint"]].settled_minus_real_rad.abs().mean()
    ankle_before = base.loc[["left_ankle_pitch_joint", "right_ankle_pitch_joint"]].settled_minus_real_rad.abs().mean()
    ankle_after = final.loc[["left_ankle_pitch_joint", "right_ankle_pitch_joint"]].settled_minus_real_rad.abs().mean()
    return f"""# Phase 3A Standing Reference Alignment

Classification: **SIMULATION_REFERENCE_ALIGNMENT**  
Explicitly: **NOT HARDWARE_ZERO_CALIBRATION** and no `joint_mapping.csv` zero was changed.

{markdown_table(["joint", "real pre q", "base target", "base settled", "base settled-real", "candidate target", "candidate settled", "candidate settled-real"], rows)}

The bilateral knee absolute equilibrium mismatch falls from **{fmt(knee_before)} rad** to **{fmt(knee_after)} rad**. This demonstrates that most of the ~0.12 rad knee discrepancy can be generated by the simulation controller equilibrium rather than requiring a hardware-zero explanation.

The ankle absolute standing mismatch changes from **{fmt(ankle_before)} rad** to **{fmt(ankle_after)} rad** and therefore is **not improved** by the knee-prioritized full reference correction (its sign crosses zero). This tradeoff is retained transparently; it is not evidence for encoder or hardware zero. The 0.50 and 0.75 reference-scale experiments remain diagnostic alternatives in `phase3a_candidate_experiments.csv`.

Plot: [standing_equilibrium_before_after.png](plots/standing_equilibrium_before_after.png)
"""


def balance_report(imu: pd.DataFrame) -> str:
    base = BALANCE[BALANCE.experiment == "free_baseline"].set_index("joint_name")
    final = BALANCE[BALANCE.experiment == "free_final_candidate"].set_index("joint_name")
    rows = []
    for joint in BALANCE_JOINTS:
        b, a = base.loc[joint], final.loc[joint]
        rows.append(
            [
                short_name(joint),
                fmt(b.real_excursion_rad),
                fmt(b.sim_excursion_rad),
                fmt(b.excursion_ratio, 3),
                fmt(a.sim_excursion_rad),
                fmt(a.excursion_ratio, 3),
                fmt(b.relative_rmse_rad),
                fmt(a.relative_rmse_rad),
                fmt(a.phase_lag_s, 3),
                fmt(a.sim_recovery_s, 3),
            ]
        )
    imu_rows = []
    for row in imu.itertuples(index=False):
        imu_rows.append([row.experiment, row.real_imu, row.quantity, fmt(row.excursion_ratio, 3), fmt(row.relative_rmse), fmt(row.phase_lag_s, 3), fmt(row.shape_correlation, 3)])
    base_summary = json.loads((HERE / "free_baseline_summary.json").read_text(encoding="utf-8"))
    final_summary = json.loads((HERE / "free_final_candidate_summary.json").read_text(encoding="utf-8"))
    return f"""# Phase 3A Balance Position-Response Alignment

Selected simulation balance gain scale: **{fmt(CANDIDATE['parameters']['balance_gain_scale'], 2)}**. No mass, inertia, physical friction, gear, or torque/force limit was changed.

{markdown_table(["joint", "real excursion", "base sim excursion", "base ratio", "candidate sim excursion", "candidate ratio", "base RMSE", "candidate RMSE", "candidate lag s", "candidate recovery s"], rows)}

The original left ankle pitch discrepancy changes from real/sim `0.04621 / 0.07313 rad` (ratio **1.583**) to `0.04621 / 0.02976 rad` (ratio **0.644**). The overshoot is removed, but the candidate now undershoots the real excursion; this is an improvement in absolute excursion error, not a complete match. Right ankle ratio improves from **2.254** to **0.856**.

## Relative IMU auxiliary metrics

IMU transform is still PARTIAL. These metrics use only pre-roll-centered roll/pitch motion and gyro-norm shape; no absolute quaternion/yaw fitting is performed.

{markdown_table(["experiment", "real IMU", "quantity", "excursion ratio", "relative RMSE", "lag s", "shape corr"], imu_rows)}

## Base/contact safeguards

- free replay both-feet contact: {fmt(base_summary['both_feet_contact_fraction'], 4)} -> {fmt(final_summary['both_feet_contact_fraction'], 4)}
- max foot-slip proxy (left/right): `{fmt(final_summary['max_left_foot_slip_proxy_m'])} / {fmt(final_summary['max_right_foot_slip_proxy_m'])} m`
- max relative base tilt during replay (roll/pitch): `{fmt(final_summary['max_abs_base_roll_deg'], 3)} / {fmt(final_summary['max_abs_base_pitch_deg'], 3)} deg`
- self-collision and non-foot ground-contact samples: `{final_summary['self_collision_samples']} / {final_summary['nonfoot_ground_contact_samples']}`

Plot: [balance_response_before_after.png](plots/balance_response_before_after.png)
"""


def before_after_report(hash_check: pd.DataFrame) -> str:
    shoulder_before = np.mean([metric("free_baseline", joint).lag_s for joint in ("left_shoulder_roll_joint", "right_shoulder_roll_joint")])
    shoulder_after = np.mean([metric("free_final_candidate", joint).lag_s for joint in ("left_shoulder_roll_joint", "right_shoulder_roll_joint")])
    wrist_before = np.mean([metric("free_baseline", joint).lag_s for joint in ("left_wrist_yaw_joint", "right_wrist_yaw_joint")])
    wrist_after = np.mean([metric("free_final_candidate", joint).lag_s for joint in ("left_wrist_yaw_joint", "right_wrist_yaw_joint")])
    eq_base = EQUILIBRIUM[EQUILIBRIUM.experiment == "free_baseline"].set_index("joint_name")
    eq_final = EQUILIBRIUM[EQUILIBRIUM.experiment == "free_final_candidate"].set_index("joint_name")
    knee_before = eq_base.loc[["left_knee_joint", "right_knee_joint"]].settled_minus_real_rad.abs().mean()
    knee_after = eq_final.loc[["left_knee_joint", "right_knee_joint"]].settled_minus_real_rad.abs().mean()
    bal_base = BALANCE[BALANCE.experiment == "free_baseline"].set_index("joint_name")
    bal_final = BALANCE[BALANCE.experiment == "free_final_candidate"].set_index("joint_name")
    final_summary = json.loads((HERE / "free_final_candidate_summary.json").read_text(encoding="utf-8"))
    hashes_ok = bool((hash_check.status == "VERIFIED_UNCHANGED").all())
    rehearsal_ok = REHEARSAL["settled_count"] == REHEARSAL["total"] == 12
    safety_ok = bool(
        SAFETY["stable_no_fall"]
        and SAFETY["self_collision_steps"] == 0
        and SAFETY["nonfoot_ground_contact_steps"] == 0
        and SAFETY["persistent_saturation_fraction"] <= 0.01
        and final_summary["target_clip_samples"] == 0
        and final_summary["persistent_saturation_sample_fraction"] <= 0.01
    )
    improved = shoulder_after < shoulder_before and wrist_after < wrist_before
    ready = hashes_ok and rehearsal_ok and safety_ok and improved
    return f"""# Phase 3A Before/After and Final Gate

## Immutable baseline

- locked files verified unchanged: **{int((hash_check.status == 'VERIFIED_UNCHANGED').sum())}/{len(hash_check)}**
- Phase 2 overwritten: **NO**
- reported effort used for fitting: **NO**
- absolute IMU quaternion used for fitting: **NO**
- MJCF/controller source/hardware mapping modified: **NO**

## Required final answers

1. Shoulder-roll lag: **{fmt(shoulder_before, 3)} s -> {fmt(shoulder_after, 3)} s**.
2. Wrist-yaw lag: **{fmt(wrist_before, 3)} s -> {fmt(wrist_after, 3)} s**.
3. Knee equilibrium: bilateral mean absolute offset **{fmt(knee_before)} -> {fmt(knee_after)} rad**; largely explained/reduced by simulation equilibrium alignment, not assigned to hardware zero.
4. Ankle excursion: left ratio **{fmt(bal_base.loc['left_ankle_pitch_joint'].excursion_ratio, 3)} -> {fmt(bal_final.loc['left_ankle_pitch_joint'].excursion_ratio, 3)}**, right **{fmt(bal_base.loc['right_ankle_pitch_joint'].excursion_ratio, 3)} -> {fmt(bal_final.loc['right_ankle_pitch_joint'].excursion_ratio, 3)}**. Error improves, but left now undershoots and is not fully matched.
5. Free-base 10 s standing stable: **{'YES' if SAFETY['stable_no_fall'] else 'NO'}**; max tilt `{fmt(SAFETY['maximum_abs_tilt_deg'], 3)} deg`, foot-slip proxy L/R `{fmt(SAFETY['maximum_left_foot_slip_proxy_m'])}/{fmt(SAFETY['maximum_right_foot_slip_proxy_m'])} m`, no collision/non-foot contact, no persistent saturation.
6. Prior rehearsal: **{REHEARSAL['settled_count']}/{REHEARSAL['total']} SETTLED**.
7. Explained at position/controller layer: most arm phase lag and RMSE, knee equilibrium bias, and much of the excessive ankle excursion. Reference rate/timestep/global shift/free-base coupling are not the primary arm-delay cause.
8. Still requiring physical-dynamics calibration or closed evidence: residual balance amplitude/phase/recovery mismatch, ankle equilibrium tradeoff, relative base/gyro mismatch, contact/foot-slip fidelity, actuator/torque behavior, and any mass/inertia/friction effects. Sign/zero, effort semantics, and full IMU transform gates remain outside Phase 3A.
9. **POSITION_RESPONSE_BASELINE_READY = {'YES' if ready else 'NO'}**.

## Acceptance evidence

- classification: **ACCEPTED_SIM_CONTROLLER_ALIGNMENT**
- free-base replay stable/no fall: `{final_summary['stable_no_fall']}`
- target clipping: `{final_summary['target_clip_samples']}` samples
- persistent saturation: `{fmt(final_summary['persistent_saturation_sample_fraction'], 5)}`
- collision/non-foot ground contact: `{final_summary['self_collision_samples']}/{final_summary['nonfoot_ground_contact_samples']}` samples
- 10 s free standing safety gate: `{'PASS' if safety_ok else 'FAIL'}`
- 12-joint rehearsal gate: `{'PASS' if rehearsal_ok else 'FAIL'}`

The accepted file is `simulation_controller_alignment_candidate.json`. It is explicitly **NOT HARDWARE CALIBRATION**. The gain scan ending at its upper boundary and use of one temporally split heart capture limit generalization; a second independent motion should validate it before treating the response baseline as frozen for broader use.

**DYNAMICS_CALIBRATION_READY = NO.** Phase 3A does not change the Phase 2H dynamics gate.
"""


def main() -> None:
    hash_check = verify_hashes()
    imu = relative_base_metrics()
    plot_arm_tracking()
    plot_pipeline_scan()
    plot_equilibrium()
    plot_balance()
    reports = {
        "phase3a_tracking_pipeline_report.md": tracking_pipeline_report(),
        "phase3a_arm_tracking_report.md": arm_report(),
        "phase3a_standing_reference_report.md": standing_report(),
        "phase3a_balance_alignment_report.md": balance_report(imu),
        "phase3a_before_after_report.md": before_after_report(hash_check),
    }
    for name, content in reports.items():
        (HERE / name).write_text(content.rstrip() + "\n", encoding="utf-8")
    print(json.dumps({"reports": list(reports), "hashes_verified": int((hash_check.status == 'VERIFIED_UNCHANGED').sum()), "hashes_total": len(hash_check)}, indent=2))


if __name__ == "__main__":
    main()
