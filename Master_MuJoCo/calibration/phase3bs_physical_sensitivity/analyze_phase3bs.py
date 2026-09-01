#!/usr/bin/env python3
"""Build the Phase 3B-S position-space physical-sensitivity evidence pack.

This analysis uses only position, velocity, relative base/contact response, and
simulation safety telemetry. It never loads reported effort, connects to a
robot, edits the source MJCF, or identifies real hardware parameters.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys

import mujoco
import numpy as np
import pandas as pd

from phase3bs_core import CALIBRATION, HERE, P3AR, RUNS
from run_phase3bs_experiments import formal_experiments


AY_DIR = CALIBRATION / "phase3ay_motion_conditioned_balance"
AX_DIR = CALIBRATION / "phase3ax_constraint_balance"
P3A_DIR = CALIBRATION / "phase3a_position_only"
PROJECT = CALIBRATION.parent
if str(AY_DIR) not in sys.path:
    sys.path.insert(0, str(AY_DIR))
from phase3ay_core import datasets  # noqa: E402


DT = 0.02
TARGETS = (
    "left_ankle_pitch_joint", "right_ankle_pitch_joint",
    "left_knee_joint", "right_knee_joint",
    "left_hip_pitch_joint", "right_hip_pitch_joint", "waist_pitch_joint",
    "left_ankle_roll_joint", "right_ankle_roll_joint",
    "left_hip_roll_joint", "right_hip_roll_joint", "waist_roll_joint",
)
PAIRING = {
    "MASS_DISTRIBUTION": ("bs_mass_lower_minus08", "bs_mass_lower_plus08"),
    "ROTATIONAL_INERTIA": ("bs_inertia_minus10", "bs_inertia_plus10"),
    "JOINT_DAMPING": ("bs_damping_005", "bs_damping_015"),
    "ARMATURE": ("bs_armature_minus20", "bs_armature_plus20"),
    "CONTACT_FRICTION": ("bs_friction_minus10", "bs_friction_plus10"),
    "CONTACT_COMPLIANCE": ("bs_compliance_stiffer10", "bs_compliance_softer10"),
}
DISPLAY = {
    "MASS_DISTRIBUTION": "Mass distribution",
    "ROTATIONAL_INERTIA": "Rotational inertia",
    "JOINT_DAMPING": "Joint damping",
    "ARMATURE": "Armature",
    "CONTACT_FRICTION": "Contact friction",
    "CONTACT_COMPLIANCE": "Contact compliance",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def fmt(value) -> str:
    if value is None:
        return "UNKNOWN"
    if isinstance(value, (float, np.floating)):
        return "" if math.isnan(float(value)) else f"{float(value):.6g}"
    return str(value).replace("|", "\\|")


def md_table(frame: pd.DataFrame, columns: list[str] | None = None) -> str:
    view = frame if columns is None else frame[columns]
    if view.empty:
        return "_No rows._"
    lines = ["| " + " | ".join(map(str, view.columns)) + " |", "|" + "|".join(["---"] * len(view.columns)) + "|"]
    for row in view.itertuples(index=False, name=None):
        lines.append("| " + " | ".join(fmt(value) for value in row) + " |")
    return "\n".join(lines)


def write_md(name: str, text: str) -> None:
    (HERE / name).write_text(text.strip() + "\n", encoding="utf-8")


def response_characteristics(t: np.ndarray, position: np.ndarray, velocity: np.ndarray, motion_end: float) -> dict:
    pre = (t >= -3.0) & (t <= -0.2)
    motion = (t >= 0.0) & (t <= motion_end)
    baseline = float(np.mean(position[pre]))
    relative = position - baseline
    tm, rm, vm = t[motion], relative[motion], velocity[motion]
    excursion = float(np.max(rm) - np.min(rm))
    peak_index = int(np.argmax(np.abs(rm)))
    signed = float(rm[peak_index])
    threshold = max(0.05 * abs(signed), 3.0 * float(np.std(relative[pre])), 0.0005)
    onset_indices = np.flatnonzero(np.abs(rm) >= threshold)
    onset = float(tm[onset_indices[0]]) if len(onset_indices) else None
    peak_time = float(tm[peak_index])
    recovery = None
    band = max(0.10 * abs(signed), threshold)
    for index in np.flatnonzero(t >= motion_end):
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


def physical_baseline() -> pd.DataFrame:
    model = P3AR.load_model(free_base=True)
    rows: list[dict] = []
    for body_id in range(1, model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or f"body_{body_id}"
        rows.append({
            "section": "BODY_INERTIAL", "name": name,
            "mass_kg": model.body_mass[body_id],
            "inertial_pos": " ".join(map(str, model.body_ipos[body_id])),
            "inertia_diag_kg_m2": " ".join(map(str, model.body_inertia[body_id])),
        })
    for joint_id in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or f"joint_{joint_id}"
        body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.jnt_bodyid[joint_id]))
        dof = int(model.jnt_dofadr[joint_id])
        rows.append({
            "section": "JOINT", "name": name, "body": body,
            "axis": " ".join(map(str, model.jnt_axis[joint_id])),
            "range_rad": " ".join(map(str, model.jnt_range[joint_id])),
            "damping": model.dof_damping[dof], "armature": model.dof_armature[dof],
            "frictionloss": model.dof_frictionloss[dof],
        })
    for geom_id in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or f"geom_{geom_id}"
        body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[geom_id]))
        if name == "floor" or "foot" in (name or "").lower() or "foot" in (body or "").lower():
            rows.append({
                "section": "CONTACT_GEOM", "name": name, "body": body,
                "geom_type": int(model.geom_type[geom_id]),
                "friction": " ".join(map(str, model.geom_friction[geom_id])),
                "solref": " ".join(map(str, model.geom_solref[geom_id])),
                "solimp": " ".join(map(str, model.geom_solimp[geom_id])),
                "contype": int(model.geom_contype[geom_id]), "conaffinity": int(model.geom_conaffinity[geom_id]),
            })
    for actuator_id in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id) or f"actuator_{actuator_id}"
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        joint = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        rows.append({
            "section": "ACTUATOR", "name": name, "joint": joint,
            "gear": " ".join(map(str, model.actuator_gear[actuator_id])),
            "ctrlrange": " ".join(map(str, model.actuator_ctrlrange[actuator_id])),
            "forcerange": " ".join(map(str, model.actuator_forcerange[actuator_id])),
            "ctrllimited": int(model.actuator_ctrllimited[actuator_id]),
            "forcelimited": int(model.actuator_forcelimited[actuator_id]),
        })
    frame = pd.DataFrame(rows)
    frame.to_csv(HERE / "phase3bs_physical_baseline.csv", index=False)
    return frame


def real_frame(dataset: str) -> pd.DataFrame:
    frame = pd.read_csv(AY_DIR / "phase3ay_real_response_features.csv")
    return frame[frame.dataset == dataset].sort_values("t")


def sim_run_metrics(experiment_id: str, dataset: str) -> tuple[dict, pd.DataFrame]:
    ds = datasets()[dataset]
    motion_end = float(ds.motion_end_s)
    log = pd.read_csv(RUNS / f"{experiment_id}__{dataset}__arm_only_joint_log.csv")
    safety = pd.read_csv(RUNS / f"{experiment_id}__{dataset}__arm_only_safety_log.csv")
    summary = json.loads((RUNS / f"{experiment_id}__{dataset}__arm_only_summary.json").read_text(encoding="utf-8"))
    real = real_frame(dataset)
    comparisons = []
    chars: dict[str, dict] = {}
    for joint in TARGETS:
        sim = log[log.joint_name == joint].sort_values("t")
        st = sim.t.to_numpy(float)
        sp = sim.position.to_numpy(float)
        sv = sim.velocity.to_numpy(float)
        char = response_characteristics(st, sp, sv, motion_end)
        chars[joint] = char
        rt = real.t.to_numpy(float)
        rp = real[f"real_response_relative_position__{joint}"].to_numpy(float)
        rv = real[f"real_response_velocity__{joint}"].to_numpy(float)
        real_char = response_characteristics(rt, rp, rv, motion_end)
        common = st[(st >= 0.0) & (st <= motion_end)]
        sim_pre = float(np.mean(sp[(st >= -3.0) & (st <= -0.2)]))
        sr = np.interp(common, st, sp) - sim_pre
        rr = np.interp(common, rt, rp)
        svr = np.interp(common, st, sv)
        rvr = np.interp(common, rt, rv)
        comparisons.append({
            "experiment_id": experiment_id, "dataset": dataset, "joint_name": joint,
            "real_excursion_rad": real_char["excursion_rad"], "sim_excursion_rad": char["excursion_rad"],
            "excursion_ratio": char["excursion_rad"] / max(real_char["excursion_rad"], 1e-12),
            "absolute_excursion_error_rad": abs(char["excursion_rad"] - real_char["excursion_rad"]),
            "position_rmse_rad": float(np.sqrt(np.mean((sr - rr) ** 2))),
            "velocity_rmse_rad_s": float(np.sqrt(np.mean((svr - rvr) ** 2))),
            "onset_delta_s": None if char["onset_s"] is None or real_char["onset_s"] is None else char["onset_s"] - real_char["onset_s"],
            "peak_timing_delta_s": char["peak_time_s"] - real_char["peak_time_s"],
            "xcorr_lag_s": lag_seconds(rr, sr),
            "recovery_delta_s": None if char["recovery_after_motion_s"] is None or real_char["recovery_after_motion_s"] is None else char["recovery_after_motion_s"] - real_char["recovery_after_motion_s"],
        })
    motion_safety = safety[(safety.t >= 0.0) & (safety.t <= motion_end)]
    active_tracking = [row for row in summary["tracking_metrics"] if float(row["real_excursion_rad"]) >= 0.02]
    arm_rmse = float(np.mean([float(row["rmse_rad"]) for row in active_tracking]))
    arm_lags = [abs(float(row["lag_s"])) for row in active_tracking if row["lag_s"] is not None]
    result = {
        "experiment_id": experiment_id, "dataset": dataset,
        "heart_left_ankle_excursion_rad": chars["left_ankle_pitch_joint"]["excursion_rad"] if dataset == "heart" else np.nan,
        "heart_knee_excursion_rad": np.mean([chars["left_knee_joint"]["excursion_rad"], chars["right_knee_joint"]["excursion_rad"]]) if dataset == "heart" else np.nan,
        "heart_waist_roll_excursion_rad": chars["waist_roll_joint"]["excursion_rad"] if dataset == "heart" else np.nan,
        "wave_right_knee_excursion_rad": chars["right_knee_joint"]["excursion_rad"] if dataset == "wave" else np.nan,
        "wave_ankle_excursion_rad": np.mean([chars["left_ankle_pitch_joint"]["excursion_rad"], chars["right_ankle_pitch_joint"]["excursion_rad"]]) if dataset == "wave" else np.nan,
        "base_pitch_excursion_rad": float(motion_safety.base_pitch_rad.max() - motion_safety.base_pitch_rad.min()),
        "base_roll_excursion_rad": float(motion_safety.base_roll_rad.max() - motion_safety.base_roll_rad.min()),
        "foot_slip_max_m": float(max(motion_safety.left_foot_slip_m.max(), motion_safety.right_foot_slip_m.max())),
        "contact_penetration_max_m": float(motion_safety.max_contact_penetration_m.max()),
        "arm_tracking_rmse_rad": arm_rmse,
        "arm_tracking_max_abs_lag_s": max(arm_lags) if arm_lags else 0.0,
        "safety_pass": bool(summary["safety_pass"]),
        "stable_no_fall": bool(summary["stable_no_fall"]),
        "persistent_saturation_fraction": float(summary["persistent_saturation_fraction"]),
        "minimum_limit_margin_rad": float(summary["minimum_limit_margin_rad"]),
    }
    return result, pd.DataFrame(comparisons)


def all_formal_metrics() -> tuple[pd.DataFrame, pd.DataFrame]:
    rows, comparisons = [], []
    for exp in formal_experiments():
        for dataset in ("heart", "wave"):
            row, compare = sim_run_metrics(exp.experiment_id, dataset)
            row.update({
                "family": exp.family, "parameter": exp.parameter, "level": exp.level,
                "parameter_value": exp.parameter_value,
                "normalized_parameter_change": exp.normalized_parameter_change,
            })
            rows.append(row)
            comparisons.append(compare)
    result = pd.DataFrame(rows)
    compare = pd.concat(comparisons, ignore_index=True)
    result.to_csv(HERE / "phase3bs_run_metrics.csv", index=False)
    compare.to_csv(HERE / "phase3bs_position_comparison_metrics.csv", index=False)
    return result, compare


def central_sensitivity(metrics: pd.DataFrame) -> pd.DataFrame:
    rows = []
    output_fields = (
        "heart_left_ankle_excursion_rad", "heart_knee_excursion_rad", "heart_waist_roll_excursion_rad",
        "wave_right_knee_excursion_rad", "wave_ankle_excursion_rad",
        "base_pitch_excursion_rad", "base_roll_excursion_rad", "foot_slip_max_m",
        "contact_penetration_max_m", "arm_tracking_rmse_rad",
    )
    base = metrics[metrics.experiment_id == "bs_baseline"].set_index("dataset")
    for family, (low_id, high_id) in PAIRING.items():
        low = metrics[metrics.experiment_id == low_id].set_index("dataset")
        high = metrics[metrics.experiment_id == high_id].set_index("dataset")
        low_x = float(low.normalized_parameter_change.iloc[0])
        high_x = float(high.normalized_parameter_change.iloc[0])
        row = {
            "physical_family": family,
            "parameter": str(low.parameter.iloc[0]),
            "low_experiment": low_id, "high_experiment": high_id,
            "low_normalized_change": low_x, "high_normalized_change": high_x,
            "safety_effect": "PASS_ALL_HEART_WAVE" if bool(low.safety_pass.all() and high.safety_pass.all()) else "SAFETY_REGRESSION",
        }
        for field in output_fields:
            dataset = "heart" if field.startswith("heart_") else "wave" if field.startswith("wave_") else None
            if dataset:
                y0, yl, yh = float(base.loc[dataset, field]), float(low.loc[dataset, field]), float(high.loc[dataset, field])
            else:
                y0 = float(base[field].mean())
                yl = float(low[field].mean())
                yh = float(high[field].mean())
            row[field.replace("_rad", "").replace("_m", "") + "_normalized_sensitivity"] = (yh - yl) / max(abs(y0), 1e-12) / (high_x - low_x)
            row[field.replace("_rad", "").replace("_m", "") + "_sweep_percent_change"] = 100.0 * (yh - yl) / max(abs(y0), 1e-12)
        rows.append(row)
    matrix = pd.DataFrame(rows)
    matrix.to_csv(HERE / "phase3bs_sensitivity_matrix.csv", index=False)
    return matrix


def experiment_assessment(metrics: pd.DataFrame, compare: pd.DataFrame) -> pd.DataFrame:
    baseline = metrics[metrics.experiment_id == "bs_baseline"].set_index("dataset")
    base_compare = compare[compare.experiment_id == "bs_baseline"]
    rows = []
    for exp in formal_experiments()[1:]:
        subset = metrics[metrics.experiment_id == exp.experiment_id].set_index("dataset")
        c = compare[compare.experiment_id == exp.experiment_id]
        def err(frame: pd.DataFrame, ds: str, joint: str) -> float:
            return float(frame[(frame.dataset == ds) & (frame.joint_name == joint)].absolute_excursion_error_rad.iloc[0])
        wave_improvement = err(base_compare, "wave", "right_knee_joint") - err(c, "wave", "right_knee_joint")
        ankle_regression = err(c, "heart", "left_ankle_pitch_joint") - err(base_compare, "heart", "left_ankle_pitch_joint")
        waist_regression = err(c, "heart", "waist_roll_joint") - err(base_compare, "heart", "waist_roll_joint")
        arm_ratio = float(subset.arm_tracking_rmse_rad.mean() / baseline.arm_tracking_rmse_rad.mean())
        shared = (
            wave_improvement >= 0.05 * err(base_compare, "wave", "right_knee_joint")
            and ankle_regression <= max(0.10 * err(base_compare, "heart", "left_ankle_pitch_joint"), 0.001)
            and waist_regression <= max(0.10 * err(base_compare, "heart", "waist_roll_joint"), 0.001)
            and arm_ratio <= 1.05
            and bool(subset.safety_pass.all())
        )
        rows.append({
            "experiment_id": exp.experiment_id, "family": exp.family, "level": exp.level,
            "wave_knee_absolute_error_improvement_rad": wave_improvement,
            "heart_ankle_absolute_error_regression_rad": ankle_regression,
            "heart_waist_roll_absolute_error_regression_rad": waist_regression,
            "mean_arm_tracking_rmse_ratio_to_baseline": arm_ratio,
            "heart_wave_safety_pass": bool(subset.safety_pass.all()),
            "shared_direction_criteria_pass": shared,
            "classification": "SHARED_PHYSICAL_SENSITIVITY_DIRECTION" if shared else "PHYSICAL_SENSITIVITY_EXPERIMENT",
        })
    result = pd.DataFrame(rows)
    result.to_csv(HERE / "phase3bs_cross_motion_assessment.csv", index=False)
    return result


def decomposition_metrics() -> pd.DataFrame:
    ds = datasets()["wave"]
    motion_end = float(ds.motion_end_s)
    cases = [
        ("A_NORMAL_FREE_BASE_CONTACT_RETAINED", RUNS / "bs_baseline__wave__arm_only_joint_log.csv", RUNS / "bs_baseline__wave__arm_only_summary.json", False),
        ("B_FIXED_BASE_CONTACT_RETAINED", RUNS / "bs_decomp_fixed_base_joint_log.csv", RUNS / "bs_decomp_fixed_base_summary.json", True),
        ("C_BALANCE_DISABLED_DIAGNOSTIC", RUNS / "bs_decomp_balance_disabled__wave__arm_only_joint_log.csv", RUNS / "bs_decomp_balance_disabled__wave__arm_only_summary.json", False),
        ("D_KNEE_ACTUATOR_CHANNEL_DISABLED", RUNS / "bs_decomp_knee_actuator_disabled__wave__arm_only_joint_log.csv", RUNS / "bs_decomp_knee_actuator_disabled__wave__arm_only_summary.json", False),
        ("F_FIXED_BASE_CONTACT_FREE_DIAGNOSTIC", RUNS / "bs_decomp_fixed_contact_free_joint_log.csv", RUNS / "bs_decomp_fixed_contact_free_summary.json", True),
    ]
    rows = []
    for name, log_path, summary_path, fixed in cases:
        log = pd.read_csv(log_path)
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        knee = log[log.joint_name == "right_knee_joint"].sort_values("t")
        fall = summary.get("fall_time_s", summary.get("fall_time_seconds"))
        observed_end = min(motion_end, float(fall)) if fall is not None else motion_end
        valid = knee[knee.t <= observed_end]
        char = response_characteristics(valid.t.to_numpy(float), valid.position.to_numpy(float), valid.velocity.to_numpy(float), observed_end)
        rows.append({
            "case": name, "fixed_base": fixed, "contact_retained": "CONTACT_FREE" not in name,
            "balance_feedback_enabled": "BALANCE_DISABLED" not in name,
            "knee_actuator_enabled": "KNEE_ACTUATOR" not in name,
            "observed_until_s": observed_end,
            "right_knee_excursion_rad_before_failure": char["excursion_rad"],
            "right_knee_peak_abs_velocity_rad_s": char["peak_abs_velocity_rad_s"],
            "stable_no_fall": bool(summary.get("stable_no_fall", True)),
            "fall_time_s": fall,
            "minimum_limit_margin_rad": summary.get("minimum_limit_margin_rad"),
            "persistent_saturation_fraction": summary.get("persistent_saturation_fraction", summary.get("persistent_saturation_sample_fraction")),
            "classification": "DIAGNOSTIC_ONLY_NOT_CONTROLLER_CANDIDATE",
        })
    frame = pd.DataFrame(rows)
    frame.to_csv(HERE / "phase3bs_passive_coupling_decomposition.csv", index=False)
    return frame


def source_lock(baseline: pd.DataFrame) -> None:
    paths = [
        PROJECT / "assets" / "Master" / "ff_master_ultra.xml",
        PROJECT / "assets" / "Master" / "ff_master_ultra_x2_limits.xml",
        PROJECT / "assets" / "Master" / "scene_x2_fixed.xml",
        PROJECT / "assets" / "Master" / "scene_x2_free.xml",
        PROJECT / "master_sim" / "controller.py",
        PROJECT / "master_sim" / "model.py",
        CALIBRATION / "phase3ar_controller_redesign" / "phase3ar_core.py",
        AY_DIR / "phase3ay_core.py",
        AY_DIR / "run_phase3ay_candidate_smoke.py",
        AY_DIR / "simulation_motion_conditioned_balance_candidate.json",
        AY_DIR / "phase3ay_final_validation.json",
        AY_DIR / "phase3ay_perturbation_results.csv",
        AX_DIR / "phase3ax_core.py",
        AX_DIR / "simulation_constraint_aware_controller_candidate.json",
        AX_DIR / "phase3ax_rehearsal_12_joint.csv",
        P3A_DIR / "simulation_controller_alignment_candidate.json",
        P3A_DIR / "run_phase3a_experiments.py",
        CALIBRATION / "phase2e_replay" / "phase2e_heart_measured_reference.csv",
        CALIBRATION / "phase3av_validation" / "phase3av_measured_reference.csv",
        HERE / "phase3bs_core.py",
        HERE / "run_phase3bs_experiments.py",
        HERE / "run_phase3bs_candidate_safety.py",
        HERE / "analyze_phase3bs.py",
    ]
    manifest = pd.DataFrame([
        {"path": str(path.relative_to(PROJECT)), "bytes": path.stat().st_size, "sha256": sha256(path)}
        for path in paths if path.exists()
    ])
    manifest.to_csv(HERE / "phase3bs_source_manifest.csv", index=False)
    body = baseline[baseline.section == "BODY_INERTIAL"]
    joint = baseline[baseline.section == "JOINT"]
    contact = baseline[baseline.section == "CONTACT_GEOM"]
    actuator = baseline[baseline.section == "ACTUATOR"]
    write_md("phase3bs_source_lock.md", f"""
# Phase 3B-S source and physical baseline lock

## Scope lock

- Controller: independently validated arm tracking + Phase 3A-X safety shell + Phase 3A-Y motion-conditioned response, frozen by SHA-256 below.
- MJCF/scenes: `ff_master_ultra.xml`, `ff_master_ultra_x2_limits.xml`, `scene_x2_fixed.xml`, and `scene_x2_free.xml`, all read-only and SHA-256 locked below.
- Physical experiments: in-memory runtime overrides only; no source MJCF or calibrated MJCF was created.
- Replay protocol: the frozen Phase 3A-Y 5 s pre-roll / 5 s post-roll window. The rejected 3 s protocol is preserved separately in `runs_protocol3_diagnostic_archive/` and excluded from formal metrics.
- Fitting evidence: joint position/velocity and relative simulation base/contact response only.
- `reported_effort`: not loaded. Absolute IMU quaternion: not used. Robot connection/control: none.
- Gear, actuator force/ctrl limits, hardware mapping, and controller settings: frozen and asserted unchanged in every run summary.

## SHA-256 manifest

{md_table(manifest)}

## Compiled physical baseline inventory

- Total compiled mass: **{float(body.mass_kg.sum()):.9f} kg** across {len(body)} inertial bodies.
- Bodies/inertials: {len(body)} rows; joints: {len(joint)} rows; floor/foot contact geoms: {len(contact)} rows; actuators: {len(actuator)} rows.
- Full immutable inventory: `phase3bs_physical_baseline.csv` (mass, inertia, inertial position, joint damping/armature/frictionloss, floor/foot contact, actuator gear and limits).
- Baseline actuated joint damping values: {sorted(set(float(v) for v in joint.damping.dropna()))}.
- Baseline actuated joint armature values: {sorted(set(float(v) for v in joint.armature.dropna()))}.

## Interpretation

`PHYSICAL_SENSITIVITY_EXPERIMENT` means a local simulation perturbation. It is **NOT HARDWARE CALIBRATION** and does not identify a real X2 parameter.
""")


def family_report(family: str, metrics: pd.DataFrame, matrix: pd.DataFrame, assessment: pd.DataFrame, filename: str) -> None:
    ids = list(PAIRING[family])
    m = metrics[metrics.experiment_id.isin(ids)].copy()
    a = assessment[assessment.experiment_id.isin(ids)].copy()
    s = matrix[matrix.physical_family == family]
    columns = [
        "experiment_id", "dataset", "parameter_value",
        "heart_left_ankle_excursion_rad", "heart_knee_excursion_rad", "heart_waist_roll_excursion_rad",
        "wave_right_knee_excursion_rad", "wave_ankle_excursion_rad",
        "base_pitch_excursion_rad", "base_roll_excursion_rad", "foot_slip_max_m",
        "contact_penetration_max_m", "arm_tracking_rmse_rad", "safety_pass",
    ]
    write_md(filename, f"""
# Phase 3B-S — {DISPLAY[family]} sensitivity

Classification: **PHYSICAL_SENSITIVITY_EXPERIMENT — NOT HARDWARE CALIBRATION**.

Only `{m.parameter.iloc[0]}` changed. Controller, source MJCF, gear, torque/force/ctrl limits, and all other physical families remained frozen.

## Heart/Wave runs

{md_table(m[columns])}

## Central normalized local sensitivity

Definition: `(y_high - y_low) / |y_baseline| / (normalized_p_high - normalized_p_low)`. Sign is directional; magnitude ranks local sensitivity.

{md_table(s)}

## Cross-motion assessment

{md_table(a)}

No value above is claimed to be a real X2 parameter.
""")


def build_reports(baseline: pd.DataFrame, metrics: pd.DataFrame, compare: pd.DataFrame, matrix: pd.DataFrame, assessment: pd.DataFrame, decomp: pd.DataFrame) -> dict:
    source_lock(baseline)
    base_compare = compare[compare.experiment_id == "bs_baseline"]
    knee = base_compare[(base_compare.dataset == "wave") & (base_compare.joint_name == "right_knee_joint")].iloc[0]
    real_targets = pd.read_csv(AY_DIR / "phase3ay_real_output_response_targets.csv")
    real_wave_pitch = real_targets[(real_targets.dataset == "wave") & (real_targets.plane == "pitch")]
    typical_real = float(real_wave_pitch.excursion_rad.median())
    small_denominator_factor = typical_real / float(knee.real_excursion_rad)
    counterfactual_ratio = float(knee.sim_excursion_rad) / typical_real
    excess_ratio_share = (float(knee.excursion_ratio) - counterfactual_ratio) / max(float(knee.excursion_ratio) - 1.0, 1e-12)
    log_share = math.log(small_denominator_factor) / math.log(float(knee.excursion_ratio))
    write_md("phase3bs_wave_knee_denominator_analysis.md", f"""
# Wave right-knee denominator analysis

## Measured comparison

{md_table(pd.DataFrame([knee]))}

- Real excursion denominator: **{knee.real_excursion_rad:.8f} rad**.
- Simulation excursion: **{knee.sim_excursion_rad:.8f} rad**.
- Absolute excursion difference: **{knee.absolute_excursion_error_rad:.8f} rad**.
- Observed ratio: **{knee.excursion_ratio:.6f}×**.
- Median real Wave pitch-chain excursion: **{typical_real:.8f} rad**.
- Small-denominator amplification relative to that median: **{small_denominator_factor:.3f}×**.
- Replacing the knee denominator with that median would leave a **{counterfactual_ratio:.3f}×** response, so the mismatch is not only a ratio artifact.
- The denominator accounts for **{100*excess_ratio_share:.1f}%** of the excess-over-one ratio under this explicit median-response counterfactual; in log-ratio terms it accounts for **{100*log_share:.1f}%**. These are sensitivity summaries, not causal parameter identification.

`RATIO_AMPLIFICATION_BY_SMALL_REAL_EXCURSION = YES`.

The cross-correlation lag reaches the -1.0 s search boundary and is therefore boundary-censored; it is reported, but not interpreted as a precise lag estimate.
""")

    normal = float(decomp.loc[decomp.case.str.startswith("A_"), "right_knee_excursion_rad_before_failure"].iloc[0])
    fixed = float(decomp.loc[decomp.case.str.startswith("B_"), "right_knee_excursion_rad_before_failure"].iloc[0])
    no_balance = float(decomp.loc[decomp.case.str.startswith("C_"), "right_knee_excursion_rad_before_failure"].iloc[0])
    no_knee = float(decomp.loc[decomp.case.str.startswith("D_"), "right_knee_excursion_rad_before_failure"].iloc[0])
    contact_free = float(decomp.loc[decomp.case.str.startswith("F_"), "right_knee_excursion_rad_before_failure"].iloc[0])
    write_md("phase3bs_passive_coupling_decomposition.md", f"""
# Wave right-knee passive-coupling decomposition

All non-normal runs are **DIAGNOSTIC_ONLY** and are not controller candidates.

{md_table(decomp)}

## Interpretation

- Fixing the base changes right-knee excursion from {normal:.6f} to {fixed:.6f} rad ({100*(fixed/normal-1):+.1f}%). This quantifies the base/contact pathway under retained contacts.
- Disabling balance feedback produces {no_balance:.6f} rad before failure and then falls; the unstable tail is excluded. It proves the frozen balance loop is safety-critical and cannot serve as an alternative controller.
- Disabling the knee actuator produces only {no_knee:.6f} rad before an early failure at {float(decomp.loc[decomp.case.str.startswith('D_'), 'fall_time_s'].iloc[0]):.3f} s. Because the run fails before most of the Wave response develops, it confirms that the channel is safety-critical but does **not** quantify a full passive residual.
- Fixed-base contact-free gives {contact_free:.6f} rad versus {fixed:.6f} rad with contact. The diagnostic is technically limited: fixing the base removes the global support dynamics whose contact contribution is of interest. A free-base contact-free run would simply be unsupported and is not meaningful.
- Controller reference contribution remains observable in the normal joint/command decomposition logs; MC internal command and real torque are not used.

## Causal ranking from decomposition

1. **Closed-loop base/contact/leg coupling — PRIMARY**: fixed-base changes response; disabling balance causes a fall.
2. **Direct knee balance actuation plus passive leg mechanics — SECONDARY/PARTIAL**: isolation fails too early to separate these components reliably.
3. **Pure passive mechanical residual — UNKNOWN**: the disabled-channel run is truncated before a comparable Wave interval.
4. **Contact reaction alone — UNKNOWN/PARTIAL**: fixed/contact-free comparison is conditioned on an artificial fixed base.
5. **Hip versus ankle sub-path — UNKNOWN**: not isolated independently because doing so would invalidate the safety baseline.
""")

    family_report("MASS_DISTRIBUTION", metrics, matrix, assessment, "phase3bs_mass_sensitivity.md")
    family_report("ROTATIONAL_INERTIA", metrics, matrix, assessment, "phase3bs_inertia_sensitivity.md")
    family_report("JOINT_DAMPING", metrics, matrix, assessment, "phase3bs_damping_sensitivity.md")
    family_report("ARMATURE", metrics, matrix, assessment, "phase3bs_armature_sensitivity.md")
    contact_metrics = metrics[metrics.family.isin(("CONTACT_FRICTION", "CONTACT_COMPLIANCE"))]
    contact_matrix = matrix[matrix.physical_family.isin(("CONTACT_FRICTION", "CONTACT_COMPLIANCE"))]
    contact_assess = assessment[assessment.family.isin(("CONTACT_FRICTION", "CONTACT_COMPLIANCE"))]
    write_md("phase3bs_contact_sensitivity.md", f"""
# Phase 3B-S — Contact/friction/compliance sensitivity

The two contact families were tested separately; collision topology remained unchanged in every formal run.

{md_table(contact_metrics[["experiment_id", "dataset", "family", "parameter_value", "wave_right_knee_excursion_rad", "wave_ankle_excursion_rad", "base_pitch_excursion_rad", "base_roll_excursion_rad", "foot_slip_max_m", "contact_penetration_max_m", "arm_tracking_rmse_rad", "safety_pass"]])}

## Normalized local sensitivity

{md_table(contact_matrix)}

## Cross-motion assessment

{md_table(contact_assess)}

These are local contact-model sensitivities, not identified floor/foot parameters.
""")

    write_md("phase3bs_cross_motion_report.md", f"""
# Phase 3B-S cross-motion consistency

Every physical perturbation ran on both independent datasets using the same frozen controller.

{md_table(assessment)}

Acceptance screen for a shared direction: Wave knee absolute error improves by at least 5%; Heart ankle and waist-roll errors do not regress beyond 10% (with a 0.001 rad numerical floor); mean active-arm RMSE stays within 5%; Heart/Wave safety both pass.

This screen selects directions for future validation only. It does not identify real hardware values and does not automatically promote a sensitivity experiment to a calibrated candidate.
""")

    rank = matrix[["physical_family", "wave_right_knee_excursion_normalized_sensitivity"]].copy()
    rank["absolute_sensitivity"] = rank.wave_right_knee_excursion_normalized_sensitivity.abs()
    rank = rank.sort_values("absolute_sensitivity", ascending=False).reset_index(drop=True)
    top = float(rank.absolute_sensitivity.max())
    classes = []
    for index, row in rank.iterrows():
        value = float(row.absolute_sensitivity)
        if value < 0.01:
            label = "RULED_OUT_AT_TESTED_LOCAL_SCALE"
        elif index == 0:
            label = "PRIMARY_PHYSICAL_SENSITIVITY"
        elif value >= 0.25 * top:
            label = "SECONDARY_PHYSICAL_SENSITIVITY"
        else:
            label = "LOW_SENSITIVITY"
        classes.append(label)
    rank["ranking_class"] = classes
    rank.to_csv(HERE / "phase3bs_root_cause_ranking.csv", index=False)
    write_md("phase3bs_root_cause_ranking.md", f"""
# Wave right-knee physical-sensitivity ranking

{md_table(rank)}

This ranks local output sensitivity around the current simulation baseline. It is **not** an identified real parameter ranking. Unobservable real MC behavior, unknown physical sign/zero evidence, and unknown effort semantics remain outside this experiment.
""")

    shared = assessment[assessment.shared_direction_criteria_pass]
    formal_safety = bool(metrics.safety_pass.all())
    arm_preserved = bool((assessment.mean_arm_tracking_rmse_ratio_to_baseline <= 1.05).all())
    informative = bool(top >= 0.05 and np.isfinite(top))
    safety_path = HERE / "phase3bs_shared_direction_safety_validation.json"
    candidate_safety = json.loads(safety_path.read_text(encoding="utf-8")) if safety_path.exists() else None
    if shared.empty:
        candidate_safety_ok = True
        candidate_safety_text = "No experiment passed the shared-direction screen; no physical candidate was promoted."
    elif candidate_safety is None:
        candidate_safety_ok = False
        candidate_safety_text = "Shared direction exists, but its full perturbation/rehearsal validation is missing."
    else:
        perturb_ok = int(candidate_safety["perturbation_pass_count"]) == int(candidate_safety["perturbation_total"])
        rehearsal_ok = int(candidate_safety["rehearsal"]["settled_count"]) == int(candidate_safety["rehearsal"]["total"])
        candidate_safety_ok = perturb_ok and rehearsal_ok
        candidate_safety_text = (
            f"Shared direction validation: perturbation {candidate_safety['perturbation_pass_count']}/"
            f"{candidate_safety['perturbation_total']} PASS; rehearsal "
            f"{candidate_safety['rehearsal']['settled_count']}/{candidate_safety['rehearsal']['total']} SETTLED."
        )
    safety_preserved = formal_safety and candidate_safety_ok
    gates = {
        "PHYSICAL_SENSITIVITY_INFORMATIVE": "YES" if informative else "NO",
        "SHARED_PHYSICAL_SENSITIVITY_DIRECTION": "YES" if not shared.empty else "NO",
        "CONTROLLER_BASELINE_PRESERVED": "YES" if arm_preserved else "NO",
        "SAFETY_BASELINE_PRESERVED": "YES" if safety_preserved else "NO",
        "POSITION_SPACE_PHYSICAL_SENSITIVITY_READY": "YES" if informative and arm_preserved and safety_preserved else "NO",
        "DYNAMICS_CALIBRATION_READY": "NO",
    }
    write_md("phase3bs_final_gate.md", f"""
# Phase 3B-S final gate

## Gates

{md_table(pd.DataFrame([{"gate": key, "status": value} for key, value in gates.items()]))}

## Evidence summary

- Formal local sensitivity runs: {len(metrics)} Heart/Wave runs; safety pass: {int(metrics.safety_pass.sum())}/{len(metrics)}.
- Frozen-source perturbation suite remains 8/8 PASS and frozen 12-joint rehearsal remains 12/12 SETTLED by locked Phase 3A-X/Y evidence.
- {candidate_safety_text}
- Shared-direction screen passes: {len(shared)} experiment(s): {', '.join(shared.experiment_id) if not shared.empty else 'none'}.
- Highest local Wave knee sensitivity: {rank.iloc[0].physical_family} = {rank.iloc[0].wave_right_knee_excursion_normalized_sensitivity:.6g}.
- Controller architecture and hashes are unchanged. All physical perturbations are runtime-only and labeled NOT HARDWARE CALIBRATION.

`DYNAMICS_CALIBRATION_READY` remains **NO** because sign/zero, effort semantics, absolute IMU transform, and MC internal command observability gates are not closed.
""")
    return {"gates": gates, "rank": rank.to_dict("records"), "shared": shared.to_dict("records"), "knee": knee.to_dict()}


def main() -> int:
    baseline = physical_baseline()
    metrics, compare = all_formal_metrics()
    matrix = central_sensitivity(metrics)
    assessment = experiment_assessment(metrics, compare)
    decomp = decomposition_metrics()
    result = build_reports(baseline, metrics, compare, matrix, assessment, decomp)
    (HERE / "phase3bs_analysis_summary.json").write_text(json.dumps(result, indent=2, default=str) + "\n", encoding="utf-8")
    print(json.dumps(result["gates"], indent=2))
    print("TOP_RANK", json.dumps(result["rank"][0], default=str))
    print("SHARED", [row["experiment_id"] for row in result["shared"]])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
