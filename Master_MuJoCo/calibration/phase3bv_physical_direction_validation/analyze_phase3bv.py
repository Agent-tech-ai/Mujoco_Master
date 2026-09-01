#!/usr/bin/env python3
"""Generate pending or completed Phase 3B-V blind-validation reports."""

from __future__ import annotations

import hashlib
import json
import math
from datetime import datetime
from pathlib import Path
import sys

import numpy as np
import pandas as pd


HERE = Path(__file__).resolve().parent
CALIBRATION = HERE.parent
PROJECT = CALIBRATION.parent
WORKSPACE = PROJECT.parent
BS_DIR = CALIBRATION / "phase3bs_physical_sensitivity"
if str(BS_DIR) not in sys.path:
    sys.path.insert(0, str(BS_DIR))
from analyze_phase3bs import lag_seconds, md_table, response_characteristics  # noqa: E402


BASE_ID = "phase3bv_original_physical_baseline"
MASS_ID = "phase3bv_bs_mass_lower_plus08"
DATASET = "phase3bv_clap"
BALANCE_JOINTS = (
    "left_ankle_pitch_joint", "right_ankle_pitch_joint",
    "left_ankle_roll_joint", "right_ankle_roll_joint",
    "left_hip_pitch_joint", "right_hip_pitch_joint",
    "left_hip_roll_joint", "right_hip_roll_joint",
    "left_knee_joint", "right_knee_joint", "waist_pitch_joint", "waist_roll_joint",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_md(name: str, text: str) -> None:
    (HERE / name).write_text(text.strip() + "\n", encoding="utf-8")


def source_lock() -> pd.DataFrame:
    capture_dir = HERE / "capture" / "phase3bv_clap_001"
    capture_path = capture_dir / "raw_serialized_evidence.txt"
    paths = [
        PROJECT / "assets" / "Master" / "ff_master_ultra.xml",
        PROJECT / "assets" / "Master" / "ff_master_ultra_x2_limits.xml",
        PROJECT / "assets" / "Master" / "scene_x2_fixed.xml",
        PROJECT / "assets" / "Master" / "scene_x2_free.xml",
        PROJECT / "master_sim" / "controller.py",
        PROJECT / "master_sim" / "model.py",
        CALIBRATION / "phase3a_position_only" / "run_phase3a_experiments.py",
        CALIBRATION / "phase3ar_controller_redesign" / "phase3ar_core.py",
        CALIBRATION / "phase3ax_constraint_balance" / "phase3ax_core.py",
        CALIBRATION / "phase3ay_motion_conditioned_balance" / "phase3ay_core.py",
        CALIBRATION / "phase3ay_motion_conditioned_balance" / "simulation_motion_conditioned_balance_candidate.json",
        BS_DIR / "phase3bs_core.py",
        BS_DIR / "phase3bs_analysis_summary.json",
        BS_DIR / "phase3bs_sensitivity_matrix.csv",
        BS_DIR / "phase3bs_position_comparison_metrics.csv",
        BS_DIR / "phase3bs_shared_direction_safety_validation.json",
        HERE / "process_phase3bv_capture.py",
        HERE / "run_phase3bv_replays.py",
        HERE / "analyze_phase3bv.py",
        HERE / "finalize_phase3bv_blind_validation.py",
        HERE / "README.md",
        HERE / "history" / "phase3bv_offline_preparation_gate_20260827.md",
        capture_path,
        capture_dir / "recorder_status.txt",
        capture_dir / "operator_safety_confirmation.txt",
        HERE / "phase3bv_capture_metadata.json",
        HERE / "phase3bv_independence.json",
        HERE / "phase3bv_joint_metrics.csv",
        HERE / "phase3bv_measured_reference.csv",
        HERE / "phase3bv_aligned_joint_data.csv",
        HERE / "phase3bv_aligned_imu_data.csv",
        HERE / "phase3bv_replay_execution.json",
        WORKSPACE / "work" / "run_x2_phase3bv_clap_capture_readonly.ps1",
        WORKSPACE / "work" / "x2_phase2d_heart_capture_readonly.sh",
        WORKSPACE / "work" / "phase2c_agentech01_code_discovery_readonly.txt",
    ]
    rows = [{
        "path": str(path.relative_to(WORKSPACE)),
        "bytes": path.stat().st_size,
        "sha256": sha256(path),
    } for path in paths if path.exists()]
    manifest = pd.DataFrame(rows)
    manifest.to_csv(HERE / "phase3bv_source_manifest.csv", index=False)
    if capture_path.exists():
        capture_item = capture_path.stat()
        measured_reference = HERE / "phase3bv_measured_reference.csv"
        capture_lock = f"""
## Third-motion capture lock

- `THIRD_MOTION_CAPTURE = CLAP`
- `CAPTURE_SOURCE = READ_ONLY_REAL_ROBOT_STATE`
- `PRESET_EXECUTION = EXTERNAL_OPERATOR / EXISTING_MC_COMPATIBLE_PATH`
- `RECORDER_SENT_COMMAND = NO`
- absolute path: `{capture_path.resolve()}`
- bytes: `{capture_item.st_size}`
- last modified: `{datetime.fromtimestamp(capture_item.st_mtime).astimezone().isoformat()}`
- SHA-256: `{sha256(capture_path)}`

## Blind comparison lock

- original MJCF SHA-256: `{sha256(PROJECT / 'assets' / 'Master' / 'ff_master_ultra.xml')}`
- frozen Phase 3A-Y controller SHA-256: `{sha256(CALIBRATION / 'phase3ay_motion_conditioned_balance' / 'phase3ay_core.py')}`
- frozen +8% candidate/replay implementation SHA-256: `{sha256(HERE / 'run_phase3bv_replays.py')}`
- measured replay input SHA-256: `{sha256(measured_reference) if measured_reference.exists() else 'PENDING_PROCESSING'}`
- baseline/candidate controller config: `IDENTICAL` (enforced before replay)
"""
    else:
        capture_lock = "\n## Third-motion capture lock\n\n`THIRD_MOTION_CAPTURE = PENDING`\n"
    write_md("phase3bv_source_lock.md", f"""
# Phase 3B-V source lock

- Controller architecture: frozen Phase 3A arm tracking + Phase 3A-X safety shell + Phase 3A-Y motion-conditioned response.
- Physical comparison: original baseline versus `bs_mass_lower_plus08` only.
- Candidate label: **SHARED_PHYSICAL_SENSITIVITY_DIRECTION — NOT IDENTIFIED HARDWARE PARAMETER**.
- Source MJCF/scenes are immutable. No calibrated MJCF is created.
- Inertia, damping, friction, armature, gear, limits, controller parameters, hardware mapping, and standing references remain frozen.
- `reported_effort` is excluded. IMU use is relative roll/pitch and gyro only.
- Recorder is subscription-only and cannot invoke a preset. Codex analysis/replay made no robot connection; the real capture was produced by the user-run read-only recorder.

{capture_lock}

## SHA-256 manifest

{md_table(manifest)}

`DYNAMICS_CALIBRATION_READY = NO`
""")
    return manifest


def selected_motion_report(capture_ready: bool, independence: dict | None) -> None:
    independence_text = "PENDING_CAPTURE"
    if independence:
        independence_text = str(independence.get("decision", "UNKNOWN"))
    write_md("phase3bv_selected_validation_motion.md", f"""
# Phase 3B-V selected blind-validation motion

| Item | Selection/evidence |
|---|---|
| Candidate motion | `clap` |
| Native MC preset | motion `3017`, area `11` |
| Catalog qualification | `physically_tested_without_parameters=True` |
| Motion hands | left + right |
| Control path | existing standing gesture wrapper → `SetMcPresetMotion(..., interrupt=False)` → native MC |
| Direct HAL | not used |
| Existing third capture | {'YES' if capture_ready else 'NO'} |
| Numerical independence from Heart/Wave | {independence_text} |

The catalog evidence comes from the already-captured read-only Phase 2C source at `work/phase2c_agentech01_code_discovery_readonly.txt`. `clap` is selected because it is a physically-tested bilateral coordinated preset with a different preset ID and expected temporal/spatial structure from bilateral Heart and unilateral right-hand Wave.

This selection does **not** assume independence. The capture must pass active-joint, excursion-vector, duration, and left/right response checks against both prior motions.

## {'Accepted capture' if capture_ready else 'Prepared recorder (not executed)'}

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "{WORKSPACE / 'work' / 'run_x2_phase3bv_clap_capture_readonly.ps1'}" -Target "run@192.168.4.114" -CaptureSeconds 120 -OperatorSafetyConfirmed
```

{'The capture was made by the user-run read-only recorder; preset execution was external operator action through the existing MC-compatible path. Codex and the recorder did not invoke motion.' if capture_ready else 'Only the onsite operator may invoke the separately approved preset after the recorder prints `PRE-ROLL COMPLETE`. Codex and the recorder do not invoke motion.'}
""")


def historical_rows() -> tuple[pd.DataFrame, pd.DataFrame]:
    compare = pd.read_csv(BS_DIR / "phase3bs_position_comparison_metrics.csv")
    runs = pd.read_csv(BS_DIR / "phase3bs_run_metrics.csv")
    ids = ["bs_baseline", "bs_mass_lower_plus08"]
    return compare[compare.experiment_id.isin(ids)].copy(), runs[runs.experiment_id.isin(ids)].copy()


def pending_reports(reason: str) -> None:
    compare, runs = historical_rows()
    key = compare[
        ((compare.dataset == "wave") & (compare.joint_name == "right_knee_joint"))
        | ((compare.dataset == "heart") & compare.joint_name.isin(("left_ankle_pitch_joint", "waist_roll_joint")))
    ][["experiment_id", "dataset", "joint_name", "real_excursion_rad", "sim_excursion_rad", "excursion_ratio", "absolute_excursion_error_rad", "position_rmse_rad", "velocity_rmse_rad_s"]]
    write_md("phase3bv_baseline_report.md", f"""
# Phase 3B-V original physical baseline

Third-motion replay status: **PENDING — {reason}**.

The locked Heart/Wave baseline is retained for context; it is not reused as blind third-motion evidence.

{md_table(key[key.experiment_id == 'bs_baseline'])}

Wave right-knee baseline remains exactly: real `0.0101536374 rad`, simulation `0.0820920169 rad`, ratio `8.084986×`, absolute difference `0.0719383795 rad`.
""")
    write_md("phase3bv_mass_direction_report.md", f"""
# Phase 3B-V mass sensitivity direction

Candidate: `bs_mass_lower_plus08` — **SHARED_PHYSICAL_SENSITIVITY_DIRECTION, NOT IDENTIFIED HARDWARE PARAMETER**.

Third-motion replay status: **PENDING — {reason}**.

{md_table(key[key.experiment_id == 'bs_mass_lower_plus08'])}

Prior Heart/Wave evidence is supportive but cannot establish three-motion generalization.
""")
    cross = pd.DataFrame([
        {"motion": "Heart", "evidence_role": "prior", "baseline_candidate_compared": True, "blind_third_motion": False, "status": "SUPPORTIVE_PRIOR_EVIDENCE"},
        {"motion": "Wave(right)", "evidence_role": "prior", "baseline_candidate_compared": True, "blind_third_motion": False, "status": "SUPPORTIVE_PRIOR_EVIDENCE"},
        {"motion": "clap", "evidence_role": "blind validation", "baseline_candidate_compared": False, "blind_third_motion": True, "status": "PENDING_CAPTURE"},
    ])
    write_md("phase3bv_cross_motion_report.md", f"""
# Phase 3B-V cross-motion report

{md_table(cross)}

`PHYSICAL_DIRECTION_GENERALIZES` cannot be established from the two motions used to discover the direction. No third-motion result is inferred or fabricated.
""")
    safety = json.loads((BS_DIR / "phase3bs_shared_direction_safety_validation.json").read_text(encoding="utf-8"))
    write_md("phase3bv_safety_report.md", f"""
# Phase 3B-V safety report

- Existing Phase 3B-S formal Heart/Wave runs: 26/26 safety pass.
- Existing shared-direction perturbations: {safety['perturbation_pass_count']}/{safety['perturbation_total']} PASS.
- Existing shared-direction rehearsal: {safety['rehearsal']['settled_count']}/{safety['rehearsal']['total']} SETTLED.
- Third-motion baseline/candidate safety: **PENDING_CAPTURE**.
- Recorder: subscription-only; no publisher, control service/action, mode switch, process control, or motion invocation.

Because new-motion safety is unobserved, the Phase 3B-V safety gate remains NO/PENDING even though all prior safety evidence is preserved.
""")
    write_md("phase3bv_final_gate.md", f"""
# Phase 3B-V final gate

| Gate | Status |
|---|---|
| PHYSICAL_DIRECTION_GENERALIZES | NO — PENDING_THIRD_MOTION_CAPTURE |
| CONTROLLER_BASELINE_PRESERVED | NO — NEW_MOTION_ARM_TRACKING_UNVALIDATED |
| SAFETY_BASELINE_PRESERVED | NO — NEW_MOTION_UNVALIDATED |
| POSITION_SPACE_PHYSICAL_DIRECTION_VALIDATED | NO |
| DYNAMICS_CALIBRATION_READY | NO |

This is a pending-data gate, not evidence that the direction fails. Reason: {reason}.

The baseline and candidate controller configurations are source-identical, but controller-response preservation cannot be promoted to YES until arm tracking is observed on the third motion.

Persistent blockers: `PHYSICAL_SIGN=UNKNOWN`, `PHYSICAL_ZERO=UNKNOWN`, `EFFORT_SEMANTICS=UNKNOWN`, `IMU_TRANSFORM=PARTIAL`, `MC_INTERNAL_COMMAND=UNOBSERVABLE`.
""")


def compare_variant(experiment_id: str, motion_end: float) -> tuple[pd.DataFrame, dict, pd.DataFrame]:
    real = pd.read_csv(HERE / "phase3bv_aligned_joint_data.csv")
    log = pd.read_csv(HERE / "runs" / f"{experiment_id}__{DATASET}__arm_only_joint_log.csv")
    safety = pd.read_csv(HERE / "runs" / f"{experiment_id}__{DATASET}__arm_only_safety_log.csv")
    summary = json.loads((HERE / "runs" / f"{experiment_id}__{DATASET}__arm_only_summary.json").read_text(encoding="utf-8"))
    rows = []
    for joint in sorted(set(real.joint_name) & set(log.joint_name)):
        r = real[real.joint_name == joint].sort_values("t")
        s = log[log.joint_name == joint].sort_values("t")
        rt, rp, rv = r.t.to_numpy(float), r.position.to_numpy(float), r.velocity.to_numpy(float)
        st, sp, sv = s.t.to_numpy(float), s.position.to_numpy(float), s.velocity.to_numpy(float)
        rc = response_characteristics(rt, rp, rv, motion_end)
        sc = response_characteristics(st, sp, sv, motion_end)
        common = st[(st >= 0.0) & (st <= motion_end)]
        rpre = float(np.mean(rp[(rt >= -3.0) & (rt <= -0.2)]))
        spre = float(np.mean(sp[(st >= -3.0) & (st <= -0.2)]))
        rr = np.interp(common, rt, rp) - rpre
        sr = np.interp(common, st, sp) - spre
        rvr = np.interp(common, rt, rv)
        svr = np.interp(common, st, sv)
        rows.append({
            "experiment_id": experiment_id, "joint_name": joint,
            "real_excursion_rad": rc["excursion_rad"], "sim_excursion_rad": sc["excursion_rad"],
            "excursion_ratio": sc["excursion_rad"] / max(rc["excursion_rad"], 1e-12),
            "absolute_excursion_error_rad": abs(sc["excursion_rad"] - rc["excursion_rad"]),
            "position_rmse_rad": float(np.sqrt(np.mean((sr - rr) ** 2))),
            "velocity_rmse_rad_s": float(np.sqrt(np.mean((svr - rvr) ** 2))),
            "velocity_shape_correlation": float(np.corrcoef(rvr, svr)[0, 1]) if np.std(rvr) > 1e-8 and np.std(svr) > 1e-8 else np.nan,
            "onset_delta_s": None if rc["onset_s"] is None or sc["onset_s"] is None else sc["onset_s"] - rc["onset_s"],
            "peak_timing_delta_s": sc["peak_time_s"] - rc["peak_time_s"],
            "recovery_delta_s": None if rc["recovery_after_motion_s"] is None or sc["recovery_after_motion_s"] is None else sc["recovery_after_motion_s"] - rc["recovery_after_motion_s"],
            "xcorr_lag_s": lag_seconds(rr, sr),
        })
    return pd.DataFrame(rows), summary, safety


def imu_metrics(experiment_id: str, motion_end: float) -> pd.DataFrame:
    real = pd.read_csv(HERE / "phase3bv_aligned_imu_data.csv")
    sim = pd.read_csv(HERE / "runs" / f"{experiment_id}__{DATASET}__arm_only_safety_log.csv")
    rows = []
    for sensor in sorted(real.imu.unique()):
        r = real[real.imu == sensor].sort_values("t")
        for axis, real_col, sim_col in (
            ("roll", "relative_roll_rad", "base_roll_rad"),
            ("pitch", "relative_pitch_rad", "base_pitch_rad"),
        ):
            motion = r.t.between(0.0, motion_end)
            grid = r.loc[motion, "t"].to_numpy(float)
            rv = r.loc[motion, real_col].to_numpy(float)
            spre = float(sim.loc[sim.t.between(-3.0, -0.2), sim_col].mean())
            sv = np.interp(grid, sim.t, sim[sim_col]) - spre
            rows.append({
                "experiment_id": experiment_id, "sensor": sensor, "axis": axis,
                "real_relative_excursion_rad": float(rv.max() - rv.min()),
                "sim_relative_excursion_rad": float(sv.max() - sv.min()),
                "relative_shape_rmse_rad": float(np.sqrt(np.mean((sv - rv) ** 2))),
                "scope": "AUXILIARY_RELATIVE_ONLY_IMU_TRANSFORM_PARTIAL",
            })
    return pd.DataFrame(rows)


def completed_reports(metadata: dict, independence: dict) -> None:
    motion_end = float(metadata["motion_duration_seconds"])
    base, base_summary, base_safety = compare_variant(BASE_ID, motion_end)
    mass, mass_summary, mass_safety = compare_variant(MASS_ID, motion_end)
    metrics = pd.concat([base, mass], ignore_index=True)
    metrics.to_csv(HERE / "phase3bv_joint_comparison.csv", index=False)
    imu = pd.concat([imu_metrics(BASE_ID, motion_end), imu_metrics(MASS_ID, motion_end)], ignore_index=True)
    imu.to_csv(HERE / "phase3bv_relative_imu_comparison.csv", index=False)
    measured = pd.read_csv(HERE / "phase3bv_joint_metrics.csv")
    primary = set(measured.loc[measured.classification == "BALANCE_COMPENSATION_CANDIDATE", "joint_name"]) & set(BALANCE_JOINTS)
    b = base.set_index("joint_name")
    m = mass.set_index("joint_name")
    primary = sorted(primary & set(b.index) & set(m.index))
    improvements = pd.DataFrame([{
        "joint_name": joint,
        "baseline_abs_excursion_error_rad": b.loc[joint, "absolute_excursion_error_rad"],
        "mass_abs_excursion_error_rad": m.loc[joint, "absolute_excursion_error_rad"],
        "improvement_rad": b.loc[joint, "absolute_excursion_error_rad"] - m.loc[joint, "absolute_excursion_error_rad"],
    } for joint in primary])
    improvements.to_csv(HERE / "phase3bv_primary_leg_improvement.csv", index=False)
    baseline_mean = float(improvements.baseline_abs_excursion_error_rad.mean()) if len(improvements) else np.nan
    mass_mean = float(improvements.mass_abs_excursion_error_rad.mean()) if len(improvements) else np.nan
    aggregate_improvement = (baseline_mean - mass_mean) / max(baseline_mean, 1e-12) if len(improvements) else -np.inf
    improved_fraction = float((improvements.improvement_rad > 0.0).mean()) if len(improvements) else 0.0
    severe = False
    for joint in primary:
        old, new = float(b.loc[joint, "absolute_excursion_error_rad"]), float(m.loc[joint, "absolute_excursion_error_rad"])
        severe |= new > old + max(0.10 * old, 0.001)
    real_exc = base.set_index("joint_name").real_excursion_rad
    active_arm = [joint for joint in base.joint_name if any(token in joint for token in ("shoulder", "elbow", "wrist")) and real_exc.get(joint, 0.0) >= 0.02]
    arm_rmse_ratio = float(m.loc[active_arm, "position_rmse_rad"].mean() / max(b.loc[active_arm, "position_rmse_rad"].mean(), 1e-12)) if active_arm else np.nan
    arm_ok = bool(active_arm and arm_rmse_ratio <= 1.05)
    whole_safety = []
    for experiment in (BASE_ID, MASS_ID):
        path = HERE / "runs" / f"{experiment}__{DATASET}__whole_body_summary.json"
        whole_safety.append(bool(json.loads(path.read_text(encoding="utf-8"))["safety_pass"]))
    safety_ok = bool(base_summary["safety_pass"] and mass_summary["safety_pass"] and all(whole_safety))
    direction_ok = bool(
        independence.get("decision") == "SUFFICIENTLY_INDEPENDENT_FROM_HEART_AND_WAVE"
        and len(primary) > 0 and aggregate_improvement >= 0.05 and improved_fraction > 0.5
        and not severe and arm_ok and safety_ok
    )
    key_cols = ["joint_name", "real_excursion_rad", "sim_excursion_rad", "excursion_ratio", "absolute_excursion_error_rad", "position_rmse_rad", "velocity_rmse_rad_s", "velocity_shape_correlation", "onset_delta_s", "peak_timing_delta_s", "recovery_delta_s"]
    write_md("phase3bv_baseline_report.md", f"""
# Phase 3B-V original physical baseline

{md_table(base[key_cols])}

Relative IMU auxiliary metrics:

{md_table(imu[imu.experiment_id == BASE_ID])}
""")
    write_md("phase3bv_mass_direction_report.md", f"""
# Phase 3B-V `bs_mass_lower_plus08` direction

**SHARED_PHYSICAL_SENSITIVITY_DIRECTION — NOT IDENTIFIED HARDWARE PARAMETER**

{md_table(mass[key_cols])}

Primary leg absolute-error comparison:

{md_table(improvements)}

- aggregate primary-leg error improvement: `{100*aggregate_improvement:.3f}%`
- fraction of primary leg joints improved: `{improved_fraction:.3f}`
- active-arm mean RMSE ratio candidate/baseline: `{arm_rmse_ratio:.6f}`
""")
    prior_compare, _ = historical_rows()
    prior_key = prior_compare[
        ((prior_compare.dataset == "wave") & (prior_compare.joint_name == "right_knee_joint"))
        | ((prior_compare.dataset == "heart") & prior_compare.joint_name.isin(("left_ankle_pitch_joint", "waist_roll_joint")))
    ]
    write_md("phase3bv_cross_motion_report.md", f"""
# Phase 3B-V cross-motion report

Prior Heart/Wave locked evidence:

{md_table(prior_key[["experiment_id", "dataset", "joint_name", "real_excursion_rad", "sim_excursion_rad", "excursion_ratio", "absolute_excursion_error_rad"]])}

Blind `clap` primary-leg evidence:

{md_table(improvements)}

`PHYSICAL_DIRECTION_GENERALIZES = {'YES' if direction_ok else 'NO'}`
""")
    safety_table = pd.DataFrame([
        {"physical_condition": "original", "arm_only_safety": base_summary["safety_pass"], "whole_body_safety": whole_safety[0], "max_foot_slip_m": max(base_summary["maximum_left_foot_slip_proxy_m"], base_summary["maximum_right_foot_slip_proxy_m"]), "penetration_m": base_summary["maximum_pelvis_hip_penetration_m"]},
        {"physical_condition": "mass_direction", "arm_only_safety": mass_summary["safety_pass"], "whole_body_safety": whole_safety[1], "max_foot_slip_m": max(mass_summary["maximum_left_foot_slip_proxy_m"], mass_summary["maximum_right_foot_slip_proxy_m"]), "penetration_m": mass_summary["maximum_pelvis_hip_penetration_m"]},
    ])
    write_md("phase3bv_safety_report.md", f"""
# Phase 3B-V safety report

{md_table(safety_table)}

`SAFETY_BASELINE_PRESERVED = {'YES' if safety_ok else 'NO'}`
""")
    write_md("phase3bv_final_gate.md", f"""
# Phase 3B-V final gate

| Gate | Status |
|---|---|
| PHYSICAL_DIRECTION_GENERALIZES | {'YES' if direction_ok else 'NO'} |
| CONTROLLER_BASELINE_PRESERVED | {'YES' if arm_ok else 'NO'} |
| SAFETY_BASELINE_PRESERVED | {'YES' if safety_ok else 'NO'} |
| POSITION_SPACE_PHYSICAL_DIRECTION_VALIDATED | {'YES' if direction_ok and arm_ok and safety_ok else 'NO'} |
| DYNAMICS_CALIBRATION_READY | NO |

This remains position-space output-response validation. It is not real mass calibration, hardware mass identification, or actuator system identification.

Persistent blockers: `PHYSICAL_SIGN=UNKNOWN`, `PHYSICAL_ZERO=UNKNOWN`, `EFFORT_SEMANTICS=UNKNOWN`, `IMU_TRANSFORM=PARTIAL`, `MC_INTERNAL_COMMAND=UNOBSERVABLE`.
""")


def main() -> int:
    source_lock()
    metadata_path = HERE / "phase3bv_capture_metadata.json"
    independence_path = HERE / "phase3bv_independence.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8")) if metadata_path.exists() else None
    independence = json.loads(independence_path.read_text(encoding="utf-8")) if independence_path.exists() else None
    capture_ready = bool(metadata and metadata.get("data_ready") and independence and independence.get("decision") == "SUFFICIENTLY_INDEPENDENT_FROM_HEART_AND_WAVE")
    selected_motion_report(capture_ready, independence)
    execution_path = HERE / "phase3bv_replay_execution.json"
    if not capture_ready:
        pending_reports("no quality-gated third independent motion capture exists")
        print(json.dumps({"PHASE3BV_STATUS": "PENDING_CAPTURE", "robot_connected": False}, indent=2))
        return 0
    if not execution_path.exists():
        pending_reports("capture is ready but dual replay has not run")
        print(json.dumps({"PHASE3BV_STATUS": "PENDING_REPLAY", "robot_connected": False}, indent=2))
        return 0
    completed_reports(metadata, independence)
    print(json.dumps({"PHASE3BV_STATUS": "ANALYSIS_COMPLETE", "robot_connected": False}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
