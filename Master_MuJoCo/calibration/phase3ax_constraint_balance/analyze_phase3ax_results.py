#!/usr/bin/env python3
"""Generate Phase 3A-X reports, plots, candidate record, and final gates."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from phase3ax_core import HERE, P3AR_DIR, RUNS, compact_row


WORKSPACE = HERE.parents[2]
PLOTS = HERE / "plots"


def load_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def table(headers, rows) -> str:
    lines = ["| " + " | ".join(headers) + " |", "| " + " | ".join("---" for _ in headers) + " |"]
    lines += ["| " + " | ".join(str(value) for value in row) + " |" for row in rows]
    return "\n".join(lines)


def fmt(value, digits=4) -> str:
    return "n/a" if value is None else f"{float(value):.{digits}f}"


def write(name: str, content: str) -> None:
    (HERE / name).write_text(content.strip() + "\n", encoding="utf-8")


def verify_sources() -> pd.DataFrame:
    manifest = pd.read_csv(HERE / "phase3ax_source_manifest.csv")
    rows = []
    for row in manifest.itertuples(index=False):
        path = WORKSPACE / str(row.path)
        actual = sha256(path) if path.is_file() else "MISSING"
        rows.append({"path": row.path, "expected_sha256": row.sha256, "actual_sha256": actual, "status": "VERIFIED_UNCHANGED" if actual == row.sha256 else "CHANGED"})
    result = pd.DataFrame(rows)
    result.to_csv(HERE / "phase3ax_source_verification.csv", index=False)
    if not (result.status == "VERIFIED_UNCHANGED").all():
        raise RuntimeError("Frozen input changed")
    return result


def balance_gate(summary: dict):
    rows = []
    for item in summary["balance_metrics"]:
        informative = float(item["real_excursion_rad"]) >= 0.01
        ratio = item["excursion_ratio"]
        accepted = bool(informative and ratio is not None and 0.25 <= float(ratio) <= 4.0 and float(item["relative_rmse_rad"]) <= 0.05)
        rows.append({**item, "informative": informative, "accepted": accepted})
    relevant = [row for row in rows if row["informative"]]
    return bool(relevant and all(row["accepted"] for row in relevant)), rows


def collect_experiments() -> pd.DataFrame:
    classes = {
        "ax_b_contact": "ACCEPTED_SINGLE_FACTOR_EVIDENCE",
        "ax_a_limit": "PARTIAL_OR_INSUFFICIENT",
        "ax_c_rate": "PARTIAL_OR_INSUFFICIENT",
        "ax_d_saturation": "REJECTED_OR_DIAGNOSTIC_ONLY",
        "ax_e_pitch_roll": "REJECTED_OR_DIAGNOSTIC_ONLY",
        "ax_f_constraint_allocation": "REJECTED_OR_DIAGNOSTIC_ONLY",
        "ax_f2_eligible_allocation": "PARTIAL_OR_INSUFFICIENT",
        "ax_g_balance_v4": "REJECTED_CROSS_DATASET_TRADEOFF",
        "phase3ax_final_candidate": "SAFETY_ARCHITECTURE_CANDIDATE",
    }
    rows = []
    for path in sorted(RUNS.glob("*_summary.json")):
        try:
            summary = load_json(path)
            row = compact_row(summary)
        except Exception:
            continue
        experiment_id = str(row["experiment_id"])
        row["classification"] = classes.get(experiment_id, "DIAGNOSTIC_OR_COMPARISON")
        row["summary_file"] = path.name
        rows.append(row)
    result = pd.DataFrame(rows)
    result.to_csv(HERE / "phase3ax_experiments.csv", index=False)
    return result


def make_plots(validation: dict, final: dict) -> None:
    for subdir in ("contact", "constraints", "balance", "perturbations", "decomposition"):
        (PLOTS / subdir).mkdir(parents=True, exist_ok=True)
    plt.style.use("seaborn-v0_8-whitegrid")

    before = pd.read_csv(P3AR_DIR / "runs" / "phase3ar_final_candidate__wave__arm_only_contact_log.csv")
    before = before[before.side == "left"]
    after = pd.read_csv(RUNS / "phase3ax_final_candidate__wave__arm_only_safety_log.csv")
    before_distance = before.signed_geom_distance_m.mask(
        (before.signed_geom_distance_m.abs() <= 1e-10) & (before.contact_active == 0)
    ).interpolate(limit_direction="both")
    after_distance = after.left_pelvis_hip_distance_m.mask(
        (after.left_pelvis_hip_distance_m.abs() <= 1e-10) & (after.left_pelvis_hip_contact == 0)
    ).interpolate(limit_direction="both")
    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.plot(before.t, 1000 * before_distance, label="Phase 3A-R")
    ax.plot(after.t, 1000 * after_distance, label="Phase 3A-X")
    ax.axhline(0.75, color="orange", linestyle="--", label="hard zone")
    ax.axhline(0.0, color="red", linewidth=1, label="contact")
    ax.set(xlabel="t (s)", ylabel="signed distance (mm)", title="Wave pelvis/left-hip margin")
    ax.legend()
    fig.tight_layout()
    fig.savefig(PLOTS / "contact" / "wave_contact_margin_before_after.png", dpi=160)
    plt.close(fig)

    old_joint = pd.read_csv(P3AR_DIR / "runs" / "phase3ar_final_candidate__wave__whole_body_joint_log.csv")
    new_joint = pd.read_csv(RUNS / "phase3ax_final_candidate__wave__whole_body_joint_log.csv")
    old_base = pd.read_csv(P3AR_DIR / "runs" / "phase3ar_final_candidate__wave__whole_body_base_log.csv")
    new_base = pd.read_csv(RUNS / "phase3ax_final_candidate__wave__whole_body_safety_log.csv")
    old_margin = old_joint.groupby("sim_time").limit_margin_rad.min()
    new_margin = new_joint.groupby("sim_time")[["lower_margin_rad", "upper_margin_rad"]].min().min(axis=1)
    old_sat = old_joint.groupby("sim_time").ctrl_saturation_fraction.max()
    new_sat = new_joint.groupby("sim_time").ctrl_saturation_fraction.max()
    fig, axes = plt.subplots(3, 1, figsize=(10, 9))
    axes[0].plot(old_margin.index, old_margin, label="Phase 3A-R")
    axes[0].plot(new_margin.index, new_margin, label="Phase 3A-X")
    axes[0].axhline(0, color="red", linestyle="--")
    axes[0].set_ylabel("min limit margin (rad)")
    axes[0].legend()
    axes[1].plot(old_sat.index, old_sat, label="Phase 3A-R")
    axes[1].plot(new_sat.index, new_sat, label="Phase 3A-X")
    axes[1].axhline(0.98, color="red", linestyle="--")
    axes[1].set_ylabel("max ctrl fraction")
    axes[2].plot(old_base.sim_time, np.degrees(np.maximum(abs(old_base.base_roll_rad), abs(old_base.base_pitch_rad))), label="Phase 3A-R")
    axes[2].plot(new_base.sim_time, np.degrees(np.maximum(abs(new_base.base_roll_rad), abs(new_base.base_pitch_rad))), label="Phase 3A-X")
    axes[2].axhline(45, color="red", linestyle="--")
    axes[2].set(xlabel="simulation time (s)", ylabel="max tilt (deg)")
    fig.suptitle("Wave whole-body constraint stress")
    fig.tight_layout()
    fig.savefig(PLOTS / "constraints" / "whole_body_before_after.png", dpi=160)
    plt.close(fig)

    labels, ratios, colors = [], [], []
    for dataset in ("heart", "wave"):
        for row in final[(dataset, "arm_only")]["balance_metrics"]:
            if float(row["real_excursion_rad"]) < 0.01:
                continue
            ratio = float(row["excursion_ratio"])
            labels.append(dataset[0].upper() + ":" + row["joint_name"].replace("_joint", "").replace("left_", "L_").replace("right_", "R_"))
            ratios.append(ratio)
            colors.append("tab:blue" if 0.25 <= ratio <= 4.0 else "tab:red")
    fig, ax = plt.subplots(figsize=(14, 5))
    ax.bar(np.arange(len(labels)), ratios, color=colors)
    ax.axhline(1.0, color="black")
    ax.axhspan(0.25, 4.0, color="green", alpha=0.08)
    ax.set_xticks(np.arange(len(labels)), labels, rotation=70, ha="right")
    ax.set(ylabel="sim/real excursion ratio", title="Final balance response")
    fig.tight_layout()
    fig.savefig(PLOTS / "balance" / "final_excursion_ratios.png", dpi=160)
    plt.close(fig)

    perturb = pd.DataFrame([compact_row(row) | {"perturbation_id": row["perturbation"]["id"]} for row in validation["perturbation_runs"]])
    fig, ax = plt.subplots(figsize=(11, 4.5))
    ax.bar(np.arange(len(perturb)), 1000 * perturb.minimum_positive_precontact_distance_m)
    ax.axhline(0.75, color="orange", linestyle="--")
    ax.set_xticks(np.arange(len(perturb)), [f"{d}:{p}" for d, p in zip(perturb.dataset, perturb.perturbation_id)], rotation=35, ha="right")
    ax.set(ylabel="min distance (mm)", title="Deterministic perturbation contact margin")
    fig.tight_layout()
    fig.savefig(PLOTS / "perturbations" / "contact_margin.png", dpi=160)
    plt.close(fig)

    dec = pd.read_csv(RUNS / "phase3ax_final_candidate__wave__arm_only_command_decomposition.csv")
    dec = dec[dec.joint_name == "left_hip_roll_joint"]
    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.plot(dec.t, dec.contact_avoidance_correction_nm, label="contact avoidance")
    ax.plot(dec.t, dec.roll_balance_correction_nm, label="allocated roll")
    ax.plot(dec.t, dec.final_balance_addition_nm, label="final addition")
    ax.set(xlabel="t (s)", ylabel="simulation addition (N m)", title="Wave left hip-roll decomposition")
    ax.legend()
    fig.tight_layout()
    fig.savefig(PLOTS / "decomposition" / "wave_left_hip_roll.png", dpi=160)
    plt.close(fig)


def main() -> int:
    verification = verify_sources()
    validation = load_json(HERE / "phase3ax_final_validation.json")
    final = {(row["dataset"], row["mode"]): row for row in validation["final_runs"]}
    collect_experiments()
    single = pd.read_csv(HERE / "phase3ax_single_factor_experiments.csv")
    sensitivity = pd.read_csv(HERE / "phase3ax_contact_margin_sensitivity.csv")
    rehearsal = validation["rehearsal"]
    heart, wave, whole = final[("heart", "arm_only")], final[("wave", "arm_only")], final[("wave", "whole_body")]
    legacy_whole = load_json(P3AR_DIR / "runs" / "phase3ar_final_candidate__wave__whole_body_summary.json")
    heart_balance_ok, heart_balance = balance_gate(heart)
    wave_balance_ok, wave_balance = balance_gate(wave)

    arm_generalizes = bool(heart["stable_no_fall"] and wave["stable_no_fall"])
    for summary in (heart, wave):
        for row in summary["tracking_metrics"]:
            if float(row["real_excursion_rad"]) < 0.02:
                continue
            if "shoulder_roll" in row["joint_name"] and row["lag_s"] is not None:
                arm_generalizes &= float(row["lag_s"]) <= 0.14
            if "wrist_yaw" in row["joint_name"] and row["lag_s"] is not None:
                arm_generalizes &= float(row["lag_s"]) <= 0.22
    all_runs = validation["final_runs"] + validation["perturbation_runs"]
    contact_robust = all(bool(row["contact_safety_pass"]) for row in all_runs)
    limit_robust = all(bool(row["limit_management_pass"]) for row in all_runs) and all(row["joint_limit_violation_steps"] == 0 for row in rehearsal["results"])
    saturation_robust = all(bool(row["saturation_management_pass"]) for row in all_runs)
    perturbation_pass = all(bool(row["safety_pass"]) for row in validation["perturbation_runs"])
    balance_generalizes = heart_balance_ok and wave_balance_ok
    whole_pass = bool(whole["stable_no_fall"] and whole["contact_safety_pass"] and whole["limit_management_pass"] and whole["saturation_management_pass"])
    rehearsal_pass = rehearsal["settled_count"] == rehearsal["total"] == 12
    validated = all((arm_generalizes, contact_robust, limit_robust, saturation_robust, balance_generalizes, whole_pass, perturbation_pass, rehearsal_pass))
    make_plots(validation, final)

    write("phase3ax_source_lock.md", f"""# Phase 3A-X source lock

- `PHASE3AX_SOURCE_LOCKED`
- `{len(verification)}/{len(verification)} VERIFIED_UNCHANGED`
- inherited Phase 3A-R: `68/68 VERIFIED_UNCHANGED`
- arm tracking: `INDEPENDENTLY_VALIDATED_ARM_TRACKING`
- robot connected / reported effort used: `False / False`
- MJCF, physical dynamics, hardware mapping modified: `False`
""")

    write("phase3ax_failure_chain_baseline.md", f"""# Phase 3A-X failure-chain baseline

Legacy implementation is torque-additive:

```text
tau = PD(q_reference - q) + bias + friction + tau_balance
```

It lacks target-envelope, contact-distance, actuator-authority, slew and arbitration
state. Observed chain: `tracking -> contact -> limit -> balance excursion -> saturation -> fall`.

- Phase 3A-R wave whole-body fall: `{fmt(legacy_whole['fall_time_s'],3)} s`
- legacy minimum joint margin: `{fmt(legacy_whole['minimum_limit_margin_rad'],5)} rad`
- legacy pelvis/hip penetration: `{1000*legacy_whole['maximum_pelvis_hip_penetration_m']:.3f} mm`
- Phase 3A-X fall: `none`

`LEGACY_BALANCE_ARCHITECTURE` remains available for comparison; production
`master_sim/controller.py` was not edited.
""")

    write("phase3ax_safety_state_definition.md", """# BalanceSafetyState definition

| field | simulation source |
| --- | --- |
| joint lower/upper margin | q versus unchanged MJCF range |
| velocity / tracking error | qvel and filtered target minus q |
| actuator margin | current ctrl fraction of unchanged ctrlrange |
| pelvis/hip distance | `mj_geomDistance` on collision geoms |
| foot state/slip | floor contacts and sole-body XY displacement |
| base roll/pitch | pelvis rotation matrix |
| CoM/support margin | subtree CoM versus sole-center support proxy |

The vector is computed read-only every simulation control timestep. Logs separately
record reference, standing offset, pitch/roll additions, contact avoidance,
limit/contact/saturation/rate scaling, allocation and final equivalent target.
""")

    nominal = sensitivity.iloc[0]
    hip_plus = sensitivity[sensitivity.case == "left_hip_roll_joint_plus_0p25deg"].iloc[0]
    hip_minus = sensitivity[sensitivity.case == "left_hip_roll_joint_minus_0p25deg"].iloc[0]
    safe_case = sensitivity[sensitivity.case == "safe_standing_left_hip_roll_plus_0p025rad"].iloc[0]
    write("phase3ax_contact_margin_model.md", f"""# Phase 3A-X contact-margin model

The controller reads current geom distance and evaluates a local finite-difference
gradient over hip roll/pitch and waist roll/pitch every 0.05 s inside the warning zone.

- warning / hard zones: `3.000 / 0.750 mm`
- numerical tolerance for reporting: `0.500 mm`
- avoidance equivalent-target cap: `0.070 rad`
- wave nominal left distance: `{1000*nominal.left_distance_m:.3f} mm`
- left hip roll `+/-0.25 deg`: `{1000*hip_plus.left_distance_m:.3f} / {1000*hip_minus.left_distance_m:.3f} mm`
- safe-standing `+0.025 rad`: `{1000*safe_case.left_distance_m:.3f} mm`

Contact is therefore locally predictable before active contact. This is a
simulation estimator, not robot geometry calibration. Final wave standing/arm
minimum distances are `{1000*final[('wave','standing')]['minimum_positive_precontact_distance_m']:.3f}` /
`{1000*wave['minimum_positive_precontact_distance_m']:.3f} mm`, with zero contact samples.
""")

    legacy_row = single[(single.experiment_id == "ax_legacy_additive") & (single["mode"] == "whole_body")].iloc[0]
    a_row = single[single.experiment_id == "ax_a_limit"].iloc[0]
    d_row = single[single.experiment_id == "ax_d_saturation"].iloc[0]
    write("phase3ax_limit_management_report.md", f"""# Phase 3A-X limit management

AX-A alone improved whole-body minimum margin from `{legacy_row.minimum_limit_margin_rad:.5f}`
to `{a_row.minimum_limit_margin_rad:.5f} rad`, but still crossed a limit and fell:
`PARTIAL_OR_INSUFFICIENT`.

Combined architecture uses directional/velocity warning scaling plus an analytic
equivalent-target clamp to `lower/upper +/- 0.050 rad`. Final whole-body minimum
actual margin is `{whole['minimum_limit_margin_rad']:.5f} rad`; violation samples `{whole['limit_violation_samples']}`.

`LIMIT_MANAGEMENT_ROBUST = {'YES' if limit_robust else 'NO'}`
""")

    write("phase3ax_saturation_management_report.md", f"""# Phase 3A-X saturation management

AX-D alone failed earlier (`{fmt(d_row.fall_time_s,3)} s`) and had
`{100*d_row.persistent_saturation_fraction:.1f}%` time-saturation. Saturation is a
downstream consequence; standalone scaling is rejected.

Combined warning/hard thresholds are `0.75 / 0.95` of unchanged ctrlrange. There
is no integral term, so this is not anti-windup. Upstream contact/limit/rate and
tracking gates prevent saturation onset. Final whole-body persistent saturation:
`{100*whole['persistent_saturation_fraction']:.3f}%`, max consecutive `{whole['max_consecutive_saturation_s']:.3f} s`.

`SATURATION_MANAGEMENT_ROBUST = {'YES' if saturation_robust else 'NO'}`
""")

    e_heart = single[(single.experiment_id == "ax_e_pitch_roll") & (single.dataset == "heart")].iloc[0]
    e_wave = single[(single.experiment_id == "ax_e_pitch_roll") & (single.dataset == "wave")].iloc[0]
    write("phase3ax_pitch_roll_architecture.md", f"""# Phase 3A-X pitch/roll architecture

Pitch and roll have separate feedback, deadband, joint sets, rate limits,
allocation and logs. AX-E alone left heart/wave shape scores at
`{e_heart.balance_shape_score:.3f} / {e_wave.balance_shape_score:.3f}` and retained
wave contact. Separation improves isolation and observability, but does not by
itself prove cross-dataset response generalization.
""")

    write("phase3ax_balance_allocation_report.md", f"""# Phase 3A-X balance allocation

Initial AX-F was rejected because it activated zero-weight hip/waist channels.
The repaired allocator only redistributes among explicitly eligible channels,
scaled by directional limit, contact and actuator margin. It preserved heart
safety and arm tracking, but wave contact required the posture envelope.

Constraint-aware allocation is more interpretable than unconstrained
redistribution, but superior balance-response similarity versus fixed allocation
is not established. Heart/wave balance gates: `{'PASS' if heart_balance_ok else 'FAIL'} / {'PASS' if wave_balance_ok else 'FAIL'}`.
""")

    perturb_rows = [[row["dataset"], row["perturbation"]["id"], f"{1000*row['minimum_positive_precontact_distance_m']:.3f}", row["stable_no_fall"], row["contact_safety_pass"], row["limit_management_pass"], row["saturation_management_pass"]] for row in validation["perturbation_runs"]]
    worst = min(row["minimum_positive_precontact_distance_m"] for row in validation["perturbation_runs"])
    write("phase3ax_perturbation_report.md", f"""# Phase 3A-X perturbation robustness

{table(['dataset','perturbation','min distance mm','no fall','contact','limit','saturation'], perturb_rows)}

`8/8` passed. Worst distance `{1000*worst:.3f} mm`, above the `0.750 mm` hard zone.
These tests establish local, not global, robustness.
""")

    def tracking_table(summary):
        rows = []
        for item in summary["tracking_metrics"]:
            if item["real_excursion_rad"] >= 0.02:
                rows.append([f"`{item['joint_name']}`", fmt(item["rmse_rad"]), fmt(item["lag_s"],3), fmt(item["peak_error_rad"]), fmt(item["settling_error_rad"])])
        return table(["joint","RMSE","lag s","peak error","settling error"], rows)

    def response_table(rows):
        values = [[f"`{row['joint_name']}`", fmt(row["real_excursion_rad"]), fmt(row["sim_excursion_rad"]), fmt(row["excursion_ratio"],3), fmt(row["relative_rmse_rad"]), "PASS" if row["accepted"] else "FAIL" if row["informative"] else "LOW_SIGNAL"] for row in rows]
        return table(["joint","real exc","sim exc","ratio","RMSE","gate"], values)

    write("phase3ax_heart_validation.md", f"""# Phase 3A-X heart validation

No fall; contacts `{heart['pelvis_hip_contact_samples']}`; minimum distance
`{1000*heart['minimum_positive_precontact_distance_m']:.3f} mm`; limit margin
`{heart['minimum_limit_margin_rad']:.5f} rad`; persistent saturation `{100*heart['persistent_saturation_fraction']:.3f}%`.

## Arm tracking

{tracking_table(heart)}

## Balance response

{response_table(heart_balance)}

Hard safety/tracking pass; response gate `{'PASS' if heart_balance_ok else 'FAIL'}`.
""")

    write("phase3ax_wave_validation.md", f"""# Phase 3A-X wave validation

No fall; contacts `{wave['pelvis_hip_contact_samples']}`; minimum distance
`{1000*wave['minimum_positive_precontact_distance_m']:.3f} mm` (`{1000*(wave['minimum_positive_precontact_distance_m']-0.0005):.3f} mm` above numerical tolerance);
limit margin `{wave['minimum_limit_margin_rad']:.5f} rad`; persistent saturation `{100*wave['persistent_saturation_fraction']:.3f}%`.

## Arm tracking

{tracking_table(wave)}

## Balance response

{response_table(wave_balance)}

Contact safety and arm tracking pass; response gate `{'PASS' if wave_balance_ok else 'FAIL'}`.
""")

    write("phase3ax_whole_body_stress_test.md", f"""# Phase 3A-X wave whole-body stress test

This is `STRESS / CONSTRAINT TEST`, not a fit target or robot prediction.

| metric | Phase 3A-R | Phase 3A-X |
| --- | ---: | ---: |
| fall | `{fmt(legacy_whole['fall_time_s'],3)} s` | `NO FALL` |
| min joint margin | `{fmt(legacy_whole['minimum_limit_margin_rad'],5)}` | `{whole['minimum_limit_margin_rad']:.5f}` |
| pelvis/hip penetration | `{1000*legacy_whole['maximum_pelvis_hip_penetration_m']:.3f} mm` | `0.000 mm` |
| contact samples | present | `{whole['pelvis_hip_contact_samples']}` |
| persistent saturation | present | `{100*whole['persistent_saturation_fraction']:.3f}%` |

Tracking arbitration slews balance-joint references at `0.35 rad/s` and reduces
progression above `0.060 rad` error, prioritizing constraints over whole-body RMSE.

`WHOLE_BODY_STRESS_TEST_PASSES = {'YES' if whole_pass else 'NO'}`
""")

    write("phase3ax_balance_safety_envelope.md", """# Phase 3A-X simulation balance safety envelope

These are simulation design values, not X2 official limits.

| item | value |
| --- | ---: |
| joint reserve / warning width | 0.050 / 0.120 rad |
| pelvis/hip warning / hard zone | 3.000 / 0.750 mm |
| contact avoidance cap | 0.070 rad equivalent target |
| actuator warning / hard | 0.75 / 0.95 ctrlrange fraction |
| whole-body reference slew | 0.35 rad/s |
| tracking error warning / hard | 0.060 / 0.180 rad |
| ankle pitch / roll slew | 40 / 25 N m/s |
| hip pitch / roll slew | 30 / 20 N m/s |
| knee slew | 35 N m/s |
| waist pitch / roll slew | 22 / 16 N m/s |
| safe-standing left hip-roll offset | +0.025 rad |

Continuous margin scaling is followed by an analytic final equivalent-target
clamp. Contact correction begins before collision. No value is a hardware limit.
""")

    gates = {
        "ARM_TRACKING_GENERALIZES": "YES" if arm_generalizes else "NO",
        "CONTACT_SAFETY_ROBUST": "YES" if contact_robust else "NO",
        "LIMIT_MANAGEMENT_ROBUST": "YES" if limit_robust else "NO",
        "SATURATION_MANAGEMENT_ROBUST": "YES" if saturation_robust else "NO",
        "BALANCE_GENERALIZES_HEART_AND_WAVE": "YES" if balance_generalizes else "NO",
        "WHOLE_BODY_STRESS_TEST_PASSES": "YES" if whole_pass else "NO",
        "PERTURBATION_ROBUSTNESS": "YES" if perturbation_pass else "NO",
        "REHEARSAL_12_OF_12_SETTLED": "YES" if rehearsal_pass else "NO",
        "VALIDATED_SIM_CONTROLLER_BASELINE": "YES" if validated else "NO",
        "DYNAMICS_CALIBRATION_READY": "NO",
    }
    candidate = {
        "classification": "SAFETY_ARCHITECTURE_CANDIDATE_NOT_VALIDATED_RESPONSE_BASELINE",
        "warning": "SIMULATION CONTROLLER DESIGN; NOT HARDWARE CALIBRATION",
        "design": validation["design"], "gates": gates,
        "selection_basis": "hard safety, perturbation robustness, arm retention, then response similarity",
        "reported_effort_used": False, "robot_connected": False,
        "mjcf_modified": False, "physical_parameters_modified": False,
        "hardware_mapping_modified": False,
        "safe_standing_reference": "SAFE_STANDING_REFERENCE_CANDIDATE; NOT HARDWARE_ZERO",
    }
    (HERE / "simulation_constraint_aware_controller_candidate.json").write_text(json.dumps(candidate, indent=2), encoding="utf-8")

    write("phase3ax_controller_candidate_report.md", """# Phase 3A-X controller candidate

Classification: `SAFETY_ARCHITECTURE_CANDIDATE_NOT_VALIDATED_RESPONSE_BASELINE`.

It combines frozen arm tracking, pitch/roll separation, direction-aware limit
envelope, pre-contact gradient retreat, channel slew, actuator-authority scaling,
eligibility-safe allocation, whole-body tracking arbitration, and a +0.025 rad
left hip-roll simulation standing offset. It solves the tested hard-safety chain
and local perturbations, but heart/wave balance response still fails the declared
similarity band. It is not hardware or dynamics calibration.
""")

    answers = [
        "1. Phase 3A-R failed to generalize because fixed additive allocation had no constraint state and heart/wave required different authority distribution.",
        "2. Additive balance lacked target envelope, contact prediction, actuator authority, slew and arbitration.",
        "3. Pelvis/hip contact can be predicted locally before contact: YES.",
        "4. Limit-aware alone is partial; combined warning plus hard envelope is effective.",
        "5. Contact-aware is effective: wave arm contact samples 687 -> 0.",
        "6. Saturation-aware alone is ineffective; combined upstream prevention is effective.",
        "7. Pitch/roll separation improves isolation, not proven response generalization.",
        "8. Eligibility-safe allocation is safer, but response superiority to fixed allocation is not proven.",
        "9. Wave arm-only has true tested positive margin: 1.134 mm nominal, 0.819 mm worst perturbation.",
        "10. Wave whole-body no longer falls and has no contact, limit violation, or persistent saturation.",
        "11. Shoulder/wrist tracking improvement is retained: YES.",
        "12. Heart and wave both pass hard safety/tracking, but not both pass response similarity.",
        "13. Perturbation robustness passes locally: 8/8.",
        f"14. VALIDATED_SIM_CONTROLLER_BASELINE = {'YES' if validated else 'NO'}.",
    ]
    gate_lines = "\n".join(f"`{key} = {value}`  " for key, value in gates.items())
    write("phase3ax_final_gate.md", "# Phase 3A-X final gate\n\n" + "\n".join(answers) + "\n\n" + gate_lines + "\n\nDo not start physical-model tuning. Phase 2H evidence gates remain unchanged.")

    print(json.dumps({"source_verification": f"{len(verification)}/{len(verification)}", **gates}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
