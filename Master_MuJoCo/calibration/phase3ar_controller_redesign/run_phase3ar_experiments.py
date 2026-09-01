#!/usr/bin/env python3
"""Run Phase 3A-R single-factor families, then evidence-based combinations."""

from __future__ import annotations

from dataclasses import asdict, replace
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd

from phase3ar_core import (
    BALANCE_JOINTS,
    Design,
    HERE,
    aggregate_score,
    datasets,
    run_replay,
)


def balance_shape_score(summaries: list[dict[str, object]]) -> float:
    values = []
    for summary in summaries:
        for row in summary["balance_metrics"]:
            real = max(float(row["real_excursion_rad"]), 0.005)
            values.append(abs(float(row["sim_excursion_rad"]) - float(row["real_excursion_rad"])) / real)
    return float(np.mean(values)) if values else math.inf


def arm_preserved(summaries: list[dict[str, object]], baseline: list[dict[str, object]]) -> bool:
    base = {
        (summary["dataset"], row["joint_name"]): row
        for summary in baseline
        for row in summary["tracking_metrics"]
        if float(row["real_excursion_rad"]) >= 0.02
    }
    for summary in summaries:
        for row in summary["tracking_metrics"]:
            key = (summary["dataset"], row["joint_name"])
            if key not in base or float(row["real_excursion_rad"]) < 0.02:
                continue
            if float(row["rmse_rad"]) > 1.15 * float(base[key]["rmse_rad"]) + 0.002:
                return False
            lag = row["lag_s"]
            base_lag = base[key]["lag_s"]
            if lag is not None and base_lag is not None and abs(float(lag)) > abs(float(base_lag)) + 0.04:
                return False
    return True


def severity(summaries: list[dict[str, object]]) -> tuple[float, ...]:
    return (
        float(sum(not bool(item["stable_no_fall"]) for item in summaries)),
        float(sum(int(item["nonfoot_ground_contact_samples"]) for item in summaries)),
        float(sum(int(item["pelvis_hip_over_tolerance_samples"]) for item in summaries)),
        max(0.0, -min(float(item["minimum_limit_margin_rad"]) for item in summaries)),
        max(float(item["persistent_saturation_fraction"]) for item in summaries),
        balance_shape_score(summaries),
        float(aggregate_score(summaries)["mean_balance_rmse_rad"]),
    )


def execute(design: Design, source, *, detail: bool = False) -> list[dict[str, object]]:
    results = []
    for dataset_name in ("heart", "wave"):
        print(f"RUN {design.experiment_id} {dataset_name} arm-only", flush=True)
        results.append(
            run_replay(
                design,
                source[dataset_name],
                "arm_only",
                pre_s=2.0,
                post_s=2.0,
                save_detail=detail,
            )
        )
    return results


def family_designs() -> list[Design]:
    current = Design("sweep_current_07", "CURRENT", "none", "none", "none")
    return [
        current,
        Design(
            "ab_improved_arm_legacy_balance",
            "A_B_SUBSYSTEM_ISOLATION",
            "balance feedback",
            "0.7x",
            "legacy 1.0x with arm 8x retained",
            pitch_kp=200.0,
            pitch_kd=30.0,
            roll_kp=100.0,
            roll_kd=20.0,
        ),
        replace(current, experiment_id="family_a_clamp_3_1p5", candidate_family="A_CLAMP", parameter="pitch/roll clamp Nm", old_value="unbounded", new_value="3/1.5", pitch_clamp_nm=3.0, roll_clamp_nm=1.5),
        replace(current, experiment_id="family_a_clamp_5_2p5", candidate_family="A_CLAMP", parameter="pitch/roll clamp Nm", old_value="unbounded", new_value="5/2.5", pitch_clamp_nm=5.0, roll_clamp_nm=2.5),
        replace(current, experiment_id="family_a_clamp_8_4", candidate_family="A_CLAMP", parameter="pitch/roll clamp Nm", old_value="unbounded", new_value="8/4", pitch_clamp_nm=8.0, roll_clamp_nm=4.0),
        replace(current, experiment_id="family_b_rate_20_10", candidate_family="B_RATE_LIMIT", parameter="pitch/roll slew Nm/s", old_value="unbounded", new_value="20/10", pitch_rate_nm_s=20.0, roll_rate_nm_s=10.0),
        replace(current, experiment_id="family_b_rate_50_25", candidate_family="B_RATE_LIMIT", parameter="pitch/roll slew Nm/s", old_value="unbounded", new_value="50/25", pitch_rate_nm_s=50.0, roll_rate_nm_s=25.0),
        replace(current, experiment_id="family_b_rate_100_50", candidate_family="B_RATE_LIMIT", parameter="pitch/roll slew Nm/s", old_value="unbounded", new_value="100/50", pitch_rate_nm_s=100.0, roll_rate_nm_s=50.0),
        replace(current, experiment_id="family_c_ankle_0p7", candidate_family="C_JOINT_ALLOCATION", parameter="ankle contribution", old_value="1.0", new_value="0.7", ankle_pitch_weight=0.7, ankle_roll_weight=0.7),
        replace(current, experiment_id="family_c_distributed_light", candidate_family="C_JOINT_ALLOCATION", parameter="ankle/hip/knee/waist weights", old_value="1/0/0/0", new_value="0.7/-0.04/0.03/-0.02", ankle_pitch_weight=0.7, hip_pitch_weight=-0.04, knee_pitch_weight=0.03, waist_pitch_weight=-0.02, ankle_roll_weight=0.7, hip_roll_weight=-0.04, waist_roll_weight=-0.02),
        replace(current, experiment_id="family_c_distributed_medium", candidate_family="C_JOINT_ALLOCATION", parameter="ankle/hip/knee/waist weights", old_value="1/0/0/0", new_value="0.5/-0.10/0.05/-0.05", ankle_pitch_weight=0.5, hip_pitch_weight=-0.10, knee_pitch_weight=0.05, waist_pitch_weight=-0.05, ankle_roll_weight=0.5, hip_roll_weight=-0.10, waist_roll_weight=-0.05),
        replace(current, experiment_id="family_d_standing_0p75", candidate_family="D_STANDING_REFERENCE", parameter="standing reference scale", old_value="1.0", new_value="0.75", standing_reference_scale=0.75),
        replace(current, experiment_id="family_d_standing_0p50", candidate_family="D_STANDING_REFERENCE", parameter="standing reference scale", old_value="1.0", new_value="0.50", standing_reference_scale=0.50),
        replace(current, experiment_id="family_d_standing_0p25", candidate_family="D_STANDING_REFERENCE", parameter="standing reference scale", old_value="1.0", new_value="0.25", standing_reference_scale=0.25),
        replace(current, experiment_id="family_d_left_hip_offset_0", candidate_family="D_STANDING_REFERENCE", parameter="left hip-pitch offset scale", old_value="1.0", new_value="0.0", left_hip_pitch_offset_scale=0.0),
        replace(current, experiment_id="family_d_left_knee_offset_0p5", candidate_family="D_STANDING_REFERENCE", parameter="left knee offset scale", old_value="1.0", new_value="0.5", left_knee_offset_scale=0.5),
    ]


def apply_selected(base: Design, selected: Design) -> Design:
    fields = {
        "A_CLAMP": ("pitch_clamp_nm", "roll_clamp_nm"),
        "B_RATE_LIMIT": ("pitch_rate_nm_s", "roll_rate_nm_s"),
        "C_JOINT_ALLOCATION": (
            "ankle_pitch_weight", "hip_pitch_weight", "knee_pitch_weight", "waist_pitch_weight",
            "ankle_roll_weight", "hip_roll_weight", "waist_roll_weight",
        ),
        "D_STANDING_REFERENCE": (
            "standing_reference_scale", "left_hip_pitch_offset_scale", "left_knee_offset_scale",
        ),
    }[selected.candidate_family]
    return replace(base, **{field: getattr(selected, field) for field in fields})


def main() -> int:
    source = datasets()
    designs = family_designs()
    results: dict[str, list[dict[str, object]]] = {}
    for design in designs:
        results[design.experiment_id] = execute(design, source)
    baseline = results["sweep_current_07"]

    best: dict[str, Design] = {}
    for family in ("A_CLAMP", "B_RATE_LIMIT", "C_JOINT_ALLOCATION", "D_STANDING_REFERENCE"):
        candidates = [design for design in designs if design.candidate_family == family]
        best[family] = min(candidates, key=lambda design: severity(results[design.experiment_id]))

    current = designs[0]
    d_base = apply_selected(current, best["D_STANDING_REFERENCE"])
    combinations = [
        replace(apply_selected(d_base, best["A_CLAMP"]), experiment_id="family_e_standing_plus_clamp", candidate_family="E_COMBINATION", parameter="D+A", old_value="current", new_value=f"{best['D_STANDING_REFERENCE'].experiment_id}+{best['A_CLAMP'].experiment_id}"),
        replace(apply_selected(d_base, best["B_RATE_LIMIT"]), experiment_id="family_e_standing_plus_rate", candidate_family="E_COMBINATION", parameter="D+B", old_value="current", new_value=f"{best['D_STANDING_REFERENCE'].experiment_id}+{best['B_RATE_LIMIT'].experiment_id}"),
        replace(apply_selected(d_base, best["C_JOINT_ALLOCATION"]), experiment_id="family_e_standing_plus_allocation", candidate_family="E_COMBINATION", parameter="D+C", old_value="current", new_value=f"{best['D_STANDING_REFERENCE'].experiment_id}+{best['C_JOINT_ALLOCATION'].experiment_id}"),
    ]
    all_combined = d_base
    for family in ("A_CLAMP", "B_RATE_LIMIT", "C_JOINT_ALLOCATION"):
        all_combined = apply_selected(all_combined, best[family])
    combinations.append(replace(all_combined, experiment_id="family_e_all_evidence", candidate_family="E_COMBINATION", parameter="D+A+B+C", old_value="current", new_value="evidence-selected single-factor values", limit_guard_margin_rad=0.05))
    for design in combinations:
        designs.append(design)
        results[design.experiment_id] = execute(design, source)

    rows = []
    for design in designs:
        summaries = results[design.experiment_id]
        aggregate = aggregate_score(summaries)
        preserved = arm_preserved(summaries, baseline)
        shape = balance_shape_score(summaries)
        safe = bool(aggregate["all_safe"])
        if safe and preserved:
            decision = "ACCEPTED_FOR_FINAL_VALIDATION"
        elif preserved and int(aggregate["safety_pass_count"]) > int(aggregate_score(baseline)["safety_pass_count"]):
            decision = "DIAGNOSTIC_PROMISING"
        else:
            decision = "REJECTED_OR_DIAGNOSTIC_ONLY"
        by_dataset = {summary["dataset"]: summary for summary in summaries}
        rows.append(
            {
                "experiment_id": design.experiment_id,
                "candidate_family": design.candidate_family,
                "parameter": design.parameter,
                "old_value": design.old_value,
                "new_value": design.new_value,
                "heart_result": "PASS" if by_dataset["heart"]["safety_pass"] else "FAIL",
                "wave_result": "PASS" if by_dataset["wave"]["safety_pass"] else "FAIL",
                "safety_result": "PASS" if safe else "FAIL",
                "arm_tracking_preserved": preserved,
                "mean_arm_rmse_rad": aggregate["mean_arm_rmse_rad"],
                "mean_balance_rmse_rad": aggregate["mean_balance_rmse_rad"],
                "balance_shape_score": shape,
                "max_pelvis_hip_penetration_m": aggregate["max_contact_penetration_m"],
                "minimum_limit_margin_rad": aggregate["minimum_limit_margin_rad"],
                "maximum_persistent_saturation_fraction": aggregate["maximum_saturation_fraction"],
                "decision": decision,
                "warning": design.warning,
            }
        )
    frame = pd.DataFrame(rows)
    frame.to_csv(HERE / "phase3ar_experiments.csv", index=False)

    eligible = [
        design for design in designs
        if design.candidate_family in {"D_STANDING_REFERENCE", "E_COMBINATION"}
        and all(bool(item["safety_pass"]) for item in results[design.experiment_id])
        and arm_preserved(results[design.experiment_id], baseline)
    ]
    if eligible:
        selected = min(eligible, key=lambda design: (balance_shape_score(results[design.experiment_id]), aggregate_score(results[design.experiment_id])["mean_balance_rmse_rad"]))
        classification = "FINAL_VALIDATION_CANDIDATE"
    else:
        selected = min(combinations + [best["D_STANDING_REFERENCE"]], key=lambda design: severity(results[design.experiment_id]))
        classification = "BEST_AVAILABLE_BUT_GATE_NOT_MET"
    payload = {
        "classification": classification,
        "warning": selected.warning,
        "selected_experiment_id": selected.experiment_id,
        "parameters": asdict(selected),
        "single_factor_selections": {family: design.experiment_id for family, design in best.items()},
        "reported_effort_used_for_fitting": False,
        "physical_parameters_modified": False,
        "mjcf_modified": False,
        "hardware_mapping_modified": False,
    }
    (HERE / "simulation_controller_robustness_candidate.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
