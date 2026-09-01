#!/usr/bin/env python3
"""Refine roll-only clamping and combine it with pitch allocation evidence."""

from __future__ import annotations

from dataclasses import asdict, replace
import json

import pandas as pd

from phase3ar_core import Design, HERE, aggregate_score, datasets
from run_phase3ar_experiments import arm_preserved, balance_shape_score, execute


def load(experiment_id: str) -> list[dict[str, object]]:
    return [
        json.loads((HERE / "runs" / f"{experiment_id}__{dataset}__arm_only_summary.json").read_text(encoding="utf-8"))
        for dataset in ("heart", "wave")
    ]


def main() -> int:
    source = datasets()
    baseline = load("sweep_current_07")
    base = Design("channel_base", "CHANNEL_REFINEMENT", "none", "none", "none")
    designs = [
        replace(base, experiment_id="family_a_roll_clamp_2", candidate_family="A_CLAMP", parameter="roll clamp Nm", old_value="unbounded", new_value="2", roll_clamp_nm=2.0),
        replace(base, experiment_id="family_a_roll_clamp_3", candidate_family="A_CLAMP", parameter="roll clamp Nm", old_value="unbounded", new_value="3", roll_clamp_nm=3.0),
        replace(base, experiment_id="family_a_roll_clamp_4", candidate_family="A_CLAMP", parameter="roll clamp Nm", old_value="unbounded", new_value="4", roll_clamp_nm=4.0),
        replace(base, experiment_id="family_e_roll3_pitch1_hip0p10_knee0p15", candidate_family="E_COMBINATION", parameter="roll clamp + pitch allocation", old_value="current", new_value="roll3; ankle/hip/knee=1/0.10/0.15", roll_clamp_nm=3.0, ankle_pitch_weight=1.0, hip_pitch_weight=0.10, knee_pitch_weight=0.15),
        replace(base, experiment_id="family_e_roll3_pitch0p9_hip0p05_knee0p10", candidate_family="E_COMBINATION", parameter="roll clamp + pitch allocation", old_value="current", new_value="roll3; ankle/hip/knee=0.9/0.05/0.10", roll_clamp_nm=3.0, ankle_pitch_weight=0.9, hip_pitch_weight=0.05, knee_pitch_weight=0.10),
        replace(base, experiment_id="family_e_roll3_pitch0p85_hip0p10_knee0p15", candidate_family="E_COMBINATION", parameter="roll clamp + pitch allocation", old_value="current", new_value="roll3; ankle/hip/knee=0.85/0.10/0.15", roll_clamp_nm=3.0, ankle_pitch_weight=0.85, hip_pitch_weight=0.10, knee_pitch_weight=0.15),
    ]
    rows = []
    results = {}
    for design in designs:
        result = execute(design, source)
        results[design.experiment_id] = result
        aggregate = aggregate_score(result)
        preserved = arm_preserved(result, baseline)
        safe = bool(aggregate["all_safe"])
        by_dataset = {item["dataset"]: item for item in result}
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
                "balance_shape_score": balance_shape_score(result),
                "max_pelvis_hip_penetration_m": aggregate["max_contact_penetration_m"],
                "minimum_limit_margin_rad": aggregate["minimum_limit_margin_rad"],
                "maximum_persistent_saturation_fraction": aggregate["maximum_saturation_fraction"],
                "decision": "ACCEPTED_FOR_FINAL_VALIDATION" if safe and preserved else "REJECTED_OR_DIAGNOSTIC_ONLY",
                "warning": design.warning,
            }
        )
    path = HERE / "phase3ar_experiments.csv"
    frame = pd.concat([pd.read_csv(path), pd.DataFrame(rows)], ignore_index=True).drop_duplicates("experiment_id", keep="last")
    frame.to_csv(path, index=False)

    previous = Design(
        "family_c_ankle_0p7_hip_0p10_knee_0p15",
        "C_JOINT_ALLOCATION",
        "ankle/hip/knee pitch contribution",
        "current",
        "0.7/0.10/0.15",
        ankle_pitch_weight=0.7,
        hip_pitch_weight=0.10,
        knee_pitch_weight=0.15,
        ankle_roll_weight=0.7,
    )
    pool = [previous] + designs
    all_results = {previous.experiment_id: load(previous.experiment_id), **results}
    eligible = [
        design for design in pool
        if all(bool(item["safety_pass"]) for item in all_results[design.experiment_id])
        and arm_preserved(all_results[design.experiment_id], baseline)
    ]
    selected = min(eligible, key=lambda design: (balance_shape_score(all_results[design.experiment_id]), aggregate_score(all_results[design.experiment_id])["mean_balance_rmse_rad"]))
    payload = {
        "classification": "FINAL_VALIDATION_CANDIDATE",
        "warning": selected.warning,
        "selected_experiment_id": selected.experiment_id,
        "parameters": asdict(selected),
        "selection_basis": "heart+wave safety, preserved arm tracking, channel-specific response-shape score",
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
