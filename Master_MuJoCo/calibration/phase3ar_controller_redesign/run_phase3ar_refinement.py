#!/usr/bin/env python3
"""Evidence-driven Family C refinement after the broad A-D sweep."""

from __future__ import annotations

from dataclasses import asdict, replace
import json

import pandas as pd

from phase3ar_core import Design, HERE, aggregate_score, datasets
from run_phase3ar_experiments import arm_preserved, balance_shape_score, execute


def load_summaries(experiment_id: str) -> list[dict[str, object]]:
    result = []
    for dataset in ("heart", "wave"):
        path = HERE / "runs" / f"{experiment_id}__{dataset}__arm_only_summary.json"
        result.append(json.loads(path.read_text(encoding="utf-8")))
    return result


def main() -> int:
    source = datasets()
    baseline = load_summaries("sweep_current_07")
    base = Design("refine_base", "C_JOINT_ALLOCATION", "allocation", "current", "refined")
    designs = [
        replace(base, experiment_id="family_c_pitch_0p7_roll_1p0", parameter="pitch/roll ankle contribution", new_value="0.7/1.0", ankle_pitch_weight=0.7, ankle_roll_weight=1.0),
        replace(base, experiment_id="family_c_pitch_1p0_roll_0p7", parameter="pitch/roll ankle contribution", new_value="1.0/0.7", ankle_pitch_weight=1.0, ankle_roll_weight=0.7),
        replace(base, experiment_id="family_c_pitch_0p7_roll_0p8", parameter="pitch/roll ankle contribution", new_value="0.7/0.8", ankle_pitch_weight=0.7, ankle_roll_weight=0.8),
        replace(base, experiment_id="family_c_ankle_0p7_knee_0p02", parameter="ankle/knee pitch contribution", new_value="0.7/0.02", ankle_pitch_weight=0.7, knee_pitch_weight=0.02, ankle_roll_weight=0.7),
        replace(base, experiment_id="family_c_ankle_0p7_hip_0p02_knee_0p03", parameter="ankle/hip/knee pitch contribution", new_value="0.7/0.02/0.03", ankle_pitch_weight=0.7, hip_pitch_weight=0.02, knee_pitch_weight=0.03, ankle_roll_weight=0.7),
        replace(base, experiment_id="family_c_ankle_0p8_hip_0p02_knee_0p03", parameter="ankle/hip/knee pitch contribution", new_value="0.8/0.02/0.03", ankle_pitch_weight=0.8, hip_pitch_weight=0.02, knee_pitch_weight=0.03, ankle_roll_weight=0.7),
        replace(base, experiment_id="family_c_ankle_0p7_hip_0p05_knee_0p06", parameter="ankle/hip/knee pitch contribution", new_value="0.7/0.05/0.06", ankle_pitch_weight=0.7, hip_pitch_weight=0.05, knee_pitch_weight=0.06, ankle_roll_weight=0.7),
        replace(base, experiment_id="family_c_ankle_0p7_hip_0p05_knee_0p10", parameter="ankle/hip/knee pitch contribution", new_value="0.7/0.05/0.10", ankle_pitch_weight=0.7, hip_pitch_weight=0.05, knee_pitch_weight=0.10, ankle_roll_weight=0.7),
        replace(base, experiment_id="family_c_ankle_0p7_hip_0p10_knee_0p10", parameter="ankle/hip/knee pitch contribution", new_value="0.7/0.10/0.10", ankle_pitch_weight=0.7, hip_pitch_weight=0.10, knee_pitch_weight=0.10, ankle_roll_weight=0.7),
        replace(base, experiment_id="family_c_ankle_0p7_hip_0p10_knee_0p15", parameter="ankle/hip/knee pitch contribution", new_value="0.7/0.10/0.15", ankle_pitch_weight=0.7, hip_pitch_weight=0.10, knee_pitch_weight=0.15, ankle_roll_weight=0.7),
    ]
    summaries = {}
    new_rows = []
    for design in designs:
        existing = [HERE / "runs" / f"{design.experiment_id}__{dataset}__arm_only_summary.json" for dataset in ("heart", "wave")]
        result = load_summaries(design.experiment_id) if all(path.exists() for path in existing) else execute(design, source)
        summaries[design.experiment_id] = result
        aggregate = aggregate_score(result)
        preserved = arm_preserved(result, baseline)
        safe = bool(aggregate["all_safe"])
        by_dataset = {item["dataset"]: item for item in result}
        new_rows.append(
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
    frame = pd.read_csv(path)
    frame = pd.concat([frame, pd.DataFrame(new_rows)], ignore_index=True)
    frame = frame.drop_duplicates(subset=["experiment_id"], keep="last")
    frame.to_csv(path, index=False)

    incumbent = Design(
        "family_c_ankle_0p7",
        "C_JOINT_ALLOCATION",
        "ankle contribution",
        "1.0",
        "0.7",
        ankle_pitch_weight=0.7,
        ankle_roll_weight=0.7,
    )
    pool = [incumbent] + designs
    all_results = {incumbent.experiment_id: load_summaries(incumbent.experiment_id), **summaries}
    eligible = [
        design for design in pool
        if all(bool(item["safety_pass"]) for item in all_results[design.experiment_id])
        and arm_preserved(all_results[design.experiment_id], baseline)
    ]
    selected = min(
        eligible,
        key=lambda design: (
            balance_shape_score(all_results[design.experiment_id]),
            aggregate_score(all_results[design.experiment_id])["mean_balance_rmse_rad"],
        ),
    )
    payload = {
        "classification": "FINAL_VALIDATION_CANDIDATE",
        "warning": selected.warning,
        "selected_experiment_id": selected.experiment_id,
        "parameters": asdict(selected),
        "selection_basis": "heart+wave safety hard constraints, arm preservation, then balance shape and relative RMSE",
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
