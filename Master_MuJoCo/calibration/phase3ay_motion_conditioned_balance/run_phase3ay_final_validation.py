#!/usr/bin/env python3
"""Final offline safety validation for the Phase 3A-Y joint candidate."""

from __future__ import annotations

import json

import pandas as pd

from phase3ay_core import HERE, RUNS, compact_row, datasets, run_replay, run_standing
from run_phase3ay_candidate_smoke import candidate


FINAL_ID = "phase3ay_final_candidate_v3"


def load_or_run(dataset_name: str, mode: str, *, save_detail: bool = True):
    path = RUNS / f"{FINAL_ID}__{dataset_name}__{mode}_summary.json"
    if path.exists():
        return json.loads(path.read_text(encoding="utf-8"))
    return run_replay(candidate(FINAL_ID), datasets()[dataset_name], mode, save_detail=save_detail)


def main() -> int:
    source = datasets()
    final = []
    for dataset_name, mode in (
        ("heart", "arm_only"), ("wave", "arm_only"),
        ("heart", "whole_body"), ("wave", "whole_body"),
    ):
        print(f"FINAL {dataset_name} {mode}", flush=True)
        summary = load_or_run(dataset_name, mode)
        final.append(summary)
        print(json.dumps(compact_row(summary), indent=2), flush=True)

    perturbations = [
        ("heart", {"id": "roll_plus_0p25", "base_roll_deg": 0.25}),
        ("heart", {"id": "pitch_minus_0p25", "base_pitch_deg": -0.25}),
        ("wave", {"id": "roll_plus_0p25", "base_roll_deg": 0.25}),
        ("wave", {"id": "roll_minus_0p25", "base_roll_deg": -0.25}),
        ("wave", {"id": "pitch_plus_0p25", "base_pitch_deg": 0.25}),
        ("wave", {"id": "pitch_minus_0p25", "base_pitch_deg": -0.25}),
        ("wave", {"id": "left_hip_roll_plus_0p25", "joint_name": "left_hip_roll_joint", "joint_delta_deg": 0.25}),
        ("wave", {"id": "left_hip_roll_minus_0p25", "joint_name": "left_hip_roll_joint", "joint_delta_deg": -0.25}),
    ]
    perturbed = []
    for dataset_name, perturbation in perturbations:
        print(f"PERTURB {dataset_name} {perturbation['id']}", flush=True)
        stem = f"{FINAL_ID}__{dataset_name}__standing__perturb_{perturbation['id']}_summary.json"
        path = RUNS / stem
        if path.exists():
            summary = json.loads(path.read_text(encoding="utf-8"))
        else:
            summary = run_standing(candidate(FINAL_ID), source[dataset_name], perturbation=perturbation)
        perturbed.append(summary)
        print(json.dumps(compact_row(summary), indent=2), flush=True)

    payload = {
        "warning": "SIMULATION CONTROLLER CANDIDATE; NOT HARDWARE CALIBRATION",
        "robot_connected": False,
        "reported_effort_used": False,
        "mjcf_modified": False,
        "physical_parameters_modified": False,
        "final_runs": final,
        "perturbation_runs": perturbed,
    }
    (HERE / "phase3ay_final_validation.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    pd.DataFrame([compact_row(item) for item in final]).to_csv(HERE / "phase3ay_final_validation.csv", index=False)
    pd.DataFrame([compact_row(item) for item in perturbed]).to_csv(HERE / "phase3ay_perturbation_results.csv", index=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
