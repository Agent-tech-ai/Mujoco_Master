#!/usr/bin/env python3
"""Run full-window validation for the selected Phase 3A-R candidate."""

from __future__ import annotations

import json

from phase3ar_core import Design, HERE, P3A, datasets, run_replay, run_standing, standing_offsets


def main() -> int:
    candidate = json.loads((HERE / "simulation_controller_robustness_candidate.json").read_text(encoding="utf-8"))
    design = Design(**candidate["parameters"])
    design = Design(**{**candidate["parameters"], "experiment_id": "phase3ar_final_candidate"})
    source = datasets()
    summaries = []
    for label, callback in (
        ("heart standing 10 s", lambda: run_standing(design, source["heart"], save_detail=True)),
        ("wave standing 10 s", lambda: run_standing(design, source["wave"], save_detail=True)),
        ("heart arm-only full", lambda: run_replay(design, source["heart"], "arm_only", save_detail=True)),
        ("wave arm-only full", lambda: run_replay(design, source["wave"], "arm_only", save_detail=True)),
        ("wave whole-body full", lambda: run_replay(design, source["wave"], "whole_body", save_detail=True)),
    ):
        print(f"RUN final {label}", flush=True)
        summaries.append(callback())

    P3A.HERE = HERE
    rehearsal_experiment = P3A.Experiment(
        name="phase3ar_final_rehearsal",
        parent="phase3a_final_candidate",
        free_base=False,
        changed_category="simulation controller robustness validation",
        classification="VALIDATION_ONLY",
        shoulder_gain_scale=design.shoulder_gain_scale,
        wrist_gain_scale=design.wrist_gain_scale,
        balance_gain_scale=0.7,
        standing_reference_scale=1.0,
    )
    rehearsal = P3A.run_rehearsal_regression(rehearsal_experiment, standing_offsets(design))
    payload = {
        "candidate": candidate,
        "full_validation_runs": summaries,
        "rehearsal": rehearsal,
        "reported_effort_used_for_fitting": False,
        "physical_parameters_modified": False,
        "mjcf_modified": False,
        "hardware_mapping_modified": False,
    }
    (HERE / "phase3ar_final_validation_summary.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps({"runs": len(summaries), "rehearsal": f"{rehearsal['settled_count']}/{rehearsal['total']}"}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
