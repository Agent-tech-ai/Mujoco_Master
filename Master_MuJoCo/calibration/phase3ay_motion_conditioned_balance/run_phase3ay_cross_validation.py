#!/usr/bin/env python3
"""Strict leave-one-motion-out response-architecture experiments."""

from __future__ import annotations

from dataclasses import replace
import json

import pandas as pd

from phase3ay_core import HERE, RUNS, compact_row, datasets, run_replay
from run_phase3ay_candidate_smoke import candidate


def designs():
    joint = candidate("phase3ay_cv_joint_design")
    heart_only = replace(
        joint,
        experiment_id="phase3ay_cv_heart_only",
        hypothesis="Fit symmetric heart response only; blind-test same structure on unilateral wave",
        pitch_asymmetric=joint.pitch_symmetric,
        roll_asymmetric=joint.roll_symmetric,
        pitch_total_scale_asymmetric=joint.pitch_total_scale_symmetric,
        roll_total_scale_asymmetric=joint.roll_total_scale_symmetric,
    )
    wave_only = replace(
        joint,
        experiment_id="phase3ay_cv_wave_only",
        hypothesis="Fit wave with safest observed nonnegative distribution; blind-test heart",
        pitch_symmetric=(0.70, 0.10, 0.15, 0.00),
        pitch_asymmetric=(0.70, 0.10, 0.15, 0.00),
        roll_symmetric=(0.70, 0.00, 0.00),
        roll_asymmetric=(0.70, 0.00, 0.00),
        pitch_total_scale_symmetric=1.0,
        pitch_total_scale_asymmetric=1.0,
        roll_total_scale_symmetric=1.0,
        roll_total_scale_asymmetric=1.0,
    )
    return heart_only, wave_only


def main() -> int:
    source = datasets()
    rows = []
    payload = []
    for design in designs():
        for name in ("heart", "wave"):
            print(f"CROSS_VALIDATION {design.experiment_id} {name}", flush=True)
            path = RUNS / f"{design.experiment_id}__{name}__arm_only_summary.json"
            summary = json.loads(path.read_text(encoding="utf-8")) if path.exists() else run_replay(
                design, source[name], "arm_only", save_detail=False
            )
            payload.append(summary)
            row = compact_row(summary)
            row["fit_dataset"] = "heart" if "heart_only" in design.experiment_id else "wave"
            row["evaluation"] = "FIT" if row["dataset"] == row["fit_dataset"] else "BLIND"
            row["hypothesis"] = design.hypothesis
            rows.append(row)
            print(json.dumps(row, indent=2), flush=True)
    pd.DataFrame(rows).to_csv(HERE / "phase3ay_cross_validation.csv", index=False)
    (HERE / "phase3ay_cross_validation.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

