#!/usr/bin/env python3
"""Run bounded Phase 3A-Y response-architecture experiments offline."""

from __future__ import annotations

import json
from dataclasses import replace

import pandas as pd

from phase3ay_core import AYDesign, HERE, RUNS, compact_row, datasets, run_replay


def baseline(experiment_id: str, hypothesis: str) -> AYDesign:
    return AYDesign(
        experiment_id=experiment_id,
        family="AY_MOTION_CONDITIONED_RESPONSE",
        hypothesis=hypothesis,
        limit_aware=True,
        contact_aware=True,
        rate_aware=True,
        saturation_aware=True,
        split_pitch_roll=True,
        dynamic_allocation=True,
        tracking_gate=True,
        safe_standing_reference=True,
        left_hip_roll_standing_offset_rad=0.025,
        contact_warning_m=0.003,
        contact_hard_m=0.00075,
        contact_avoidance_cap_rad=0.07,
        contact_avoidance_gain=2.0,
    )


def designs() -> list[AYDesign]:
    base = baseline("ay_fixed_ax", "Frozen 3A-X fixed response expressed through the new interface")
    return [
        base,
        replace(
            base,
            experiment_id="ay_heart_structure",
            hypothesis="Heart-only structure: ankle-dominant pitch and explicit waist-roll response",
            pitch_symmetric=(1.00, 0.03, -0.08, 0.03),
            pitch_asymmetric=(1.00, 0.03, -0.08, 0.03),
            roll_symmetric=(0.46, 0.02, 0.46),
            roll_asymmetric=(0.46, 0.02, 0.46),
        ),
        replace(
            base,
            experiment_id="ay_wave_structure",
            hypothesis="Wave-only structure: suppress passive knee response with signed bounded knee share",
            pitch_symmetric=(0.62, -0.05, -0.33, 0.10),
            pitch_asymmetric=(0.62, -0.05, -0.33, 0.10),
            roll_symmetric=(0.55, 0.05, 0.25),
            roll_asymmetric=(0.55, 0.05, 0.25),
        ),
        replace(
            base,
            experiment_id="ay_low_wave_pitch",
            hypothesis="Nonnegative wave-only diagnostic: reduce total pitch request without changing distribution",
            pitch_total_scale_symmetric=0.25,
            pitch_total_scale_asymmetric=0.25,
        ),
        replace(
            base,
            experiment_id="ay_knee_support",
            hypothesis="Nonnegative diagnostic: test whether increased same-sign knee authority restrains passive knee coupling",
            pitch_symmetric=(0.70, 0.10, 0.50, 0.00),
            pitch_asymmetric=(0.70, 0.10, 0.50, 0.00),
        ),
        replace(
            base,
            experiment_id="ay_joint_schedule_v4",
            hypothesis="Conservative continuous schedule: retain stable distribution while modestly reducing asymmetric pitch authority",
            pitch_symmetric=(0.70, 0.10, 0.15, 0.00),
            pitch_asymmetric=(0.70, 0.10, 0.15, 0.00),
            roll_symmetric=(0.35, 0.00, 0.70),
            roll_asymmetric=(0.60, 0.00, 0.20),
            pitch_total_scale_symmetric=1.05,
            pitch_total_scale_asymmetric=0.80,
            roll_total_scale_symmetric=2.0,
            roll_total_scale_asymmetric=1.0,
        ),
        replace(
            base,
            experiment_id="ay_joint_schedule_v5",
            hypothesis="Ankle-dominant symmetric response, conservative asymmetric response, and explicit waist-roll authority",
            pitch_symmetric=(1.00, 0.05, 0.05, 0.00),
            pitch_asymmetric=(0.70, 0.10, 0.15, 0.00),
            roll_symmetric=(0.20, 0.00, 0.80),
            roll_asymmetric=(0.60, 0.00, 0.20),
            pitch_total_scale_symmetric=1.25,
            pitch_total_scale_asymmetric=0.82,
            roll_total_scale_symmetric=3.0,
            roll_total_scale_asymmetric=1.0,
        ),
    ]


def main() -> int:
    source = datasets()
    rows = []
    payload = []
    for design in designs():
        for dataset_name in ("heart", "wave"):
            print(f"EXPERIMENT {design.experiment_id} {dataset_name}", flush=True)
            summary_path = RUNS / f"{design.experiment_id}__{dataset_name}__arm_only_summary.json"
            if summary_path.exists():
                summary = json.loads(summary_path.read_text(encoding="utf-8"))
                print("REUSED EXISTING DETERMINISTIC RUN", flush=True)
            else:
                summary = run_replay(design, source[dataset_name], "arm_only", save_detail=True)
            payload.append(summary)
            row = compact_row(summary)
            row["hypothesis"] = design.hypothesis
            rows.append(row)
            print(json.dumps(row, indent=2), flush=True)
    pd.DataFrame(rows).to_csv(HERE / "phase3ay_experiments.csv", index=False)
    (HERE / "phase3ay_experiments.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
