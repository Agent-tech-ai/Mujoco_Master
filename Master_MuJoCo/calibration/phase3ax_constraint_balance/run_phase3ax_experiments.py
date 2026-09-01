#!/usr/bin/env python3
"""Run controlled single-architecture Phase 3A-X experiments."""

from __future__ import annotations

from dataclasses import asdict
import json

import pandas as pd

from phase3ax_core import AXDesign, HERE, compact_row, datasets, run_replay, run_standing


def experiment_matrix():
    legacy = AXDesign(
        "ax_legacy_additive", "LEGACY_BALANCE_ARCHITECTURE",
        "Frozen Phase 3A-R torque-additive baseline; no constraint awareness",
    )
    a = AXDesign(
        "ax_a_limit", "AX-A_JOINT_LIMIT_AWARE",
        "Break tracking->limit by directional mechanical-margin scaling",
        limit_aware=True,
    )
    b = AXDesign(
        "ax_b_contact", "AX-B_CONTACT_MARGIN_AWARE",
        "Break posture->contact by pre-contact geom-distance scaling and gradient retreat",
        contact_aware=True, contact_warning_m=0.003, contact_avoidance_cap_rad=0.07,
        contact_avoidance_gain=2.0,
    )
    c = AXDesign(
        "ax_c_rate", "AX-C_CHANNEL_RATE_LIMITED",
        "Break disturbance->excursion with independent ankle/hip/knee/waist slew limits",
        rate_aware=True,
    )
    d = AXDesign(
        "ax_d_saturation", "AX-D_SATURATION_AWARE",
        "Break saturation->continued correction using available actuator authority",
        saturation_aware=True,
    )
    e = AXDesign(
        "ax_e_pitch_roll", "AX-E_PITCH_ROLL_DECOMPOSED",
        "Prevent cross-plane noise from consuming correction authority",
        split_pitch_roll=True,
    )
    f = AXDesign(
        "ax_f_constraint_allocation", "AX-F_CONSTRAINT_AWARE_ALLOCATION",
        "Redistribute pitch/roll demand away from contact, limit, and saturation constraints",
        limit_aware=True, contact_aware=True, saturation_aware=True,
        split_pitch_roll=True, dynamic_allocation=True,
        contact_warning_m=0.003, contact_avoidance_cap_rad=0.07,
        contact_avoidance_gain=2.0,
    )
    return {
        "legacy": (legacy, (("heart", "arm_only"), ("wave", "arm_only"), ("wave", "whole_body"))),
        "A": (a, (("wave", "whole_body"),)),
        "B": (b, (("wave", "standing"), ("wave", "arm_only"))),
        "C": (c, (("wave", "whole_body"),)),
        "D": (d, (("wave", "whole_body"),)),
        "E": (e, (("heart", "arm_only"), ("wave", "arm_only"))),
        "F": (f, (("heart", "arm_only"), ("wave", "arm_only"), ("wave", "whole_body"))),
    }


def main() -> int:
    source = datasets()
    rows: list[dict[str, object]] = []
    records: list[dict[str, object]] = []
    for label, (design, scenarios) in experiment_matrix().items():
        for dataset_name, mode in scenarios:
            print(f"RUN {label} {dataset_name} {mode}", flush=True)
            if mode == "standing":
                summary = run_standing(design, source[dataset_name])
            else:
                summary = run_replay(design, source[dataset_name], mode)
            row = compact_row(summary)
            row.update({
                "architecture_hypothesis": design.hypothesis,
                "changed_flags": ",".join(name for name in (
                    "limit_aware", "contact_aware", "rate_aware", "saturation_aware",
                    "split_pitch_roll", "dynamic_allocation", "tracking_gate",
                ) if getattr(design, name)),
                "classification": "SINGLE_FACTOR_EVIDENCE",
            })
            rows.append(row)
            records.append(summary)
            print(json.dumps(row, indent=2), flush=True)
    pd.DataFrame(rows).to_csv(HERE / "phase3ax_single_factor_experiments.csv", index=False)
    (HERE / "phase3ax_single_factor_summaries.json").write_text(json.dumps(records, indent=2), encoding="utf-8")
    (HERE / "phase3ax_family_definitions.json").write_text(
        json.dumps({label: asdict(value[0]) for label, value in experiment_matrix().items()}, indent=2),
        encoding="utf-8",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
