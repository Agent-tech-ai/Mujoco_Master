#!/usr/bin/env python3
"""Run the rest-safe joint Phase 3A-Y candidate on both measured gestures."""

from __future__ import annotations

import json

from phase3ay_core import AYDesign, compact_row, datasets, run_replay


def candidate(experiment_id: str = "phase3ay_joint_candidate") -> AYDesign:
    return AYDesign(
        experiment_id=experiment_id,
        family="AY_MOTION_CONDITIONED_RESPONSE",
        hypothesis="Continuous activity/asymmetry schedule with exact frozen fallback at rest",
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
        pitch_symmetric=(1.00, 0.05, 0.05, 0.00),
        pitch_asymmetric=(0.70, 0.10, 0.15, 0.00),
        roll_symmetric=(0.20, 0.00, 0.80),
        roll_asymmetric=(0.70, 0.00, 0.00),
        pitch_total_scale_symmetric=1.25,
        pitch_total_scale_asymmetric=1.00,
        roll_total_scale_symmetric=3.00,
        roll_total_scale_asymmetric=1.00,
    )


def main() -> int:
    source = datasets()
    for name in ("heart", "wave"):
        summary = run_replay(candidate(), source[name], "arm_only", save_detail=True)
        print(json.dumps(compact_row(summary), indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

