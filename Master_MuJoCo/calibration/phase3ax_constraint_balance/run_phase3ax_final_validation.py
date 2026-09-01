#!/usr/bin/env python3
"""Final deterministic validation for the Phase 3A-X architecture candidate."""

from __future__ import annotations

import json
import math

import mujoco
import pandas as pd

from phase3ax_core import (
    AXDesign, ConstraintAwareBalanceController, HERE, P3AR, compact_row,
    datasets, run_replay, run_standing, standing_offsets,
)


def candidate() -> AXDesign:
    return AXDesign(
        "phase3ax_final_candidate", "AX-G_COMBINED",
        "Constraint-aware architecture selected by hard-safety evidence before response similarity",
        limit_aware=True, contact_aware=True, rate_aware=True,
        saturation_aware=True, split_pitch_roll=True,
        dynamic_allocation=True, tracking_gate=True,
        safe_standing_reference=True,
        left_hip_roll_standing_offset_rad=0.025,
        contact_warning_m=0.003,
        contact_hard_m=0.00075,
        contact_avoidance_cap_rad=0.07,
        contact_avoidance_gain=2.0,
    )


def run_rehearsal(design: AXDesign) -> dict[str, object]:
    targets = P3AR.P3A._base_targets()
    offsets = standing_offsets(design)
    for name, value in offsets.items():
        if name in targets:
            targets[name] += value
    results = []
    for joint_name in P3AR.P3A.REHEARSAL_JOINTS:
        model = P3AR.load_model(free_base=False)
        controller = ConstraintAwareBalanceController(model, design)
        data = mujoco.MjData(model)
        for name, value in targets.items():
            if name not in controller.by_name:
                continue
            joint = controller.by_name[name]
            command = float(max(joint.lower, min(joint.upper, value)))
            controller.target[joint.qpos_adr] = command
            controller.reference_target[joint.qpos_adr] = command
            data.qpos[joint.qpos_adr] = command
            data.qvel[joint.dof_adr] = 0.0
        mujoco.mj_forward(model, data)
        controller.set_initial_foot_positions(data)
        driven = controller.by_name[joint_name]
        center = float(controller.target[driven.qpos_adr])
        delta = math.radians(2.0)
        maximum_error = final_error = 0.0
        max_self = limit_steps = saturation_steps = total = 0
        segment_start = 0.0
        for segment in P3AR.P3A.SEGMENTS:
            segment_end = segment_start + segment.duration
            while data.time < segment_end - 1e-12:
                elapsed = data.time - segment_start
                ratio = min(max(elapsed / segment.duration, 0.0), 1.0)
                smooth = ratio**3 * (10.0 + ratio * (-15.0 + 6.0 * ratio))
                scale = segment.start_scale + (segment.end_scale - segment.start_scale) * smooth
                command = center + scale * delta
                controller.target[driven.qpos_adr] = command
                controller.reference_target[driven.qpos_adr] = command
                controller.apply(data)
                mujoco.mj_step(model, data)
                measured = float(data.qpos[driven.qpos_adr])
                maximum_error = max(maximum_error, abs(command - measured))
                final_error = measured - center
                max_self = max(max_self, len(P3AR.P3A._self_contacts(model, data)))
                limit_steps += int(measured < driven.lower - 1e-9 or measured > driven.upper + 1e-9)
                limit = max(abs(float(x)) for x in model.actuator_ctrlrange[driven.actuator_id])
                saturation_steps += int(abs(float(data.ctrl[driven.actuator_id])) >= 0.98 * limit)
                total += 1
            segment_start = segment_end
        settled = math.degrees(maximum_error) <= 1.0 and abs(math.degrees(final_error)) <= 0.1
        results.append({
            "joint_name": joint_name,
            "tracking_status": "SETTLED" if settled else "NOT_SETTLED",
            "maximum_position_error_deg": math.degrees(maximum_error),
            "return_error_deg": math.degrees(final_error),
            "self_collision_steps": max_self,
            "joint_limit_violation_steps": limit_steps,
            "persistent_saturation_fraction": saturation_steps / total,
        })
        print(f"REHEARSAL {joint_name} {results[-1]['tracking_status']}", flush=True)
    payload = {
        "warning": "SIMULATION CONTROLLER DESIGN; NOT HARDWARE CALIBRATION",
        "results": results,
        "settled_count": sum(row["tracking_status"] == "SETTLED" for row in results),
        "total": len(results),
    }
    pd.DataFrame(results).to_csv(HERE / "phase3ax_rehearsal_12_joint.csv", index=False)
    (HERE / "phase3ax_rehearsal_12_joint.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return payload


def main() -> int:
    design = candidate()
    source = datasets()
    final = []
    for dataset_name, mode in (
        ("heart", "standing"), ("wave", "standing"),
        ("heart", "arm_only"), ("wave", "arm_only"),
        ("wave", "whole_body"),
    ):
        print(f"FINAL {dataset_name} {mode}", flush=True)
        result = run_standing(design, source[dataset_name], save_detail=True) if mode == "standing" else run_replay(design, source[dataset_name], mode, save_detail=True)
        final.append(result)
        print(json.dumps(compact_row(result), indent=2), flush=True)

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
        result = run_standing(design, source[dataset_name], perturbation=perturbation)
        perturbed.append(result)
        print(json.dumps(compact_row(result), indent=2), flush=True)

    rehearsal = run_rehearsal(design)
    payload = {"design": design.__dict__, "final_runs": final, "perturbation_runs": perturbed, "rehearsal": rehearsal}
    (HERE / "phase3ax_final_validation.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    pd.DataFrame([compact_row(item) for item in final]).to_csv(HERE / "phase3ax_final_validation.csv", index=False)
    pd.DataFrame([compact_row(item) for item in perturbed]).to_csv(HERE / "phase3ax_perturbation_results.csv", index=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
