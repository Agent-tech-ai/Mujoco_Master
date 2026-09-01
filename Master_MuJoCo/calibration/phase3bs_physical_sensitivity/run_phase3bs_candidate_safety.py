#!/usr/bin/env python3
"""Full safety checks for the sole Phase 3B-S shared sensitivity direction."""

from __future__ import annotations

import json
import math

import mujoco
import pandas as pd

from phase3bs_core import (
    AX, HERE, P3AR, RUNS, Y, PhysicalExperiment, apply_runtime_override,
    ay_candidate,
)


CANDIDATE_ID = "bs_mass_lower_plus08_safety_candidate"
EXPERIMENT = PhysicalExperiment(
    CANDIDATE_ID, "MASS_DISTRIBUTION",
    "lower_limb_mass_scale_total_mass_preserved", "plus", 1.08, 0.08,
    "Safety validation of shared local sensitivity direction; not hardware calibration",
    classification="SHARED_PHYSICAL_SENSITIVITY_DIRECTION_NOT_HARDWARE_CALIBRATION",
)


def compact(summary: dict) -> dict:
    row = AX.compact_row(summary)
    row["physical_family"] = EXPERIMENT.family
    row["runtime_only"] = True
    row["not_hardware_calibration"] = True
    return row


def run_perturbations() -> list[dict]:
    design = ay_candidate(CANDIDATE_ID)
    source = Y.datasets()
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
    old_load = P3AR.load_model
    old_runs = Y.RUNS

    def loader(*, free_base: bool):
        model = old_load(free_base=free_base)
        apply_runtime_override(model, EXPERIMENT)
        return model

    P3AR.load_model = loader
    Y.RUNS = RUNS
    results = []
    try:
        for dataset_name, perturbation in perturbations:
            print(f"PERTURB {dataset_name} {perturbation['id']}", flush=True)
            summary = Y.run_standing(design, source[dataset_name], perturbation=perturbation, save_detail=False)
            results.append(compact(summary))
            print(json.dumps({"safety_pass": summary["safety_pass"], "stable_no_fall": summary["stable_no_fall"]}), flush=True)
    finally:
        P3AR.load_model = old_load
        Y.RUNS = old_runs
    return results


def run_rehearsal() -> dict:
    design = ay_candidate(CANDIDATE_ID)
    targets = P3AR.P3A._base_targets()
    offsets = AX.standing_offsets(design)
    for name, value in offsets.items():
        if name in targets:
            targets[name] += value
    results = []
    for joint_name in P3AR.P3A.REHEARSAL_JOINTS:
        model = P3AR.load_model(free_base=False)
        audit = apply_runtime_override(model, EXPERIMENT)
        controller = Y.MotionConditionedBalanceController(model, design)
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
        row = {
            "joint_name": joint_name,
            "tracking_status": "SETTLED" if settled else "NOT_SETTLED",
            "maximum_position_error_deg": math.degrees(maximum_error),
            "return_error_deg": math.degrees(final_error),
            "self_collision_steps": max_self,
            "joint_limit_violation_steps": limit_steps,
            "persistent_saturation_fraction": saturation_steps / total,
            "runtime_total_mass_kg": audit["total_mass_after_kg"],
        }
        results.append(row)
        print(f"REHEARSAL {joint_name} {row['tracking_status']}", flush=True)
    return {
        "results": results,
        "settled_count": sum(row["tracking_status"] == "SETTLED" for row in results),
        "total": len(results),
    }


def main() -> int:
    perturb = run_perturbations()
    rehearsal = run_rehearsal()
    pd.DataFrame(perturb).to_csv(HERE / "phase3bs_shared_direction_perturbation_results.csv", index=False)
    pd.DataFrame(rehearsal["results"]).to_csv(HERE / "phase3bs_shared_direction_rehearsal_12_joint.csv", index=False)
    payload = {
        "warning": "PHYSICAL SENSITIVITY DIRECTION; NOT HARDWARE CALIBRATION",
        "robot_connected": False,
        "reported_effort_used": False,
        "source_mjcf_modified": False,
        "experiment": EXPERIMENT.__dict__,
        "perturbation_runs": perturb,
        "rehearsal": rehearsal,
        "perturbation_pass_count": sum(bool(row["safety_pass"]) for row in perturb),
        "perturbation_total": len(perturb),
    }
    (HERE / "phase3bs_shared_direction_safety_validation.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return 0 if payload["perturbation_pass_count"] == len(perturb) and rehearsal["settled_count"] == rehearsal["total"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
