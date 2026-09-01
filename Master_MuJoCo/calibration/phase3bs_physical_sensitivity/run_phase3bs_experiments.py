#!/usr/bin/env python3
"""Run one-family-at-a-time Phase 3B-S sensitivity experiments."""

from __future__ import annotations

import json

import pandas as pd

from phase3bs_core import HERE, RUNS, PhysicalExperiment, run_fixed, run_free


def formal_experiments() -> list[PhysicalExperiment]:
    return [
        PhysicalExperiment("bs_baseline", "BASELINE", "none", "baseline", 1.0, 0.0, "Frozen physical/controller baseline"),
        PhysicalExperiment("bs_mass_lower_minus08", "MASS_DISTRIBUTION", "lower_limb_mass_scale_total_mass_preserved", "minus", 0.92, -0.08, "Shift mass from lower limbs to upper body"),
        PhysicalExperiment("bs_mass_lower_plus08", "MASS_DISTRIBUTION", "lower_limb_mass_scale_total_mass_preserved", "plus", 1.08, 0.08, "Shift mass from upper body to lower limbs"),
        PhysicalExperiment("bs_inertia_minus10", "ROTATIONAL_INERTIA", "selected_link_rotational_inertia_scale", "minus", 0.90, -0.10, "Reduce selected link rotational inertia only"),
        PhysicalExperiment("bs_inertia_plus10", "ROTATIONAL_INERTIA", "selected_link_rotational_inertia_scale", "plus", 1.10, 0.10, "Increase selected link rotational inertia only"),
        PhysicalExperiment("bs_damping_005", "JOINT_DAMPING", "pitch_chain_damping_Nm_s_rad", "low", 0.05, 0.5, "Add low pitch-chain viscous damping from zero baseline"),
        PhysicalExperiment("bs_damping_015", "JOINT_DAMPING", "pitch_chain_damping_Nm_s_rad", "high", 0.15, 1.5, "Add moderate pitch-chain viscous damping from zero baseline"),
        PhysicalExperiment("bs_armature_minus20", "ARMATURE", "pitch_chain_armature_scale", "minus", 0.80, -0.20, "Reduce pitch-chain numerical armature"),
        PhysicalExperiment("bs_armature_plus20", "ARMATURE", "pitch_chain_armature_scale", "plus", 1.20, 0.20, "Increase pitch-chain numerical armature"),
        PhysicalExperiment("bs_friction_minus10", "CONTACT_FRICTION", "floor_and_foot_friction_scale", "minus", 0.90, -0.10, "Reduce existing floor/foot friction coefficients"),
        PhysicalExperiment("bs_friction_plus10", "CONTACT_FRICTION", "floor_and_foot_friction_scale", "plus", 1.10, 0.10, "Increase existing floor/foot friction coefficients"),
        PhysicalExperiment("bs_compliance_stiffer10", "CONTACT_COMPLIANCE", "floor_and_foot_solref_timeconst_scale", "minus", 0.90, -0.10, "Reduce contact time constant (stiffer diagnostic direction)"),
        PhysicalExperiment("bs_compliance_softer10", "CONTACT_COMPLIANCE", "floor_and_foot_solref_timeconst_scale", "plus", 1.10, 0.10, "Increase contact time constant (softer diagnostic direction)"),
    ]


def decomposition_experiments() -> list[PhysicalExperiment]:
    return [
        PhysicalExperiment("bs_decomp_balance_disabled", "BASELINE", "balance_feedback", "disabled", 1.0, 0.0, "Free-base arm replay without balance feedback", controller_mode="BALANCE_DISABLED_DIAGNOSTIC"),
        PhysicalExperiment("bs_decomp_knee_actuator_disabled", "BASELINE", "knee_actuator_channel", "disabled", 1.0, 0.0, "Free-base arm replay with knee actuator command zeroed", controller_mode="KNEE_ACTUATOR_DISABLED_DIAGNOSTIC"),
    ]


def main() -> int:
    rows = []
    for experiment in formal_experiments():
        for dataset in ("heart", "wave"):
            path = RUNS / f"{experiment.experiment_id}__{dataset}__arm_only_summary.json"
            print(f"SENSITIVITY {experiment.experiment_id} {dataset}", flush=True)
            summary = json.loads(path.read_text(encoding="utf-8")) if path.exists() else run_free(experiment, dataset)
            rows.append({
                **experiment.__dict__, "dataset": dataset,
                "stable_no_fall": summary["stable_no_fall"], "safety_pass": summary["safety_pass"],
                "contact_safety_pass": summary["contact_safety_pass"],
                "limit_management_pass": summary["limit_management_pass"],
                "saturation_management_pass": summary["saturation_management_pass"],
            })
            print(json.dumps({key: rows[-1][key] for key in ("experiment_id", "dataset", "safety_pass")}), flush=True)

    for experiment in decomposition_experiments():
        print(f"DECOMPOSITION {experiment.experiment_id} wave", flush=True)
        summary = run_free(experiment, "wave", save_detail=True)
        rows.append({**experiment.__dict__, "dataset": "wave", "stable_no_fall": summary["stable_no_fall"], "safety_pass": summary["safety_pass"]})

    fixed = PhysicalExperiment("bs_decomp_fixed_base", "BASELINE", "base_constraint", "fixed", 1.0, 0.0, "Arm reference with fixed base", base_mode="FIXED")
    fixed_free = PhysicalExperiment("bs_decomp_fixed_contact_free", "CONTACT_FREE_DIAGNOSTIC", "floor_contact", "disabled", 1.0, 0.0, "Fixed-base contact-free diagnostic", base_mode="FIXED", contact_mode="DISABLED_DIAGNOSTIC")
    for experiment, contact_free in ((fixed, False), (fixed_free, True)):
        print(f"DECOMPOSITION {experiment.experiment_id} wave", flush=True)
        summary = run_fixed(experiment, "wave", contact_free=contact_free)
        rows.append({**experiment.__dict__, "dataset": "wave", "stable_no_fall": summary["stable_no_fall"], "safety_pass": None})

    pd.DataFrame(rows).to_csv(HERE / "phase3bs_experiments.csv", index=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

