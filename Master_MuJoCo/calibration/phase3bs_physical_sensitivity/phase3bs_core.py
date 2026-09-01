#!/usr/bin/env python3
"""Runtime-only physical sensitivity utilities for Phase 3B-S.

No source MJCF is edited.  No robot interface, reported effort, absolute IMU,
gear, actuator force/ctrl limit, or hardware mapping is used or changed.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import json
import math
from pathlib import Path
import sys
from typing import Callable

import mujoco
import numpy as np


HERE = Path(__file__).resolve().parent
CALIBRATION = HERE.parent
P3AY_DIR = CALIBRATION / "phase3ay_motion_conditioned_balance"
RUNS = HERE / "runs"
if str(P3AY_DIR) not in sys.path:
    sys.path.insert(0, str(P3AY_DIR))
import phase3ay_core as Y  # noqa: E402
from run_phase3ay_candidate_smoke import candidate as ay_candidate  # noqa: E402

AX = Y.AX
P3AR = AX.P3AR
P3A = P3AR.P3A


PITCH_JOINTS = (
    "left_hip_pitch_joint", "right_hip_pitch_joint",
    "left_knee_joint", "right_knee_joint",
    "left_ankle_pitch_joint", "right_ankle_pitch_joint",
    "waist_pitch_joint",
)
FOOT_BODIES = {"left_ankle_roll_link", "right_ankle_roll_link"}


@dataclass(frozen=True)
class PhysicalExperiment:
    experiment_id: str
    family: str
    parameter: str
    level: str
    parameter_value: float
    normalized_parameter_change: float
    hypothesis: str
    classification: str = "PHYSICAL_SENSITIVITY_EXPERIMENT_NOT_HARDWARE_CALIBRATION"
    controller_mode: str = "FROZEN_PHASE3AY"
    base_mode: str = "FREE"
    contact_mode: str = "RETAINED"


def _name(model: mujoco.MjModel, kind, index: int) -> str:
    return mujoco.mj_id2name(model, kind, index) or ""


def body_groups(model: mujoco.MjModel) -> dict[str, list[int]]:
    groups = {key: [] for key in ("pelvis_torso", "thigh", "shank", "foot", "arms", "lower_limb", "rotational_links")}
    for index in range(1, model.nbody):
        name = _name(model, mujoco.mjtObj.mjOBJ_BODY, index)
        if name in ("pelvis", "torso_link", "waist_yaw_link", "waist_pitch_link"):
            groups["pelvis_torso"].append(index)
        if "hip_" in name:
            groups["thigh"].append(index)
        if "knee_link" in name:
            groups["shank"].append(index)
        if "ankle_" in name:
            groups["foot"].append(index)
        if any(token in name for token in ("shoulder", "elbow", "wrist")):
            groups["arms"].append(index)
    groups["lower_limb"] = sorted(set(groups["thigh"] + groups["shank"] + groups["foot"]))
    groups["rotational_links"] = sorted(set(groups["lower_limb"] + groups["pelvis_torso"] + groups["arms"]))
    return groups


def contact_geom_ids(model: mujoco.MjModel) -> list[int]:
    result = []
    for index in range(model.ngeom):
        geom = _name(model, mujoco.mjtObj.mjOBJ_GEOM, index)
        body = _name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[index]))
        if geom == "floor" or body in FOOT_BODIES:
            if geom == "floor" or int(model.geom_contype[index]) != 0:
                result.append(index)
    return result


def dof_for_joint(model: mujoco.MjModel, name: str) -> int:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise KeyError(name)
    return int(model.jnt_dofadr[joint_id])


def apply_runtime_override(model: mujoco.MjModel, experiment: PhysicalExperiment) -> dict[str, object]:
    """Apply exactly one physical family to an in-memory derived model."""
    before = {
        "total_mass_kg": float(model.body_mass.sum()),
        "actuator_gear": model.actuator_gear.copy(),
        "actuator_ctrlrange": model.actuator_ctrlrange.copy(),
        "actuator_forcerange": model.actuator_forcerange.copy(),
        "geom_type": model.geom_type.copy(),
        "geom_bodyid": model.geom_bodyid.copy(),
    }
    groups = body_groups(model)
    changed: list[str] = []
    family = experiment.family
    value = experiment.parameter_value
    if family == "BASELINE":
        pass
    elif family == "MASS_DISTRIBUTION":
        lower = groups["lower_limb"]
        upper = [index for index in range(1, model.nbody) if index not in lower]
        lower_before = float(model.body_mass[lower].sum())
        upper_before = float(model.body_mass[upper].sum())
        model.body_mass[lower] *= value
        transferred = float(model.body_mass[lower].sum() - lower_before)
        model.body_mass[upper] *= (upper_before - transferred) / upper_before
        changed.extend(_name(model, mujoco.mjtObj.mjOBJ_BODY, index) for index in lower + upper)
    elif family == "ROTATIONAL_INERTIA":
        ids = groups["rotational_links"]
        model.body_inertia[ids, :] *= value
        changed.extend(_name(model, mujoco.mjtObj.mjOBJ_BODY, index) for index in ids)
    elif family == "JOINT_DAMPING":
        for name in PITCH_JOINTS:
            model.dof_damping[dof_for_joint(model, name)] = value
            changed.append(name)
    elif family == "ARMATURE":
        for name in PITCH_JOINTS:
            model.dof_armature[dof_for_joint(model, name)] *= value
            changed.append(name)
    elif family == "CONTACT_FRICTION":
        for geom_id in contact_geom_ids(model):
            model.geom_friction[geom_id, :] *= value
            changed.append(f"geom:{geom_id}")
    elif family == "CONTACT_COMPLIANCE":
        for geom_id in contact_geom_ids(model):
            model.geom_solref[geom_id, 0] *= value
            changed.append(f"geom:{geom_id}")
    elif family == "CONTACT_FREE_DIAGNOSTIC":
        floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        model.geom_contype[floor] = 0
        model.geom_conaffinity[floor] = 0
        changed.append("floor collision disabled in fixed-base diagnostic")
    else:
        raise ValueError(f"Unknown physical family: {family}")

    if not np.array_equal(before["actuator_gear"], model.actuator_gear):
        raise RuntimeError("Forbidden actuator gear mutation")
    if not np.array_equal(before["actuator_ctrlrange"], model.actuator_ctrlrange):
        raise RuntimeError("Forbidden ctrlrange mutation")
    if not np.array_equal(before["actuator_forcerange"], model.actuator_forcerange):
        raise RuntimeError("Forbidden forcerange mutation")
    if not np.array_equal(before["geom_type"], model.geom_type) or not np.array_equal(before["geom_bodyid"], model.geom_bodyid):
        raise RuntimeError("Forbidden collision topology mutation")
    total_after = float(model.body_mass.sum())
    if family == "MASS_DISTRIBUTION" and abs(total_after - before["total_mass_kg"]) > 1e-10:
        raise RuntimeError("Mass distribution experiment failed to preserve total mass")
    return {
        "family": family,
        "parameter": experiment.parameter,
        "level": experiment.level,
        "parameter_value": value,
        "normalized_parameter_change": experiment.normalized_parameter_change,
        "changed_entities": changed,
        "total_mass_before_kg": before["total_mass_kg"],
        "total_mass_after_kg": total_after,
        "gear_unchanged": True,
        "ctrlrange_unchanged": True,
        "forcerange_unchanged": True,
        "collision_topology_unchanged": family != "CONTACT_FREE_DIAGNOSTIC",
        "source_mjcf_modified": False,
    }


def _diagnostic_controller(mode: str):
    if mode == "FROZEN_PHASE3AY":
        return Y.MotionConditionedBalanceController

    class DiagnosticController(Y.MotionConditionedBalanceController):
        def apply(self, data) -> None:
            super().apply(data)
            if mode == "KNEE_ACTUATOR_DISABLED_DIAGNOSTIC":
                for name in ("left_knee_joint", "right_knee_joint"):
                    joint = self.by_name[name]
                    data.ctrl[joint.actuator_id] = 0.0
                    self.last_decomposition[name]["final_balance_addition_nm"] = 0.0
            elif mode == "BALANCE_DISABLED_DIAGNOSTIC":
                # Balance gains are zeroed in run_free; no post-action needed.
                pass
            else:
                raise ValueError(mode)

    return DiagnosticController


def run_free(
    experiment: PhysicalExperiment,
    dataset_name: str,
    *,
    save_detail: bool = True,
    pre_s: float = 5.0,
    post_s: float = 5.0,
) -> dict[str, object]:
    RUNS.mkdir(parents=True, exist_ok=True)
    design = ay_candidate(experiment.experiment_id)
    if experiment.controller_mode == "BALANCE_DISABLED_DIAGNOSTIC":
        design = replace(design, pitch_kp=0.0, pitch_kd=0.0, roll_kp=0.0, roll_kd=0.0)
    original_load = AX.P3AR.load_model
    original_runs = Y.RUNS
    original_controller = Y.MotionConditionedBalanceController
    override_audit: dict[str, object] = {}

    def loader(*, free_base: bool):
        model = original_load(free_base=free_base)
        override_audit.update(apply_runtime_override(model, experiment))
        return model

    AX.P3AR.load_model = loader
    Y.RUNS = RUNS
    Y.MotionConditionedBalanceController = _diagnostic_controller(experiment.controller_mode)
    try:
        summary = Y.run_replay(
            design,
            Y.datasets()[dataset_name],
            "arm_only",
            pre_s=pre_s,
            post_s=post_s,
            save_detail=save_detail,
        )
    finally:
        AX.P3AR.load_model = original_load
        Y.RUNS = original_runs
        Y.MotionConditionedBalanceController = original_controller
    summary["physical_experiment"] = asdict(experiment)
    summary["runtime_override_audit"] = override_audit
    summary["physical_parameters_modified"] = experiment.family != "BASELINE"
    summary["physical_override_scope"] = "DERIVED_RUNTIME_MODEL_ONLY"
    summary["source_mjcf_modified"] = False
    summary["reported_effort_loaded"] = False
    path = RUNS / f"{experiment.experiment_id}__{dataset_name}__arm_only_summary.json"
    path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def run_fixed(experiment: PhysicalExperiment, dataset_name: str, *, contact_free: bool = False) -> dict[str, object]:
    """Fixed-base diagnostic using the frozen shoulder/wrist tracking scales."""
    RUNS.mkdir(parents=True, exist_ok=True)
    source_frame, real = P3AR.load_frames(Y.datasets()[dataset_name])
    module = P3A
    original_here = module.HERE
    original_load = module.load_model
    override_audit: dict[str, object] = {}

    def loader(*, free_base: bool):
        model = original_load(free_base=free_base)
        target = replace(experiment, family="CONTACT_FREE_DIAGNOSTIC") if contact_free else experiment
        override_audit.update(apply_runtime_override(model, target))
        return model

    module.HERE = RUNS
    module.load_model = loader
    frozen = module.Experiment(
        experiment.experiment_id,
        "phase3ay_final_candidate_v3",
        False,
        "passive-coupling diagnostic",
        "DIAGNOSTIC_ONLY",
        shoulder_gain_scale=8.0,
        wrist_gain_scale=8.0,
        standing_reference_scale=1.0,
    )
    controlled = P3AR.active_joints(source_frame, "arm_only")
    offsets = AX.standing_offsets(ay_candidate(experiment.experiment_id))
    try:
        summary = module.run_replay(frozen, source_frame, real, controlled, inherited_offsets=offsets)
    finally:
        module.HERE = original_here
        module.load_model = original_load
    summary["physical_experiment"] = asdict(experiment)
    summary["runtime_override_audit"] = override_audit
    summary["physical_override_scope"] = "DERIVED_RUNTIME_MODEL_ONLY"
    summary["source_mjcf_modified"] = False
    summary["reported_effort_used_for_fitting"] = False
    path = RUNS / f"{experiment.experiment_id}_summary.json"
    path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary
