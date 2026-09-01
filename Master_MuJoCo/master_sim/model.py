"""Model paths, documented limits, and structural validation."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ASSET_DIR = PROJECT_ROOT / "assets" / "Master"
FIXED_SCENE = ASSET_DIR / "scene_x2_fixed.xml"
FREE_SCENE = ASSET_DIR / "scene_x2_free.xml"


@dataclass(frozen=True)
class JointLimit:
    """A joint limit expressed in the MJCF joint coordinate, in degrees."""

    minimum: float
    maximum: float


# The official table describes the right-side mechanical convention. Some axes in
# the supplied MJCF are sign-reversed or mirrored on the left side. These values
# preserve the documented physical envelope in each actual MJCF coordinate.
EXPECTED_LIMITS_DEG: dict[str, JointLimit] = {
    "left_hip_pitch_joint": JointLimit(-146.5, 146.5),
    "left_hip_roll_joint": JointLimit(-13.5, 166.5),
    "left_hip_yaw_joint": JointLimit(-96.5, 196.5),
    "left_knee_joint": JointLimit(0.0, 138.0),
    "left_ankle_pitch_joint": JointLimit(-46.0, 26.0),
    "left_ankle_roll_joint": JointLimit(-15.0, 15.0),
    "right_hip_pitch_joint": JointLimit(-146.5, 146.5),
    "right_hip_roll_joint": JointLimit(-166.5, 13.5),
    "right_hip_yaw_joint": JointLimit(-196.5, 96.5),
    "right_knee_joint": JointLimit(0.0, 138.0),
    "right_ankle_pitch_joint": JointLimit(-46.0, 26.0),
    "right_ankle_roll_joint": JointLimit(-15.0, 15.0),
    "waist_yaw_joint": JointLimit(-196.5, 126.5),
    "waist_pitch_joint": JointLimit(-18.0, 18.0),
    "waist_roll_joint": JointLimit(-28.0, 28.0),
    "left_shoulder_pitch_joint": JointLimit(-176.5, 116.5),
    "left_shoulder_roll_joint": JointLimit(-3.5, 174.5),
    "left_shoulder_yaw_joint": JointLimit(-146.5, 146.5),
    "left_elbow_joint": JointLimit(-135.0, 0.0),
    "left_wrist_yaw_joint": JointLimit(-146.5, 146.5),
    "left_wrist_pitch_joint": JointLimit(-33.0, 33.0),
    "left_wrist_roll_joint": JointLimit(-86.5, 41.5),
    "right_shoulder_pitch_joint": JointLimit(-176.5, 116.5),
    "right_shoulder_roll_joint": JointLimit(-174.5, 3.5),
    "right_shoulder_yaw_joint": JointLimit(-146.5, 146.5),
    "right_elbow_joint": JointLimit(-135.0, 0.0),
    "right_wrist_yaw_joint": JointLimit(-146.5, 146.5),
    "right_wrist_pitch_joint": JointLimit(-33.0, 33.0),
    "right_wrist_roll_joint": JointLimit(-41.5, 86.5),
    "head_yaw_joint": JointLimit(-20.0, 20.0),
}


def object_name(model: mujoco.MjModel, obj_type: mujoco.mjtObj, obj_id: int) -> str:
    name = mujoco.mj_id2name(model, obj_type, obj_id)
    if name is None:
        raise ValueError(f"Unnamed {obj_type!s} object at id {obj_id}")
    return name


def load_model(*, free_base: bool = False) -> mujoco.MjModel:
    """Compile and return the fixed- or free-base documented-limit model."""

    path = FREE_SCENE if free_base else FIXED_SCENE
    return mujoco.MjModel.from_xml_path(str(path))


def actuated_joint_names(model: mujoco.MjModel) -> list[str]:
    """Return joints driven by the model's direct-drive motor actuators."""

    names: list[str] = []
    for actuator_id in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        if joint_id < 0:
            raise ValueError(f"Actuator {actuator_id} is not attached to a joint")
        names.append(object_name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id))
    return names


def joint_limit_degrees(model: mujoco.MjModel, name: str) -> tuple[float, float]:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise KeyError(name)
    return tuple(np.rad2deg(model.jnt_range[joint_id]).tolist())


def validate_model(model: mujoco.MjModel) -> list[str]:
    """Return human-readable validation errors; an empty list means valid."""

    errors: list[str] = []
    names = actuated_joint_names(model)
    expected = set(EXPECTED_LIMITS_DEG)

    if model.nu != 30:
        errors.append(f"Expected 30 actuators, got {model.nu}")
    if len(names) != len(set(names)):
        errors.append("Multiple actuators target the same joint")
    if set(names) != expected:
        missing = sorted(expected - set(names))
        extra = sorted(set(names) - expected)
        errors.append(f"Actuated joint mismatch: missing={missing}, extra={extra}")

    head_pitch_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "head_pitch_joint"
    )
    if head_pitch_id >= 0:
        errors.append("head_pitch_joint must be fixed because the official limit is 0°")

    for name, expected_limit in EXPECTED_LIMITS_DEG.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            continue
        actual = np.rad2deg(model.jnt_range[joint_id])
        wanted = np.array([expected_limit.minimum, expected_limit.maximum])
        if not np.allclose(actual, wanted, atol=1e-3):
            errors.append(
                f"{name}: got {actual[0]:.4f}..{actual[1]:.4f}°, "
                f"expected {wanted[0]:.4f}..{wanted[1]:.4f}°"
            )
        if not bool(model.jnt_limited[joint_id]):
            errors.append(f"{name}: joint limit is not enabled")

    if model.nmesh < 35:
        errors.append(f"Expected the detailed mesh model, got only {model.nmesh} meshes")
    return errors


def validation_summary(model: mujoco.MjModel) -> str:
    errors = validate_model(model)
    if errors:
        return "MODEL INVALID\n- " + "\n- ".join(errors)
    base_mode = "free" if model.neq == 0 else "fixed"
    return (
        "MODEL OK\n"
        f"- base: {base_mode}\n"
        f"- nq/nv/nu: {model.nq}/{model.nv}/{model.nu}\n"
        f"- bodies/geometries/meshes: {model.nbody}/{model.ngeom}/{model.nmesh}\n"
        "- documented actuated joints: 30 (head pitch fixed at 0°)"
    )
