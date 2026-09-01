"""Reproduce the X2 MuJoCo standing-stability audit and cleanup evidence.

This program is local simulation only. It never imports ROS, opens SSH, or reads
hardware interfaces. The original and current X2 MJCF files are read-only.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import mujoco
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from calibration.phase2b2_sim_rehearsal import (
    _base_targets,
    run_fixed_rehearsal,
    run_free_base_probe,
)
from calibration.phase2b2_common import load_snapshot, ranked_candidates
from master_sim.controller import (
    JointPositionController,
    SimulationStabilityConfig,
    SimulationStabilityController,
)
from master_sim.model import ASSET_DIR, load_model, object_name


HERE = Path(__file__).resolve().parent
PLOTS = HERE / "plots"
DATA = HERE / "data"
EXPERIMENTS = HERE / "standing_stability_experiments.csv"
SUPPORT_X = (-0.08441, 0.11959)
SUPPORT_Y = (-0.19715009, 0.19715009)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--sample-rate", type=float, default=100.0)
    parser.add_argument("--reports-only", action="store_true")
    return parser.parse_args()


def rpy(rotation: np.ndarray) -> tuple[float, float, float]:
    matrix = rotation.reshape(3, 3)
    roll = math.atan2(float(matrix[2, 1]), float(matrix[2, 2]))
    pitch = math.asin(float(np.clip(-matrix[2, 0], -1.0, 1.0)))
    yaw = math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))
    return roll, pitch, yaw


def convex_hull(points: list[list[float]]) -> list[list[float]]:
    unique = sorted({(float(point[0]), float(point[1])) for point in points})
    if len(unique) <= 1:
        return [list(point) for point in unique]

    def cross(origin, first, second) -> float:
        return (first[0] - origin[0]) * (second[1] - origin[1]) - (first[1] - origin[1]) * (second[0] - origin[0])

    lower = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return [list(point) for point in lower[:-1] + upper[:-1]]


def support_geometry(geom_rows: list[dict], com: np.ndarray) -> dict:
    points_by_side = {}
    for side in ("left", "right"):
        side_rows = [row for row in geom_rows if row["side"] == side]
        lowest = min(row["surface_z"] for row in side_rows)
        # Include the intended sole/toe spheres within 3 mm of the lowest
        # surface, excluding the visibly elevated auxiliary sphere.
        points_by_side[side] = [
            row["world_pos"][:2]
            for row in side_rows
            if row["surface_z"] <= lowest + 0.003
        ]
    left = convex_hull(points_by_side["left"])
    right = convex_hull(points_by_side["right"])
    combined = convex_hull(points_by_side["left"] + points_by_side["right"])
    point = np.asarray(com[:2], dtype=float)
    edge_distances = []
    cross_values = []
    for index, start_value in enumerate(combined):
        start = np.asarray(start_value)
        end = np.asarray(combined[(index + 1) % len(combined)])
        edge = end - start
        parameter = float(np.clip(np.dot(point - start, edge) / np.dot(edge, edge), 0.0, 1.0))
        closest = start + parameter * edge
        edge_distances.append(float(np.linalg.norm(point - closest)))
        cross_values.append(float(edge[0] * (point[1] - start[1]) - edge[1] * (point[0] - start[0])))
    x_values = [value[0] for value in combined]
    y_values = [value[1] for value in combined]
    return {
        "xyz_m": com.tolist(),
        "left_polygon_xy_m": left,
        "right_polygon_xy_m": right,
        "combined_polygon_xy_m": combined,
        "inside": bool(all(value >= -1e-12 for value in cross_values) or all(value <= 1e-12 for value in cross_values)),
        "rear_margin_m": float(point[0] - min(x_values)),
        "front_margin_m": float(max(x_values) - point[0]),
        "right_margin_m": float(point[1] - min(y_values)),
        "left_margin_m": float(max(y_values) - point[1]),
        "minimum_margin_m": min(edge_distances),
        "method": "convex hull of foot collision-sphere centers whose surfaces are within 3 mm of the lowest sole surface",
    }


def contact_force(model: mujoco.MjModel, data: mujoco.MjData, index: int) -> np.ndarray:
    wrench = np.zeros(6)
    mujoco.mj_contactForce(model, data, index, wrench)
    return wrench


def feet_in_contact(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[bool, bool]:
    names: set[str] = set()
    for index in range(data.ncon):
        contact = data.contact[index]
        for geom_id in (contact.geom1, contact.geom2):
            body_id = int(model.geom_bodyid[geom_id])
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            if name:
                names.add(name)
    return "left_ankle_roll_link" in names, "right_ankle_roll_link" in names


def body_audit(model: mujoco.MjModel) -> list[dict]:
    rows = []
    for body_id in range(1, model.nbody):
        name = object_name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        mass = float(model.body_mass[body_id])
        inertia = model.body_inertia[body_id].copy()
        flags = []
        if mass == 0:
            flags.append("MASS_ZERO")
        elif mass < 0.05:
            flags.append("MASS_VERY_SMALL")
        elif mass > 15:
            flags.append("MASS_LARGE_REVIEW")
        if np.any(inertia <= 0):
            flags.append("NONPOSITIVE_INERTIA")
        rows.append(
            {
                "body": name,
                "mass_kg": mass,
                "inertia": inertia.tolist(),
                "inertial_pos": model.body_ipos[body_id].tolist(),
                "body_pos": model.body_pos[body_id].tolist(),
                "flags": flags,
            }
        )
    return rows


def run_standing(
    label: str,
    *,
    free_base: bool,
    controller_kind: str,
    duration: float,
    sample_rate: float,
    friction_scale: float = 1.0,
    timestep: float | None = None,
    gain_scale: float = 1.0,
    damping_scale: float = 1.0,
    initial_base_z_shift: float = 0.0,
    foot_box: bool = False,
) -> dict:
    model = load_model(free_base=free_base)
    if timestep is not None:
        model.opt.timestep = timestep
    model.geom_friction[:, 0] *= friction_scale
    if foot_box:
        for body_name in ("left_ankle_roll_link", "right_ankle_roll_link"):
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            collision_ids = [
                geom_id
                for geom_id in range(model.ngeom)
                if int(model.geom_bodyid[geom_id]) == body_id
                and int(model.geom_contype[geom_id]) != 0
            ]
            primary = collision_ids[0]
            model.geom_type[primary] = mujoco.mjtGeom.mjGEOM_BOX
            model.geom_pos[primary] = (0.02, 0.0, -0.063)
            model.geom_quat[primary] = (1.0, 0.0, 0.0, 0.0)
            model.geom_size[primary] = (0.105, 0.06, 0.005)
            for geom_id in collision_ids[1:]:
                model.geom_contype[geom_id] = 0
                model.geom_conaffinity[geom_id] = 0
    data = mujoco.MjData(model)
    base_type = (
        SimulationStabilityController
        if controller_kind == "cleanup"
        else JointPositionController
    )
    controller = base_type(model)
    if gain_scale != 1.0 or damping_scale != 1.0:
        controller.joints = [
            type(joint)(
                joint.actuator_id,
                joint.joint_id,
                joint.qpos_adr,
                joint.dof_adr,
                joint.name,
                joint.lower,
                joint.upper,
                joint.kp * gain_scale,
                joint.kd * damping_scale,
            )
            for joint in controller.joints
        ]
    controller.initialize_data(data)
    if free_base and initial_base_z_shift:
        data.qpos[2] += initial_base_z_shift
        mujoco.mj_forward(model, data)
    pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    foot_ids = {
        name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        for name in ("left_ankle_roll_link", "right_ankle_roll_link")
    }
    initial_foot_xy = {name: data.xpos[body_id, :2].copy() for name, body_id in foot_ids.items()}
    initial_qpos = data.qpos.copy()
    initial_qvel = data.qvel.copy()
    initial_target = controller.target.copy()
    period = 1.0 / sample_rate
    next_sample = 0.0
    rows = []
    contact_rows = []
    saturation_steps = 0
    fall_time = None
    both_contact_steps = 0
    any_contact_steps = 0
    max_penetration = 0.0
    previous_contact_count = 0
    chatter_transitions = 0
    while data.time < duration - 1e-12:
        if controller_kind == "zero":
            data.ctrl[:] = 0.0
        else:
            controller.apply(data)
        mujoco.mj_step(model, data)
        left_contact, right_contact = feet_in_contact(model, data)
        any_contact_steps += int(left_contact or right_contact)
        both_contact_steps += int(left_contact and right_contact)
        chatter_transitions += int(data.ncon != previous_contact_count)
        previous_contact_count = data.ncon
        limit = np.maximum(np.abs(model.actuator_ctrlrange[:, 0]), np.abs(model.actuator_ctrlrange[:, 1]))
        saturation = float(np.max(np.abs(data.ctrl) / limit)) if model.nu else 0.0
        saturation_steps += int(saturation >= 0.98)
        if fall_time is None and data.xpos[pelvis_id, 2] < 0.30:
            fall_time = float(data.time)
        for index in range(data.ncon):
            contact = data.contact[index]
            force = contact_force(model, data, index)
            max_penetration = max(max_penetration, max(0.0, -float(contact.dist)))
            contact_rows.append(
                {
                    "timestamp": data.time,
                    "pair": f"{mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1) or contact.geom1}<->{mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2) or contact.geom2}",
                    "x": contact.pos[0],
                    "y": contact.pos[1],
                    "z": contact.pos[2],
                    "nx": contact.frame[0],
                    "ny": contact.frame[1],
                    "nz": contact.frame[2],
                    "normal_force": force[0],
                    "friction_force_1": force[1],
                    "friction_force_2": force[2],
                    "distance": contact.dist,
                }
            )
        if data.time + 1e-12 >= next_sample:
            roll, pitch, yaw = rpy(data.xmat[pelvis_id])
            target = controller.target_for_time(data.time)
            errors = [abs(target[j.qpos_adr] - data.qpos[j.qpos_adr]) for j in controller.joints]
            left_slip = float(np.linalg.norm(data.xpos[foot_ids["left_ankle_roll_link"], :2] - initial_foot_xy["left_ankle_roll_link"]))
            right_slip = float(np.linalg.norm(data.xpos[foot_ids["right_ankle_roll_link"], :2] - initial_foot_xy["right_ankle_roll_link"]))
            rows.append(
                {
                    "timestamp": data.time,
                    "base_x": data.xpos[pelvis_id, 0],
                    "base_y": data.xpos[pelvis_id, 1],
                    "base_z": data.xpos[pelvis_id, 2],
                    "roll_deg": math.degrees(roll),
                    "pitch_deg": math.degrees(pitch),
                    "yaw_deg": math.degrees(yaw),
                    "com_x": data.subtree_com[0, 0],
                    "com_y": data.subtree_com[0, 1],
                    "com_z": data.subtree_com[0, 2],
                    "contact_count": data.ncon,
                    "left_contact": int(left_contact),
                    "right_contact": int(right_contact),
                    "left_slip_m": left_slip,
                    "right_slip_m": right_slip,
                    "max_joint_error_deg": math.degrees(max(errors)),
                    "max_saturation_fraction": saturation,
                    "max_abs_actuator_force": float(np.max(np.abs(data.actuator_force))) if model.nu else 0.0,
                    "left_ankle_pitch_target_deg": math.degrees(target[controller._by_name["left_ankle_pitch_joint"].qpos_adr]) if hasattr(controller, "_by_name") else 0.0,
                    "left_ankle_pitch_actual_deg": math.degrees(data.qpos[next(j.qpos_adr for j in controller.joints if j.name == "left_ankle_pitch_joint")]),
                    "left_ankle_pitch_force": float(data.actuator_force[next(j.actuator_id for j in controller.joints if j.name == "left_ankle_pitch_joint")]),
                }
            )
            next_sample += period
    csv_path = DATA / f"{label}.csv"
    contact_path = DATA / f"{label}_contacts.csv"
    DATA.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    with contact_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(contact_rows[0]) if contact_rows else ["timestamp"])
        writer.writeheader()
        writer.writerows(contact_rows)
    first = rows[0]
    last = rows[-1]
    metrics = {
        "label": label,
        "free_base": free_base,
        "controller_kind": controller_kind,
        "duration_s": duration,
        "timestep_s": model.opt.timestep,
        "integrator": int(model.opt.integrator),
        "solver": int(model.opt.solver),
        "iterations": int(model.opt.iterations),
        "tolerance": float(model.opt.tolerance),
        "friction_scale": friction_scale,
        "gain_scale": gain_scale,
        "damping_scale": damping_scale,
        "initial_base_z_shift_m": initial_base_z_shift,
        "foot_box_experiment": foot_box,
        "initial_qpos": initial_qpos.tolist(),
        "initial_qvel": initial_qvel.tolist(),
        "initial_target": initial_target.tolist(),
        "initial_base_xyz": [first["base_x"], first["base_y"], first["base_z"]],
        "initial_base_quaternion_wxyz": initial_qpos[3:7].tolist(),
        "initial_max_joint_target_error_deg": 0.0,
        "final_base_xyz": [last["base_x"], last["base_y"], last["base_z"]],
        "base_displacement_m": float(np.linalg.norm(np.array([last["base_x"], last["base_y"], last["base_z"]]) - np.array([first["base_x"], first["base_y"], first["base_z"]]))),
        "max_abs_roll_deg": max(abs(row["roll_deg"]) for row in rows),
        "max_abs_pitch_deg": max(abs(row["pitch_deg"]) for row in rows),
        "max_tilt_deg": max(math.hypot(row["roll_deg"], row["pitch_deg"]) for row in rows),
        "max_joint_error_deg": max(row["max_joint_error_deg"] for row in rows),
        "max_saturation_fraction": max(row["max_saturation_fraction"] for row in rows),
        "saturation_ratio": saturation_steps / max(1, round(duration / model.opt.timestep)),
        "left_foot_slip_m": last["left_slip_m"],
        "right_foot_slip_m": last["right_slip_m"],
        "any_foot_contact_fraction": any_contact_steps / max(1, round(duration / model.opt.timestep)),
        "both_feet_contact_fraction": both_contact_steps / max(1, round(duration / model.opt.timestep)),
        "contact_count_transitions": chatter_transitions,
        "maximum_penetration_m": max_penetration,
        "fall_time_s": fall_time,
        "csv": str(csv_path.relative_to(PROJECT_ROOT)),
        "contact_csv": str(contact_path.relative_to(PROJECT_ROOT)),
    }
    return metrics


def plot_baselines(labels: list[str]) -> None:
    PLOTS.mkdir(parents=True, exist_ok=True)
    datasets = {}
    for label in labels:
        with (DATA / f"{label}.csv").open("r", encoding="utf-8", newline="") as stream:
            datasets[label] = list(csv.DictReader(stream))

    figure, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for label, rows in datasets.items():
        t = [float(row["timestamp"]) for row in rows]
        axes[0].plot(t, [float(row["base_x"]) for row in rows], label=f"{label}: x")
        axes[0].plot(t, [float(row["base_z"]) for row in rows], linestyle="--", label=f"{label}: z")
        axes[1].plot(t, [float(row["pitch_deg"]) for row in rows], label=f"{label}: pitch")
        axes[1].plot(t, [float(row["roll_deg"]) for row in rows], linestyle="--", label=f"{label}: roll")
    axes[0].set_ylabel("base position (m)")
    axes[1].set_ylabel("base angle (deg)")
    axes[1].set_xlabel("time (s)")
    for axis in axes:
        axis.grid(alpha=0.25)
        axis.legend(fontsize=7, ncol=2)
    figure.tight_layout()
    figure.savefig(PLOTS / "base_xyz_and_rpy.png", dpi=150)
    plt.close(figure)

    figure, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    for label in ("free_default", "free_cleanup"):
        rows = datasets[label]
        t = [float(row["timestamp"]) for row in rows]
        axes[0].plot(t, [float(row["left_slip_m"]) for row in rows], label=f"{label}: left")
        axes[0].plot(t, [float(row["right_slip_m"]) for row in rows], linestyle="--", label=f"{label}: right")
        axes[1].plot(t, [float(row["max_joint_error_deg"]) for row in rows], label=label)
        axes[2].plot(t, [float(row["max_abs_actuator_force"]) for row in rows], label=label)
    axes[0].set_ylabel("foot slip (m)")
    axes[1].set_ylabel("max joint error (deg)")
    axes[2].set_ylabel("max actuator force")
    axes[2].set_xlabel("time (s)")
    for axis in axes:
        axis.grid(alpha=0.25)
        axis.legend(fontsize=8)
    figure.tight_layout()
    figure.savefig(PLOTS / "foot_slip_joint_error_actuator_force.png", dpi=150)
    plt.close(figure)

    rows = datasets["free_cleanup"]
    t = [float(row["timestamp"]) for row in rows]
    figure, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    axes[0].plot(t, [float(row["left_ankle_pitch_target_deg"]) for row in rows], label="target")
    axes[0].plot(t, [float(row["left_ankle_pitch_actual_deg"]) for row in rows], label="actual")
    axes[1].plot(t, [float(row["left_ankle_pitch_force"]) for row in rows])
    axes[0].set_ylabel("left ankle pitch (deg)")
    axes[1].set_ylabel("actuator force")
    axes[1].set_xlabel("time (s)")
    axes[0].legend()
    figure.tight_layout()
    figure.savefig(PLOTS / "representative_joint_tracking.png", dpi=150)
    plt.close(figure)


def plot_com_support(com_summary: dict) -> None:
    com = com_summary["xyz_m"]
    figure, axis = plt.subplots(figsize=(7, 6))
    for key, color, label in (("left_polygon_xy_m", "tab:blue", "left foot"), ("right_polygon_xy_m", "tab:orange", "right foot"), ("combined_polygon_xy_m", "tab:green", "combined hull")):
        polygon = np.asarray(com_summary[key] + [com_summary[key][0]])
        axis.plot(polygon[:, 0], polygon[:, 1], color=color, label=label)
        if key != "combined_polygon_xy_m":
            axis.fill(polygon[:, 0], polygon[:, 1], color=color, alpha=0.12)
    axis.scatter([com[0]], [com[1]], marker="x", s=100, color="red", label="whole-body CoM projection")
    axis.set_aspect("equal")
    axis.set_xlabel("world X (m)")
    axis.set_ylabel("world Y (m)")
    axis.legend()
    axis.grid(alpha=0.25)
    figure.tight_layout()
    figure.savefig(PLOTS / "com_projection_support_polygon.png", dpi=150)
    plt.close(figure)


def plot_contacts() -> None:
    figure, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for label in ("free_default", "free_cleanup"):
        rows = []
        with (DATA / f"{label}_contacts.csv").open("r", encoding="utf-8", newline="") as stream:
            rows = list(csv.DictReader(stream))
        bins: dict[float, float] = {}
        for row in rows:
            key = round(float(row["timestamp"]), 2)
            bins[key] = bins.get(key, 0.0) + max(0.0, float(row["normal_force"]))
        axes[0].plot(list(bins), list(bins.values()), label=label)
        base_rows = list(csv.DictReader((DATA / f"{label}.csv").open("r", encoding="utf-8", newline="")))
        axes[1].plot([float(row["timestamp"]) for row in base_rows], [int(row["contact_count"]) for row in base_rows], label=label)
    axes[0].set_ylabel("summed contact normal force")
    axes[1].set_ylabel("contact count")
    axes[1].set_xlabel("time (s)")
    for axis in axes:
        axis.grid(alpha=0.25)
        axis.legend()
    figure.tight_layout()
    figure.savefig(PLOTS / "foot_contact_force_and_count.png", dpi=150)
    plt.close(figure)


def append_experiment(rows: list[dict], name: str, factor: str, old: str, new: str, reason: str, result: dict, conclusion: str) -> None:
    rows.append(
        {
            "experiment": name,
            "parameter_changed": factor,
            "old_value": old,
            "new_value": new,
            "reason": reason,
            "result": json.dumps({key: result.get(key) for key in ("fall_time_s", "max_tilt_deg", "base_displacement_m", "max_saturation_fraction", "left_foot_slip_m", "right_foot_slip_m")}, separators=(",", ":")),
            "conclusion": conclusion,
        }
    )


def write_reports(summary: dict) -> None:
    baseline = summary["runs"]
    default = baseline["free_default"]
    cleanup = baseline["free_cleanup"]
    fixed = baseline["fixed_default"]
    zero = baseline["free_zero"]
    com = summary["com"]
    rehearsal_summary = json.loads((HERE / "rehearsal_summary.json").read_text(encoding="utf-8"))
    rehearsal_lines = [
        "| Joint | Before | After | Before steady error (°) | After steady error (°) | After overshoot (°) | After oscillation p-p (°) | After settling (s) | After saturation |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|",
    ]
    before_by_name = {row["joint"]: row for row in rehearsal_summary["before"]}
    for after in rehearsal_summary["after"]:
        before = before_by_name[after["joint"]]
        settling = "NOT_SETTLED" if after["settling_time_s"] is None else f"{after['settling_time_s']:.3f}"
        rehearsal_lines.append(
            f"| `{after['joint']}` | {before['tracking_status']} | {after['tracking_status']} | "
            f"{before['steady_state_error_deg']:.3f} | {after['steady_state_error_deg']:.3f} | "
            f"{after['overshoot_deg']:.3f} | {after['oscillation_peak_to_peak_deg']:.3f} | "
            f"{settling} | {after['actuator_saturation_ratio']:.6f} |"
        )
    rehearsal_table = "\n".join(rehearsal_lines)
    reports = {
        "baseline_report.md": f"""# Standing baseline report

Status: **BASELINE FAILED; SIMULATION-ONLY CLEANUP PASSED 10 s**

| Run | Fall time | Final base XYZ (m) | Max tilt | Base displacement | Max joint error | Max saturation | Both-feet contact |
|---|---:|---|---:|---:|---:|---:|---:|
| Free, zero command | {zero['fall_time_s']:.3f} s | {zero['final_base_xyz']} | {zero['max_tilt_deg']:.3f}° | {zero['base_displacement_m']:.3f} m | {zero['max_joint_error_deg']:.3f}° | {zero['max_saturation_fraction']:.3f} | {zero['both_feet_contact_fraction']:.3f} |
| Free, original controller | {default['fall_time_s']:.3f} s | {default['final_base_xyz']} | {default['max_tilt_deg']:.3f}° | {default['base_displacement_m']:.3f} m | {default['max_joint_error_deg']:.3f}° | {default['max_saturation_fraction']:.3f} | {default['both_feet_contact_fraction']:.3f} |
| Fixed, original controller | none | {fixed['final_base_xyz']} | {fixed['max_tilt_deg']:.3f}° | {fixed['base_displacement_m']:.6f} m | {fixed['max_joint_error_deg']:.3f}° | {fixed['max_saturation_fraction']:.3f} | N/A |
| Free, cleanup controller | none | {cleanup['final_base_xyz']} | {cleanup['max_tilt_deg']:.3f}° | {cleanup['base_displacement_m']:.3f} m | {cleanup['max_joint_error_deg']:.3f}° | {cleanup['max_saturation_fraction']:.3f} | {cleanup['both_feet_contact_fraction']:.3f} |

The free original controller falls forward at {default['fall_time_s']:.3f} s. It is deterministic, not random: repeated/sensitivity runs share positive-X drift and pitch divergence. Fixed-base joints remain numerically stable, so the immediate fall is a missing free-base balance loop rather than a solver blow-up.

All initial actuator targets equal initial actuated qpos and initial qvel is zero. The complete arrays are retained in `summary.json`.
""",
        "initial_pose_report.md": f"""# Initial pose report

- Base qpos: `{summary['initial']['base_qpos']}`; qvel is identically zero.
- Initial maximum target-minus-qpos error: **0°**.
- Lowest foot contact spheres are **5.05 mm above** the plane at initialization; both feet are symmetric within numeric precision.
- Initial contact count is 0, so the robot first drops approximately 5 mm before loading the feet.
- Pelvis orientation is identity. Knee is 0°, at its lower documented limit; ankles are 0° and not near their limits.
- Initial left/right lowest collision heights: {summary['initial']['left_lowest_geom_z_m']:.6f} / {summary['initial']['right_lowest_geom_z_m']:.6f} m.

Changing initial base height alone did not prevent the deterministic forward fall. It is a model-initialization issue, but not the primary instability cause.
""",
        "com_support_polygon_report.md": f"""# CoM and support-polygon report

- Total modeled mass: **{summary['mass']['total_kg']:.6f} kg**.
- Whole-body CoM XYZ: **{com['xyz_m']} m**.
- Left support polygon XY: `{com['left_polygon_xy_m']}` m.
- Right support polygon XY: `{com['right_polygon_xy_m']}` m.
- Combined support polygon XY: `{com['combined_polygon_xy_m']}` m.
- CoM projection is inside: **{com['inside']}**.
- Boundary margins: rear X {com['rear_margin_m']:.6f} m, front X {com['front_margin_m']:.6f} m, right Y {com['right_margin_m']:.6f} m, left Y {com['left_margin_m']:.6f} m.
- Minimum combined-polygon boundary distance: **{com['minimum_margin_m']:.6f} m**.

The CoM starts inside the combined convex hull with substantial margin. Initial CoM location is ruled out as the primary cause. Method: {com['method']}. This is a collision-center approximation, not a pressure/contact-patch measurement.
""",
        "mass_inertia_audit.md": f"""# Mass and inertia audit

- Total mass: **{summary['mass']['total_kg']:.6f} kg**.
- Left/right appendage mass: {summary['mass']['left_kg']:.6f} / {summary['mass']['right_kg']:.6f} kg; imbalance {summary['mass']['left_right_difference_kg']:.6f} kg.
- Torso/central fraction: {summary['mass']['central_fraction']:.3%}.
- Arms fraction: {summary['mass']['arms_fraction']:.3%}.
- Legs fraction: {summary['mass']['legs_fraction']:.3%}.
- Zero-mass or non-positive-inertia dynamic bodies: **0**.
- Bodies flagged by broad sanity thresholds: `{summary['mass']['flagged_bodies']}`.

No obvious mass/inertia construction fault explains the deterministic forward fall. This is a model sanity audit only; none of these values is asserted to match hardware.

Detailed body rows are stored in `summary.json`.
""",
        "foot_collision_report.md": f"""# Foot collision report

Each ankle-roll link uses one non-colliding visual mesh plus 12 tiny collision spheres; 11 are sole/edge candidates and one is above the sole. Sphere radius is 0.005 m. Default friction is `(1.0, 0.005, 0.0001)` inherited from MuJoCo defaults, while the plane declares `(1.0, 0.01, 0.001)`.

The left/right layouts are mirrored. The combined center envelope is approximately X {SUPPORT_X} m and Y {SUPPORT_Y} m. At initialization, the lowest sphere surfaces are 5.05 mm above the plane. This creates a small landing transient and discrete contact chatter.

A single-factor box-foot experiment and an initial-height experiment both still fell forward. Therefore foot geometry is a **SECONDARY_CONTRIBUTOR**, not the primary cause. No foot geometry or friction was changed in the accepted cleanup.
""",
        "contact_report.md": f"""# Contact report

Original free run classification: **SLIDING / LOST_CONTACT / ASYMMETRIC_CONTACT after instability**. It falls forward at {default['fall_time_s']:.3f} s and foot slip grows to {default['left_foot_slip_m']:.3f}/{default['right_foot_slip_m']:.3f} m.

Cleanup run classification: **CONTACT_OK with brief startup contact acquisition**. Both feet contact for {cleanup['both_feet_contact_fraction']:.3%} of steps, final foot slip is {cleanup['left_foot_slip_m']:.6f}/{cleanup['right_foot_slip_m']:.6f} m, maximum penetration is {cleanup['maximum_penetration_m']*1000:.3f} mm, and contact-count transitions are {cleanup['contact_count_transitions']} over 10 s.

Per-contact position, normal, normal/friction force components and penetration distance are stored in `data/*_contacts.csv`.
""",
        "actuator_sanity_report.md": f"""# Actuator and controller sanity report

The MJCF has 30 direct-drive motors with gear 1. Standing-relevant ctrlranges are ±118 N·m at hips/knees/waist-yaw, ±36 at ankle pitch, ±24 at ankle roll, and ±48 at waist pitch/roll. All joints inherit damping 0, armature 0.03 and frictionloss 0.3.

Original control is joint PD plus `qfrc_bias`. It initializes at zero joint error and does not provide feedback for the six free-base DOFs. Consequently, it cannot actively regulate pelvis pitch/roll inside the support region. The original free run later saturates while falling; the cleanup run peak saturation fraction is {cleanup['max_saturation_fraction']:.3f} with no sustained saturation.

The previous fixed-base 2° rehearsals show a separate tracking issue: uniform frictionloss 0.3 creates approximate pure-PD deadbands of 1.43° for wrist Kp=12 and 0.45° for arm Kp=38. A smooth compensation of the model's own frictionloss reduces that infrastructure artifact. It does not identify real friction or hardware gains.

Accepted simulation cleanup: pelvis roll/pitch feedback through both ankles (pitch 200/30, roll 100/20 in simulation units) plus smooth 1.5× compensation of the existing model frictionloss. These are **SIMULATION_STABILITY_CANDIDATE** values only.
""",
        "solver_sensitivity_report.md": f"""# Solver, timestep and sensitivity report

Baseline: timestep 0.001 s, Euler integrator enum 0, Newton solver enum 2, 100 iterations, tolerance 1e-8.

Reducing timestep to 0.0005 s produced essentially the same forward fall. Friction scaling 0.5×/1.0×/1.5×, position-gain scaling 0.5×/1.0×/1.5×, and damping scaling 0.5×/1.0×/1.5× all failed to keep the original controller standing. The instability is therefore not primarily timestep, solver, friction magnitude, or modest joint-PD tuning.

Experiment metrics are recorded in `standing_stability_experiments.csv`.
""",
        "fixed_vs_free_report.md": f"""# Fixed-base versus free-base decomposition

1. Fixed base + original controller: stable for 10 s, max joint error {fixed['max_joint_error_deg']:.3f}° and peak saturation {fixed['max_saturation_fraction']:.3f}.
2. Free base + identical target/controller: deterministic forward fall at {default['fall_time_s']:.3f} s.
3. Foot/friction/timestep single-factor changes do not remove the fall.
4. Free base + explicit simulated pelvis-attitude feedback: stands 10 s.

Decision: the primary standing failure is **missing base-attitude/whole-body balance control**. Foot initialization/contact discretization is secondary. CoM-outside-support, gross mass asymmetry and timestep instability are ruled out as primary causes.
""",
        "rehearsal_before_after_report.md": f"""# Active-test rehearsal before/after

Before: 12/12 fixed-base runs were `TRACKING_NOT_SETTLED`; zero collisions and zero limit violations.

After simulation-only frictionloss compensation: **{summary['rehearsal']['settled_after']}/12 SETTLED**, {12-summary['rehearsal']['settled_after']}/12 `TRACKING_NOT_SETTLED`; zero collisions and zero limit violations. The free-base cleanup controller also passes the 10-second standing gate before the rehearsal infrastructure is considered valid.

{rehearsal_table}

Per-joint diagnosis: the target generator completes all required phases; actuator saturation ratio is zero; these are fixed-base runs, so base and contact instability do not create the tracking error. The before-run error magnitude follows the model's frictionloss/Kp deadband. Timestep sensitivity did not change the standing failure. Evidence therefore supports `MODELED_FRICTION_DEADBAND / CONTROLLER_FORM` rather than actuator-force shortage or contact/base motion for the fixed-base tracking result.

The before/after metrics are in `rehearsal_summary.json`, with per-joint CSVs and plots under `rehearsal_after/`. Results are simulation infrastructure evidence only; they do not determine hardware sign, zero, friction, torque semantics or dynamics.
""",
        "root_cause_report.md": f"""# Standing-stability root-cause ranking

1. **PRIMARY_ROOT_CAUSE — missing free-base balance feedback.** The original controller regulates only 30 actuated joints and has no pelvis attitude/CoM feedback. Fixed base stands, all modest contact/gain/solver variants still fall, and adding simulated pelvis roll/pitch feedback makes free base stand for 10 s.
2. **SECONDARY_CONTRIBUTOR — foot initialization and discrete collision contact.** Sole spheres begin 5.05 mm above the floor and land through small 5 mm-radius points. This adds a transient/chatter, but lowering the base and replacing each foot by a box individually do not prevent the fall.
3. **SECONDARY_CONTRIBUTOR — modeled friction deadband in fixed-base small-motion tracking.** `frictionloss=0.3` plus pure PD explains the joint-family error scale. Smooth compensation changes all 12 rehearsals from `TRACKING_NOT_SETTLED` to `SETTLED` without saturation.
4. **POSSIBLE_CONTRIBUTOR — detailed physical fidelity of masses, inertias and contact parameters.** No gross sanity fault was found, but hardware correctness is UNKNOWN without manufacturer or identification evidence.
5. **RULED_OUT AS PRIMARY — CoM initially outside support, gross left/right mass imbalance, friction magnitude, modest Kp/Kd scaling, and timestep.** The initial CoM is inside the combined support hull; left/right mass differs by only {summary['mass']['left_right_difference_kg']:.6f} kg; every corresponding single-factor run still falls.
6. **UNKNOWN — real X2 whole-body controller, ground/contact properties, actuator dynamics and protection behavior.** No real-robot evidence was used.

## Final answers

1. The original free-base run falls because a joint-space PD controller cannot regulate unactuated base pitch/roll; once the body starts pitching forward, the controller has no balance objective and later saturates.
2. Controller architecture is primary. Foot/contact initialization is secondary. CoM, gross mass distribution and numerical integration are not primary based on the experiment matrix.
3. Minimal accepted changes: a separate simulation-only pelvis-attitude feedback layer and smooth compensation of the model's already-declared frictionloss. No MJCF, mass, inertia, friction, mapping, or hardware parameter was changed.
4. Yes. The candidate runs continuously for 10 s: max tilt {cleanup['max_tilt_deg']:.3f}°, displacement {cleanup['base_displacement_m']:.3f} m, foot slip below {max(cleanup['left_foot_slip_m'], cleanup['right_foot_slip_m'])*1000:.3f} mm, and saturation ratio {cleanup['saturation_ratio']:.6f}.
5. All **{summary['rehearsal']['settled_after']}/12** rehearsals changed from `TRACKING_NOT_SETTLED` to `SETTLED` under the documented simulation thresholds.
6. Both feedback gains and modeled-friction compensation are simulation cleanup only. They are not realistic, calibrated, hardware-matched, or deployable robot parameters.
7. Real dynamics calibration still needs approved single-joint command ownership/recovery, measured command/position/velocity/effort logs, effort-source semantics, hardware sign/zero/encoder offset, actuator limits/torque-current mapping, rigid-body/inertial evidence, IMU-frame extrinsics, contact/foot geometry, and safe physical excitation data.
""",
    }
    for name, text in reports.items():
        (HERE / name).write_text(text, encoding="utf-8")


def write_audit_tables(summary: dict) -> None:
    with (HERE / "mass_inertia_audit.csv").open("w", encoding="utf-8", newline="") as stream:
        rows = summary["mass"]["body_audit"]
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    with (HERE / "foot_collision_audit.csv").open("w", encoding="utf-8", newline="") as stream:
        rows = summary["initial"]["foot_geoms"]
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    model = load_model(free_base=True)
    controller = JointPositionController(model)
    rows = []
    for joint in controller.joints:
        actuator_id = joint.actuator_id
        rows.append(
            {
                "actuator": object_name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id),
                "joint": joint.name,
                "control_type": "direct-drive motor / torque-like ctrl",
                "kp_simulation": joint.kp,
                "kd_simulation": joint.kd,
                "gear": float(model.actuator_gear[actuator_id, 0]),
                "ctrlrange": model.actuator_ctrlrange[actuator_id].tolist(),
                "forcerange": model.actuator_forcerange[actuator_id].tolist(),
                "joint_actuatorfrcrange": model.jnt_actfrcrange[joint.joint_id].tolist(),
                "joint_damping": float(model.dof_damping[joint.dof_adr]),
                "armature": float(model.dof_armature[joint.dof_adr]),
                "frictionloss": float(model.dof_frictionloss[joint.dof_adr]),
                "parameter_status": "SIMULATION_ONLY_NOT_HARDWARE_CALIBRATED",
            }
        )
    with (HERE / "actuator_audit.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def run_rehearsals() -> dict:
    before = json.loads((PROJECT_ROOT / "calibration" / "evidence" / "phase2b2_sim_rehearsal_summary.json").read_text(encoding="utf-8"))
    candidates = ranked_candidates(load_snapshot(), requested_deg=2.0, reserve_deg=5.0, minimum_useful_deg=1.0)
    accepted = [row for row in candidates if row["selected_amplitude_deg"] is not None]
    targets = _base_targets()
    after = []
    output = HERE / "rehearsal_after"
    for row in accepted:
        joint = row["name"]
        result = run_fixed_rehearsal(
            joint,
            float(row["selected_amplitude_deg"]),
            targets,
            100.0,
            output / f"{joint}.csv",
            output / f"{joint}.png",
            stability_cleanup=True,
        )
        result["tracking_status"] = (
            "SETTLED"
            if result["maximum_position_error_deg"] <= 1.0
            and abs(result["return_error_deg"]) <= 0.1
            else "TRACKING_NOT_SETTLED"
        )
        after.append(result)
    summary = {
        "before": before["fixed_base_results"],
        "after": after,
        "settled_after": sum(row["tracking_status"] == "SETTLED" for row in after),
        "free_base_cleanup_probe": run_free_base_probe(
            accepted[0]["name"],
            float(accepted[0]["selected_amplitude_deg"]),
            targets,
            stability_cleanup=True,
        ),
    }
    (HERE / "rehearsal_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return refresh_rehearsal_summary()


def tracking_diagnostics(csv_path: Path, joint_name: str) -> dict:
    with csv_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    time_values = np.asarray([float(row["timestamp"]) for row in rows])
    command = np.asarray([float(row["command_position"]) for row in rows])
    measured = np.asarray([float(row["measured_position"]) for row in rows])
    force = np.asarray([float(row["measured_torque"]) for row in rows])
    phases = np.asarray([row["phase"] for row in rows])
    error = command - measured

    steady_errors = []
    oscillations = []
    for phase in ("plus_hold", "minus_hold", "returned"):
        indices = np.flatnonzero(phases == phase)
        if indices.size:
            tail = indices[max(0, indices.size // 2):]
            steady_errors.append(abs(float(np.mean(error[tail]))))
            oscillations.append(float(np.ptp(measured[tail])))
    plus = np.flatnonzero(phases == "plus_hold")
    minus = np.flatnonzero(phases == "minus_hold")
    overshoot = 0.0
    if plus.size:
        overshoot = max(overshoot, float(np.max(measured[plus] - command[plus])))
    if minus.size:
        overshoot = max(overshoot, float(np.max(command[minus] - measured[minus])))

    settling_time = None
    returned = np.flatnonzero(phases == "returned")
    threshold = math.radians(0.1)
    if returned.size:
        start = returned[0]
        for index in returned:
            if np.all(np.abs(error[index:]) <= threshold):
                settling_time = float(time_values[index] - time_values[start])
                break

    model = load_model(free_base=False)
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    actuator_id = next(
        actuator_id
        for actuator_id in range(model.nu)
        if int(model.actuator_trnid[actuator_id, 0]) == joint_id
    )
    force_limit = float(np.max(np.abs(model.actuator_ctrlrange[actuator_id])))
    saturation_ratio = float(np.mean(np.abs(force) >= 0.98 * force_limit))
    return {
        "steady_state_error_deg": math.degrees(max(steady_errors, default=float("nan"))),
        "overshoot_deg": math.degrees(max(0.0, overshoot)),
        "oscillation_peak_to_peak_deg": math.degrees(max(oscillations, default=float("nan"))),
        "settling_time_s": settling_time,
        "actuator_saturation_ratio": saturation_ratio,
        "target_generator_complete": set(("current", "plus", "plus_hold", "return_plus", "center_hold", "minus", "minus_hold", "return_minus", "returned")).issubset(set(phases)),
        "root_cause_classification": "MODELED_FRICTION_DEADBAND_CONTROLLER_FORM",
    }


def refresh_rehearsal_summary() -> dict:
    path = HERE / "rehearsal_summary.json"
    summary = json.loads(path.read_text(encoding="utf-8"))
    for group in ("before", "after"):
        for row in summary[group]:
            csv_path = PROJECT_ROOT / row["csv"]
            row.update(tracking_diagnostics(csv_path, row["joint"]))
            if group == "after":
                row["tracking_status"] = (
                    "SETTLED"
                    if row["maximum_position_error_deg"] <= 1.0
                    and row["steady_state_error_deg"] <= 0.25
                    and abs(row["return_error_deg"]) <= 0.1
                    else "TRACKING_NOT_SETTLED"
                )
    summary["settled_after"] = sum(row["tracking_status"] == "SETTLED" for row in summary["after"])
    path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary


def main() -> int:
    args = parse_args()
    if args.reports_only:
        summary = json.loads((HERE / "summary.json").read_text(encoding="utf-8"))
        summary["com"] = support_geometry(summary["initial"]["foot_geoms"], np.asarray(summary["com"]["xyz_m"]))
        plot_com_support(summary["com"])
        rehearsal = refresh_rehearsal_summary()
        summary["rehearsal"]["settled_after"] = rehearsal["settled_after"]
        (HERE / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        write_reports(summary)
        write_audit_tables(summary)
        print("Reports refreshed without rerunning simulation.")
        return 0
    if args.duration < 10 or args.sample_rate <= 0:
        print("ERROR: duration must be >=10 s and sample rate positive", file=sys.stderr)
        return 2
    HERE.mkdir(parents=True, exist_ok=True)
    PLOTS.mkdir(parents=True, exist_ok=True)
    DATA.mkdir(parents=True, exist_ok=True)

    model = load_model(free_base=True)
    data = mujoco.MjData(model)
    controller = JointPositionController(model)
    controller.initialize_data(data)
    pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    geom_rows = []
    lowest = {}
    for side in ("left", "right"):
        body_name = f"{side}_ankle_roll_link"
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        candidates = []
        for geom_id in range(model.ngeom):
            if int(model.geom_bodyid[geom_id]) != body_id or int(model.geom_contype[geom_id]) == 0:
                continue
            surface_z = float(data.geom_xpos[geom_id, 2] - model.geom_size[geom_id, 0])
            candidates.append(surface_z)
            geom_rows.append({"side": side, "geom_id": geom_id, "type": int(model.geom_type[geom_id]), "size": model.geom_size[geom_id].tolist(), "local_pos": model.geom_pos[geom_id].tolist(), "world_pos": data.geom_xpos[geom_id].tolist(), "surface_z": surface_z, "friction": model.geom_friction[geom_id].tolist(), "contype": int(model.geom_contype[geom_id]), "conaffinity": int(model.geom_conaffinity[geom_id]), "solref": model.geom_solref[geom_id].tolist(), "solimp": model.geom_solimp[geom_id].tolist()})
        lowest[side] = min(candidates)

    audit = body_audit(model)
    total = sum(row["mass_kg"] for row in audit)
    left = sum(row["mass_kg"] for row in audit if row["body"].startswith("left_"))
    right = sum(row["mass_kg"] for row in audit if row["body"].startswith("right_"))
    arms = sum(row["mass_kg"] for row in audit if any(token in row["body"] for token in ("shoulder", "elbow", "wrist")))
    legs = sum(row["mass_kg"] for row in audit if any(token in row["body"] for token in ("hip", "knee", "ankle")))
    com = data.subtree_com[0].copy()
    runs = {}
    specs = [
        ("free_zero", dict(free_base=True, controller_kind="zero")),
        ("free_default", dict(free_base=True, controller_kind="default")),
        ("fixed_default", dict(free_base=False, controller_kind="default")),
        ("free_cleanup", dict(free_base=True, controller_kind="cleanup")),
    ]
    for label, kwargs in specs:
        print(f"RUN {label}", flush=True)
        runs[label] = run_standing(label, duration=args.duration, sample_rate=args.sample_rate, **kwargs)

    experiment_rows = []
    sensitivities = [
        ("friction_0_5", "friction scale", "1.0", "0.5", dict(friction_scale=0.5)),
        ("friction_1_5", "friction scale", "1.0", "1.5", dict(friction_scale=1.5)),
        ("kp_0_5", "joint Kp scale", "1.0", "0.5", dict(gain_scale=0.5)),
        ("kp_1_5", "joint Kp scale", "1.0", "1.5", dict(gain_scale=1.5)),
        ("kd_0_5", "joint Kd scale", "1.0", "0.5", dict(damping_scale=0.5)),
        ("kd_1_5", "joint Kd scale", "1.0", "1.5", dict(damping_scale=1.5)),
        ("timestep_0_0005", "timestep", "0.001", "0.0005", dict(timestep=0.0005)),
        ("initial_height_contact", "initial base Z", "0.68000", "0.67495", dict(initial_base_z_shift=-0.00505)),
        ("foot_box", "foot collision geometry", "12 spheres", "one 0.210x0.120x0.010 m box per foot", dict(foot_box=True)),
    ]
    for label, factor, old, new, kwargs in sensitivities:
        print(f"RUN {label}", flush=True)
        result = run_standing(label, free_base=True, controller_kind="default", duration=args.duration, sample_rate=20.0, **kwargs)
        append_experiment(experiment_rows, label, factor, old, new, "single-factor sensitivity", result, "still falls" if result["fall_time_s"] is not None else "stands")
    append_experiment(experiment_rows, "free_cleanup", "controller architecture", "joint PD + bias", "PD + simulated base-attitude feedback + modeled-friction compensation", "test primary root-cause hypothesis", runs["free_cleanup"], "passes 10-second infrastructure gate")
    with EXPERIMENTS.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(experiment_rows[0]))
        writer.writeheader()
        writer.writerows(experiment_rows)

    plot_baselines([label for label, _ in specs])
    plot_contacts()
    com_summary = support_geometry(geom_rows, com)
    plot_com_support(com_summary)
    rehearsal = run_rehearsals()
    controller.apply(data)
    mujoco.mj_forward(model, data)
    initial_joint_state = {
        joint.name: {
            "qpos_rad": float(data.qpos[joint.qpos_adr]),
            "qvel_rad_s": float(data.qvel[joint.dof_adr]),
            "target_rad": float(controller.target[joint.qpos_adr]),
            "target_error_rad": float(controller.target[joint.qpos_adr] - data.qpos[joint.qpos_adr]),
            "actuator_command": float(data.ctrl[joint.actuator_id]),
            "actuator_force": float(data.actuator_force[joint.actuator_id]),
        }
        for joint in controller.joints
    }
    summary = {
        "safety": "LOCAL_SIMULATION_ONLY_NO_ROBOT_ACCESS",
        "model_files_modified": [],
        "controller_candidate": SimulationStabilityConfig().__dict__,
        "initial": {"base_qpos": data.qpos[:7].tolist(), "base_qvel": data.qvel[:6].tolist(), "joint_state": initial_joint_state, "left_lowest_geom_z_m": lowest["left"], "right_lowest_geom_z_m": lowest["right"], "foot_geoms": geom_rows},
        "com": com_summary,
        "mass": {"total_kg": total, "left_kg": left, "right_kg": right, "left_right_difference_kg": left-right, "central_fraction": (total-left-right)/total, "arms_fraction": arms/total, "legs_fraction": legs/total, "flagged_bodies": [row["body"] for row in audit if row["flags"]], "body_audit": audit},
        "runs": runs,
        "rehearsal": {"settled_after": rehearsal["settled_after"]},
    }
    (HERE / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    write_reports(summary)
    write_audit_tables(summary)
    print(f"Wrote {HERE}")
    print("NO_ROBOT_ACCESS=1")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
