"""Run the Phase 2B-2 active-test sequence entirely in local MuJoCo."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import mujoco
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from calibration.log_io import REQUIRED_COLUMNS, load_log, load_mapping, safe_filename
from calibration.phase2b2_common import (
    CALIBRATION_DIR,
    load_snapshot,
    positions_rad,
    ranked_candidates,
)
from master_sim.controller import JointPositionController, SimulationStabilityController
from master_sim.model import load_model


EXTRA_COLUMNS = ("phase", "self_collision_count")


@dataclass(frozen=True)
class Segment:
    name: str
    start_scale: float
    end_scale: float
    duration: float


SEGMENTS = (
    Segment("current", 0.0, 0.0, 1.0),
    Segment("plus", 0.0, 1.0, 1.0),
    Segment("plus_hold", 1.0, 1.0, 0.5),
    Segment("return_plus", 1.0, 0.0, 1.0),
    Segment("center_hold", 0.0, 0.0, 0.5),
    Segment("minus", 0.0, -1.0, 1.0),
    Segment("minus_hold", -1.0, -1.0, 0.5),
    Segment("return_minus", -1.0, 0.0, 1.0),
    Segment("returned", 0.0, 0.0, 1.0),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--requested-amplitude-deg", type=float, default=2.0)
    parser.add_argument("--reserve-deg", type=float, default=5.0)
    parser.add_argument("--minimum-useful-amplitude-deg", type=float, default=1.0)
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument(
        "--resume",
        action="store_true",
        help="reuse only CSVs that contain the final returned phase",
    )
    return parser.parse_args()


def _smoothstep5(value: float) -> float:
    u = min(1.0, max(0.0, value))
    return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5


def _sensor(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> list[float]:
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    address = int(model.sensor_adr[sensor_id])
    dimension = int(model.sensor_dim[sensor_id])
    return [float(value) for value in data.sensordata[address : address + dimension]]


def _self_contacts(model: mujoco.MjModel, data: mujoco.MjData) -> list[str]:
    result: list[str] = []
    for index in range(data.ncon):
        contact = data.contact[index]
        first = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
        second = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
        if first == "floor" or second == "floor":
            continue
        result.append(f"{first or contact.geom1}<->{second or contact.geom2}")
    return result


def _latest_static_means() -> dict[str, float]:
    source = CALIBRATION_DIR / "logs" / "real" / "static_001.csv"
    rows = load_log(source)
    aliases, _ = load_mapping(CALIBRATION_DIR / "joint_mapping.csv")
    values: dict[str, list[float]] = {}
    for row in rows:
        name = aliases.get(row.joint_name, row.joint_name)
        if name.startswith("NOT_PRESENT") or name.startswith("__imu"):
            continue
        if math.isfinite(row.measured_position):
            values.setdefault(name, []).append(row.measured_position)
    return {name: float(np.mean(series)) for name, series in values.items()}


def _base_targets() -> dict[str, float]:
    targets = _latest_static_means()
    targets.update(positions_rad(load_snapshot()))
    return targets


def _phase_scale(segment: Segment, elapsed: float) -> float:
    if segment.start_scale == segment.end_scale:
        return segment.end_scale
    progress = _smoothstep5(elapsed / segment.duration)
    return segment.start_scale + (segment.end_scale - segment.start_scale) * progress


def _plot(csv_path: Path, output: Path, title: str) -> None:
    with csv_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    t = np.asarray([float(row["timestamp"]) for row in rows])
    command = np.degrees([float(row["command_position"]) for row in rows])
    measured = np.degrees([float(row["measured_position"]) for row in rows])
    velocity = np.degrees([float(row["measured_velocity"]) for row in rows])
    torque = np.asarray([float(row["measured_torque"]) for row in rows])
    contacts = np.asarray([int(row["self_collision_count"]) for row in rows])
    figure, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    axes[0].plot(t, command, label="command")
    axes[0].plot(t, measured, label="measured", alpha=0.85)
    axes[0].set_ylabel("position (deg)")
    axes[0].legend(loc="best")
    axes[1].plot(t, velocity)
    axes[1].set_ylabel("velocity (deg/s)")
    axes[2].plot(t, torque)
    axes[2].set_ylabel("actuator force")
    axes[3].step(t, contacts, where="post")
    axes[3].set_ylabel("self contacts")
    axes[3].set_xlabel("simulation time (s)")
    figure.suptitle(title)
    figure.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=150)
    plt.close(figure)


def summarize_completed_csv(
    csv_path: Path,
    plot_path: Path,
    joint_name: str,
    amplitude_deg: float,
) -> dict | None:
    if not csv_path.exists() or not plot_path.exists():
        return None
    with csv_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows or rows[-1].get("phase") != "returned":
        return None
    command = np.asarray([float(row["command_position"]) for row in rows])
    measured = np.asarray([float(row["measured_position"]) for row in rows])
    velocity = np.asarray([float(row["measured_velocity"]) for row in rows])
    force = np.asarray([float(row["measured_torque"]) for row in rows])
    contacts = np.asarray([int(row["self_collision_count"]) for row in rows])
    center = float(command[0])
    model = load_model(free_base=False)
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    lower, upper = model.jnt_range[joint_id]
    violation_steps = int(np.count_nonzero((measured < lower - 1e-9) | (measured > upper + 1e-9)))
    plus_delta = float(np.max(measured) - center)
    minus_delta = float(np.min(measured) - center)
    return {
        "joint": joint_name,
        "amplitude_deg": amplitude_deg,
        "duration_s": float(rows[-1]["timestamp"]),
        "csv": str(csv_path.relative_to(PROJECT_ROOT)),
        "plot": str(plot_path.relative_to(PROJECT_ROOT)),
        "maximum_position_error_deg": math.degrees(float(np.max(np.abs(command - measured)))),
        "peak_velocity_deg_s": math.degrees(float(np.max(np.abs(velocity)))),
        "peak_actuator_force": float(np.max(np.abs(force))),
        "return_error_deg": math.degrees(float(measured[-1] - center)),
        "measured_plus_delta_deg": math.degrees(plus_delta),
        "measured_minus_delta_deg": math.degrees(minus_delta),
        "response_sign": "SIM_SIGN_RESPONSE_MATCH" if plus_delta > 0 and minus_delta < 0 else "SIM_SIGN_RESPONSE_CONFLICT",
        "maximum_self_collision_count": int(np.max(contacts)),
        "self_collision_pairs": [] if int(np.max(contacts)) == 0 else ["not retained in CSV"],
        "model_limit_violation_steps": violation_steps,
        "fixed_base_pelvis_drift_m": None,
        "resumed_from_completed_csv": True,
        "tracking_status": (
            "PASS_REHEARSAL_TRACKING"
            if math.degrees(float(np.max(np.abs(command - measured)))) <= 0.5 * amplitude_deg
            and abs(math.degrees(float(measured[-1] - center))) <= 0.1
            else "TRACKING_NOT_SETTLED"
        ),
    }


def run_fixed_rehearsal(
    joint_name: str,
    amplitude_deg: float,
    base_targets: dict[str, float],
    rate: float,
    csv_path: Path,
    plot_path: Path,
    *,
    stability_cleanup: bool = False,
) -> dict:
    model = load_model(free_base=False)
    data = mujoco.MjData(model)
    controller_type = SimulationStabilityController if stability_cleanup else JointPositionController
    controller = controller_type(model)
    valid_names = {joint.name for joint in controller.joints}
    controller.set_targets({name: value for name, value in base_targets.items() if name in valid_names})
    controller.initialize_data(data)
    driven = {joint.name: joint for joint in controller.joints}[joint_name]
    center = float(controller.target[driven.qpos_adr])
    delta = math.radians(amplitude_deg)
    initial_pelvis = data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")].copy()
    period = 1.0 / rate
    next_sample = 0.0
    contact_pairs: set[str] = set()
    max_contacts = 0
    model_limit_violations = 0
    maximum_position_error = 0.0
    peak_velocity = 0.0
    peak_force = 0.0
    plus_peak = center
    minus_peak = center
    total_duration = sum(segment.duration for segment in SEGMENTS)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=REQUIRED_COLUMNS + EXTRA_COLUMNS)
        writer.writeheader()
        segment_start = 0.0
        for segment in SEGMENTS:
            segment_end = segment_start + segment.duration
            while data.time < segment_end - 1e-12:
                elapsed = data.time - segment_start
                scale = _phase_scale(segment, elapsed)
                command = center + scale * delta
                controller.target[driven.qpos_adr] = command
                controller.apply(data)
                mujoco.mj_step(model, data)
                contacts = _self_contacts(model, data)
                contact_pairs.update(contacts)
                max_contacts = max(max_contacts, len(contacts))
                measured = float(data.qpos[driven.qpos_adr])
                velocity = float(data.qvel[driven.dof_adr])
                force = float(data.actuator_force[driven.actuator_id])
                maximum_position_error = max(maximum_position_error, abs(command - measured))
                peak_velocity = max(peak_velocity, abs(velocity))
                peak_force = max(peak_force, abs(force))
                plus_peak = max(plus_peak, measured)
                minus_peak = min(minus_peak, measured)
                if measured < driven.lower - 1e-9 or measured > driven.upper + 1e-9:
                    model_limit_violations += 1
                if data.time + 1e-12 >= next_sample:
                    writer.writerow(
                        {
                            "timestamp": f"{data.time:.9f}",
                            "joint_name": joint_name,
                            "command_position": f"{command:.12g}",
                            "measured_position": f"{measured:.12g}",
                            "measured_velocity": f"{velocity:.12g}",
                            "measured_torque": f"{force:.12g}",
                            "imu_quaternion": json.dumps(_sensor(model, data, "body-orientation"), separators=(",", ":")),
                            "imu_gyro": json.dumps(_sensor(model, data, "body-angular-velocity"), separators=(",", ":")),
                            "imu_accel": json.dumps(_sensor(model, data, "body-linear-acceleration"), separators=(",", ":")),
                            "phase": segment.name,
                            "self_collision_count": len(contacts),
                        }
                    )
                    next_sample += period
            segment_start = segment_end
    pelvis = data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")]
    return_error = float(data.qpos[driven.qpos_adr]) - center
    _plot(csv_path, plot_path, f"Phase 2B-2 MuJoCo rehearsal: {joint_name}")
    return {
        "joint": joint_name,
        "amplitude_deg": amplitude_deg,
        "duration_s": total_duration,
        "csv": str(csv_path.relative_to(PROJECT_ROOT)),
        "plot": str(plot_path.relative_to(PROJECT_ROOT)),
        "maximum_position_error_deg": math.degrees(maximum_position_error),
        "peak_velocity_deg_s": math.degrees(peak_velocity),
        "peak_actuator_force": peak_force,
        "return_error_deg": math.degrees(return_error),
        "measured_plus_delta_deg": math.degrees(plus_peak - center),
        "measured_minus_delta_deg": math.degrees(minus_peak - center),
        "response_sign": "SIM_SIGN_RESPONSE_MATCH" if plus_peak > center and minus_peak < center else "SIM_SIGN_RESPONSE_CONFLICT",
        "maximum_self_collision_count": max_contacts,
        "self_collision_pairs": sorted(contact_pairs),
        "model_limit_violation_steps": model_limit_violations,
        "fixed_base_pelvis_drift_m": float(np.linalg.norm(pelvis - initial_pelvis)),
        "tracking_status": (
            "PASS_REHEARSAL_TRACKING"
            if math.degrees(maximum_position_error) <= 0.5 * amplitude_deg
            and abs(math.degrees(return_error)) <= 0.1
            else "TRACKING_NOT_SETTLED"
        ),
    }


def run_free_base_probe(
    joint_name: str,
    amplitude_deg: float,
    base_targets: dict[str, float],
    *,
    stability_cleanup: bool = False,
) -> dict:
    model = load_model(free_base=True)
    data = mujoco.MjData(model)
    controller_type = SimulationStabilityController if stability_cleanup else JointPositionController
    controller = controller_type(model)
    valid_names = {joint.name for joint in controller.joints}
    controller.set_targets({name: value for name, value in base_targets.items() if name in valid_names})
    controller.initialize_data(data)
    driven = {joint.name: joint for joint in controller.joints}[joint_name]
    center = float(controller.target[driven.qpos_adr])
    delta = math.radians(amplitude_deg)
    pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    initial = data.xpos[pelvis_id].copy()
    min_height = float(initial[2])
    max_xy = 0.0
    max_tilt = 0.0
    segment_start = 0.0
    for segment in SEGMENTS:
        segment_end = segment_start + segment.duration
        while data.time < segment_end - 1e-12:
            scale = _phase_scale(segment, data.time - segment_start)
            controller.target[driven.qpos_adr] = center + scale * delta
            controller.apply(data)
            mujoco.mj_step(model, data)
            pelvis = data.xpos[pelvis_id]
            minimum = float(pelvis[2])
            min_height = min(min_height, minimum)
            max_xy = max(max_xy, float(np.linalg.norm(pelvis[:2] - initial[:2])))
            rotation = data.xmat[pelvis_id].reshape(3, 3)
            body_z_world = rotation[:, 2]
            tilt = math.degrees(math.acos(float(np.clip(body_z_world[2], -1.0, 1.0))))
            max_tilt = max(max_tilt, tilt)
        segment_start = segment_end
    final = data.xpos[pelvis_id].copy()
    return {
        "joint": joint_name,
        "amplitude_deg": amplitude_deg,
        "duration_s": sum(segment.duration for segment in SEGMENTS),
        "initial_pelvis_xyz_m": initial.tolist(),
        "final_pelvis_xyz_m": final.tolist(),
        "minimum_pelvis_height_m": min_height,
        "maximum_xy_drift_m": max_xy,
        "maximum_tilt_deg": max_tilt,
    }


def write_report(summary: dict) -> None:
    lines = [
        "# Phase 2B-2 MuJoCo active-test rehearsal",
        "",
        "Status: **REHEARSAL COMPLETED; TRACKING AND BASE STABILITY NOT VALIDATED; REAL SIGN/ZERO NOT INFERRED**",
        "",
        "Sequence for each accepted candidate: current → smooth +delta → hold → return → smooth -delta → hold → return. J2 candidates were skipped by the offline 5° screening reserve.",
        "",
        "The fixed-base model is used for repeatable command, logger, joint-limit, and modeled self-collision checks. Numeric hardware coordinates are applied to same-name MuJoCo joints only as an explicit rehearsal assumption; real sign/zero remains UNKNOWN.",
        "",
        "| Joint | Delta (°) | Sign response | Tracking | Max error (°) | Peak velocity (°/s) | Peak force | Return error (°) | Max self contacts | Limit violations |",
        "|---|---:|---|---|---:|---:|---:|---:|---:|---:|",
    ]
    for result in summary["fixed_base_results"]:
        lines.append(
            f"| `{result['joint']}` | {result['amplitude_deg']:.6f} | {result['response_sign']} | {result['tracking_status']} | "
            f"{result['maximum_position_error_deg']:.6f} | {result['peak_velocity_deg_s']:.6f} | "
            f"{result['peak_actuator_force']:.6f} | {result['return_error_deg']:.6f} | "
            f"{result['maximum_self_collision_count']} | {result['model_limit_violation_steps']} |"
        )
    probe = summary["free_base_probe"]
    baseline = summary["free_base_baseline"]
    lines.extend(
        [
            "",
            "## Collision and limit result",
            "",
            f"- Fixed-base candidate runs with modeled self-collision: {sum(result['maximum_self_collision_count'] > 0 for result in summary['fixed_base_results'])}.",
            f"- Fixed-base candidate runs with MuJoCo limit violation: {sum(result['model_limit_violation_steps'] > 0 for result in summary['fixed_base_results'])}.",
            "- Collision scope is incomplete: the supplied MJCF ends at wrist-roll links and comments out some collision meshes. This does not prove physical clearance for hands, cabling, clothing, or surroundings.",
            f"- Runs classified `TRACKING_NOT_SETTLED`: {sum(result['tracking_status'] != 'PASS_REHEARSAL_TRACKING' for result in summary['fixed_base_results'])}. Target generation, logging, limit checks, and coordinate response worked, but the current uncalibrated MuJoCo controller did not meet the rehearsal tracking/return thresholds. Per scope, no gains or dynamics were changed.",
            "",
            "## Base stability probe",
            "",
            f"A free-base no-action baseline over {baseline['duration_s']:.1f}s reached max tilt {baseline['maximum_tilt_deg']:.3f}°, max XY drift {baseline['maximum_xy_drift_m']:.3f} m, and minimum pelvis height {baseline['minimum_pelvis_height_m']:.3f} m.",
            f"The top-candidate `{probe['joint']}` rehearsal reached max tilt {probe['maximum_tilt_deg']:.3f}°, max XY drift {probe['maximum_xy_drift_m']:.3f} m, and minimum pelvis height {probe['minimum_pelvis_height_m']:.3f} m.",
            "",
            "Result: **BASE_STABILITY_NOT_DEMONSTRATED**. The current MuJoCo project uses joint PD control and has no validated X2 whole-body balance controller. The free-base fall/drift is therefore an infrastructure limitation and must not be attributed to the candidate joint. The fixed-base success is likewise not real balance evidence.",
            "",
            "## Generated artifacts",
            "",
        ]
    )
    for result in summary["fixed_base_results"]:
        lines.append(f"- `{result['csv']}` and `{result['plot']}`")
    lines.extend(
        [
            "",
            "MuJoCo `actuator_force` is logged in `measured_torque`; it is not asserted equivalent to real `JointState.effort`. No real hardware sign, zero, encoder offset, dynamics, mass, inertia, friction, actuator, Kp, or Kd conclusion is drawn.",
            "",
        ]
    )
    (CALIBRATION_DIR / "phase2b2_sim_rehearsal_report.md").write_text(
        "\n".join(lines), encoding="utf-8"
    )


def main() -> int:
    args = parse_args()
    if min(args.requested_amplitude_deg, args.reserve_deg, args.minimum_useful_amplitude_deg, args.rate) <= 0:
        print("ERROR: amplitudes, reserve, and rate must be positive", file=sys.stderr)
        return 2
    snapshot = load_snapshot()
    candidates = ranked_candidates(
        snapshot,
        requested_deg=args.requested_amplitude_deg,
        reserve_deg=args.reserve_deg,
        minimum_useful_deg=args.minimum_useful_amplitude_deg,
    )
    accepted = [row for row in candidates if row["selected_amplitude_deg"] is not None]
    base_targets = _base_targets()
    csv_dir = CALIBRATION_DIR / "active_tests" / "sim"
    plot_dir = CALIBRATION_DIR / "plots" / "phase2b2_sim"
    results = []
    for row in accepted:
        stem = safe_filename(row["name"])
        csv_path = csv_dir / f"{stem}.csv"
        plot_path = plot_dir / f"{stem}.png"
        result = None
        if args.resume:
            result = summarize_completed_csv(
                csv_path,
                plot_path,
                row["name"],
                float(row["selected_amplitude_deg"]),
            )
        if result is None:
            result = run_fixed_rehearsal(
                row["name"],
                float(row["selected_amplitude_deg"]),
                base_targets,
                args.rate,
                csv_path,
                plot_path,
            )
        results.append(result)
        print(
            f"SIM_REHEARSAL_OK {row['name']} delta={row['selected_amplitude_deg']:.6f}° "
            f"resumed={bool(result.get('resumed_from_completed_csv'))}",
            flush=True,
        )
    top = accepted[0]
    baseline = run_free_base_probe(top["name"], 0.0, base_targets)
    probe = run_free_base_probe(top["name"], float(top["selected_amplitude_deg"]), base_targets)
    summary = {
        "coordinate_assumption": "same-name hardware numeric coordinate applied to MuJoCo; sign/zero UNKNOWN",
        "screening_reserve_deg": args.reserve_deg,
        "requested_amplitude_deg": args.requested_amplitude_deg,
        "minimum_useful_amplitude_deg": args.minimum_useful_amplitude_deg,
        "fixed_base_results": results,
        "skipped_joints": [row["name"] for row in candidates if row["selected_amplitude_deg"] is None],
        "free_base_baseline": baseline,
        "free_base_probe": probe,
    }
    evidence = CALIBRATION_DIR / "evidence" / "phase2b2_sim_rehearsal_summary.json"
    evidence.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    write_report(summary)
    print(f"Wrote {evidence}")
    print(f"Wrote {CALIBRATION_DIR / 'phase2b2_sim_rehearsal_report.md'}")
    print("No robot connection or command was used.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
