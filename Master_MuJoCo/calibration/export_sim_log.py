"""Run MuJoCo locally and export the unified calibration CSV schema."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

import mujoco

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from calibration.log_io import REQUIRED_COLUMNS
from master_sim.controller import JointPositionController, POSES
from master_sim.model import load_model


def _sensor(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> list[float]:
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    if sensor_id < 0:
        raise KeyError(f"required sensor {name!r} is absent")
    address = int(model.sensor_adr[sensor_id])
    dimension = int(model.sensor_dim[sensor_id])
    return [float(value) for value in data.sensordata[address : address + dimension]]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_ROOT / "calibration" / "logs" / "sim" / "mujoco.csv",
    )
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--rate", type=float, default=100.0, help="CSV sample rate in Hz")
    parser.add_argument("--pose", choices=sorted(POSES), default="home")
    parser.add_argument("--free-base", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.duration <= 0 or args.rate <= 0:
        print("ERROR: duration and rate must be positive", file=sys.stderr)
        return 2
    model = load_model(free_base=args.free_base)
    data = mujoco.MjData(model)
    controller = JointPositionController(model)
    controller.set_pose(args.pose)
    controller.initialize_data(data)
    period = 1.0 / args.rate
    next_sample = 0.0
    rows_written = 0
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=REQUIRED_COLUMNS)
        writer.writeheader()
        while data.time < args.duration:
            controller.apply(data)
            mujoco.mj_step(model, data)
            if data.time + 1e-12 < next_sample:
                continue
            quaternion = _sensor(model, data, "body-orientation")
            gyro = _sensor(model, data, "body-angular-velocity")
            acceleration = _sensor(model, data, "body-linear-acceleration")
            target = controller.target_for_time(data.time)
            for joint in controller.joints:
                actuator_force = float(data.actuator_force[joint.actuator_id])
                writer.writerow(
                    {
                        "timestamp": f"{data.time:.9f}",
                        "joint_name": joint.name,
                        "command_position": f"{target[joint.qpos_adr]:.12g}",
                        "measured_position": f"{data.qpos[joint.qpos_adr]:.12g}",
                        "measured_velocity": f"{data.qvel[joint.dof_adr]:.12g}",
                        "measured_torque": f"{actuator_force:.12g}",
                        "imu_quaternion": json.dumps(quaternion, separators=(",", ":")),
                        "imu_gyro": json.dumps(gyro, separators=(",", ":")),
                        "imu_accel": json.dumps(acceleration, separators=(",", ":")),
                    }
                )
                rows_written += 1
            next_sample += period
    print(
        f"Wrote {rows_written} rows for {len(controller.joints)} joints to "
        f"{args.output.resolve()}"
    )
    print(
        "measured_torque is MuJoCo actuator_force for the direct-drive motor; "
        "it is not asserted equivalent to the robot effort signal."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
