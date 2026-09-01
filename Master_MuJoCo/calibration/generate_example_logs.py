"""Generate deterministic synthetic logs used to smoke-test the analysis tools."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parent
HEADER = (
    "timestamp",
    "joint_name",
    "command_position",
    "measured_position",
    "measured_velocity",
    "measured_torque",
    "imu_quaternion",
    "imu_gyro",
    "imu_accel",
)
JOINTS = (
    ("head_yaw", "head_yaw_joint", 0.0, 0.16, 0.7),
    ("left_knee", "left_knee_joint", 0.55, 0.35, 0.5),
    ("right_shoulder_roll", "right_shoulder_roll_joint", -0.5, 0.25, 0.9),
)


def _command(baseline: float, amplitude: float, frequency: float, time_s: float) -> float:
    return baseline + amplitude * math.sin(2.0 * math.pi * frequency * time_s)


def _write(path: Path, *, real: bool) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    step = 0.01
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=HEADER)
        writer.writeheader()
        for index in range(301):
            time_s = index * step
            imu_yaw = 0.025 * math.sin(2 * math.pi * 0.35 * time_s)
            quaternion = [math.cos(imu_yaw / 2), 0.0, 0.0, math.sin(imu_yaw / 2)]
            gyro = [0.0, 0.0, 0.025 * 2 * math.pi * 0.35 * math.cos(2 * math.pi * 0.35 * time_s)]
            accel = [0.02 * math.sin(2 * math.pi * time_s), 0.0, 9.80665]
            for hardware_name, mujoco_name, baseline, amplitude, frequency in JOINTS:
                command = _command(baseline, amplitude, frequency, time_s)
                lag = 0.07 if real else 0.02
                measured = _command(
                    baseline,
                    0.97 * amplitude,
                    frequency,
                    max(0.0, time_s - lag),
                )
                if real and hardware_name == "head_yaw":
                    measured = -measured
                if real and hardware_name == "left_knee":
                    measured = 1.28 * measured + 0.06
                velocity = (
                    0.97
                    * amplitude
                    * 2
                    * math.pi
                    * frequency
                    * math.cos(2 * math.pi * frequency * max(0.0, time_s - lag))
                )
                if real and hardware_name == "head_yaw":
                    velocity = -velocity
                if real and hardware_name == "left_knee":
                    velocity *= 1.28
                torque = 3.0 * (command - measured) - 0.15 * velocity
                writer.writerow(
                    {
                        "timestamp": f"{time_s:.6f}",
                        "joint_name": hardware_name if real else mujoco_name,
                        "command_position": f"{command:.9f}",
                        "measured_position": f"{measured:.9f}",
                        "measured_velocity": f"{velocity:.9f}",
                        "measured_torque": f"{torque:.9f}",
                        "imu_quaternion": json.dumps(quaternion, separators=(",", ":")),
                        "imu_gyro": json.dumps(gyro, separators=(",", ":")),
                        "imu_accel": json.dumps(accel, separators=(",", ":")),
                    }
                )


def main() -> None:
    _write(ROOT / "logs" / "real" / "test.csv", real=True)
    _write(ROOT / "logs" / "sim" / "test.csv", real=False)
    print("Generated synthetic calibration/logs/{real,sim}/test.csv")


if __name__ == "__main__":
    main()
