"""Summarize stationary-log bias, noise, drift, torque, and IMU stability."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from calibration.log_io import (
    LogFormatError,
    canonicalize_rows,
    imu_series,
    joint_names,
    joint_series,
    load_log,
    load_mapping,
)


def _fmt(value: float) -> str:
    return f"{value:.6g}" if np.isfinite(value) else "N/A"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument(
        "--mapping",
        type=Path,
        default=PROJECT_ROOT / "calibration" / "joint_mapping.csv",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_ROOT / "calibration" / "plots" / "static_analysis.md",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        aliases, _ = load_mapping(args.mapping)
        rows = canonicalize_rows(load_log(args.log), aliases)
    except (OSError, LogFormatError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    lines = [
        "# Static log analysis",
        "",
        f"- Log: `{args.log.resolve()}`",
        "- Report-only analysis; no mapping or MuJoCo parameter is changed.",
        "",
        "| Joint | samples | position mean (rad) | position std (rad) | drift (rad) | velocity RMS (rad/s) | torque mean (N·m) | torque RMS (N·m) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    warnings: list[str] = []
    for name in joint_names(rows):
        series = joint_series(rows, name)
        position = series.measured_position
        velocity = series.measured_velocity
        torque = series.measured_torque
        position_finite = position[np.isfinite(position)]
        velocity_finite = velocity[np.isfinite(velocity)]
        torque_finite = torque[np.isfinite(torque)]
        position_mean = float(np.mean(position_finite)) if position_finite.size else float("nan")
        position_std = float(np.std(position_finite)) if position_finite.size else float("nan")
        drift = (
            float(position_finite[-1] - position_finite[0])
            if position_finite.size >= 2
            else float("nan")
        )
        velocity_rms = (
            float(np.sqrt(np.mean(np.square(velocity_finite))))
            if velocity_finite.size
            else float("nan")
        )
        torque_mean = float(np.mean(torque_finite)) if torque_finite.size else float("nan")
        torque_rms = (
            float(np.sqrt(np.mean(np.square(torque_finite))))
            if torque_finite.size
            else float("nan")
        )
        lines.append(
            f"| {name} | {series.timestamp.size} | {_fmt(position_mean)} | "
            f"{_fmt(position_std)} | {_fmt(drift)} | {_fmt(velocity_rms)} | "
            f"{_fmt(torque_mean)} | {_fmt(torque_rms)} |"
        )
        if np.isfinite(position_std) and position_std > 0.01:
            warnings.append(f"{name}: position std {position_std:.4g} rad; log may not be static")
        if np.isfinite(velocity_rms) and velocity_rms > 0.03:
            warnings.append(f"{name}: velocity RMS {velocity_rms:.4g} rad/s; log may not be static")

    imu_time, imu = imu_series(rows)
    lines.extend(["", "## IMU stability", ""])
    if imu_time.size:
        lines.extend(
            [
                "| Signal | component mean | component std |",
                "|---|---|---|",
            ]
        )
        for field in ("imu_quaternion", "imu_gyro", "imu_accel"):
            values = imu[field]
            lines.append(
                f"| {field} | `{np.nanmean(values, axis=0).tolist()}` | "
                f"`{np.nanstd(values, axis=0).tolist()}` |"
            )
        quaternion_norm = np.linalg.norm(imu["imu_quaternion"], axis=1)
        finite_norm = quaternion_norm[np.isfinite(quaternion_norm)]
        if finite_norm.size:
            error = float(np.max(np.abs(finite_norm - 1.0)))
            lines.append("")
            lines.append(f"Maximum quaternion norm error: `{error:.6g}`")
            if error > 0.02:
                warnings.append(f"IMU quaternion norm error reaches {error:.4g}")
    else:
        lines.append("No finite IMU samples were present.")

    lines.extend(["", "## Diagnostic warnings", ""])
    lines.extend([f"- {warning}" for warning in warnings] or ["- None from current thresholds."])
    lines.append("")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(lines), encoding="utf-8")
    print(f"Analyzed {len(joint_names(rows))} joint(s). Report: {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

