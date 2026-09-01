"""Align real and simulation logs, plot signals, and report mismatch candidates."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from calibration.analysis import compare_joint, markdown_report
from calibration.log_io import (
    LogFormatError,
    canonicalize_rows,
    common_timeline,
    imu_series,
    interpolate,
    joint_names,
    joint_series,
    load_log,
    load_mapping,
    relative_time,
    safe_filename,
)


DEFAULT_MAPPING = PROJECT_ROOT / "calibration" / "joint_mapping.csv"
DEFAULT_PLOTS = PROJECT_ROOT / "calibration" / "plots"


def _plot_joint(name: str, timeline: np.ndarray, aligned: dict[str, np.ndarray], path: Path) -> None:
    fields = (
        ("command_position", "Command position", "rad"),
        ("measured_position", "Measured position", "rad"),
        ("measured_velocity", "Measured velocity", "rad/s"),
        ("measured_torque", "Measured torque / effort", "N·m"),
    )
    figure, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    for axis, (field, title, unit) in zip(axes, fields):
        axis.plot(timeline, aligned[f"real_{field}"], label="real", linewidth=1.5)
        axis.plot(timeline, aligned[f"sim_{field}"], label="simulation", linewidth=1.2)
        axis.set_ylabel(unit)
        axis.set_title(title)
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")
    axes[-1].set_xlabel("aligned time (s)")
    figure.suptitle(name)
    figure.tight_layout()
    figure.savefig(path, dpi=150)
    plt.close(figure)


def _plot_imu_group(
    title: str,
    labels: tuple[str, ...],
    timeline: np.ndarray,
    real_values: np.ndarray,
    sim_values: np.ndarray,
    ylabel: str,
    path: Path,
) -> None:
    figure, axes = plt.subplots(len(labels), 1, figsize=(11, 2.3 * len(labels)), sharex=True)
    axes = np.atleast_1d(axes)
    for index, (axis, label) in enumerate(zip(axes, labels)):
        axis.plot(timeline, real_values[:, index], label="real", linewidth=1.4)
        axis.plot(timeline, sim_values[:, index], label="simulation", linewidth=1.2)
        axis.set_ylabel(f"{label} ({ylabel})" if ylabel else label)
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")
    axes[-1].set_xlabel("aligned time (s)")
    figure.suptitle(title)
    figure.tight_layout()
    figure.savefig(path, dpi=150)
    plt.close(figure)


def _plot_imu(real_rows: list, sim_rows: list, output_dir: Path) -> list[Path]:
    real_time, real_data = imu_series(real_rows)
    sim_time, sim_data = imu_series(sim_rows)
    if real_time.size < 2 or sim_time.size < 2:
        return []
    timeline = common_timeline(real_time, sim_time)
    generated: list[Path] = []
    specifications = (
        ("imu_quaternion", "IMU orientation quaternion", ("qw", "qx", "qy", "qz"), ""),
        ("imu_gyro", "IMU angular velocity", ("x", "y", "z"), "rad/s"),
        ("imu_accel", "IMU linear acceleration", ("x", "y", "z"), "m/s²"),
    )
    for field, title, labels, unit in specifications:
        real_values = interpolate(real_time, real_data[field], timeline)
        sim_values = interpolate(sim_time, sim_data[field], timeline)
        if not np.any(np.isfinite(real_values)) or not np.any(np.isfinite(sim_values)):
            continue
        path = output_dir / f"{field}.png"
        _plot_imu_group(title, labels, timeline, real_values, sim_values, unit, path)
        generated.append(path)
    return generated


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--real", required=True, type=Path)
    parser.add_argument("--sim", required=True, type=Path)
    parser.add_argument("--mapping", type=Path, default=DEFAULT_MAPPING)
    parser.add_argument("--plots", type=Path, default=DEFAULT_PLOTS)
    parser.add_argument("--report", type=Path, help="Default: <plots>/comparison_report.md")
    parser.add_argument(
        "--time-mode",
        choices=("relative", "absolute"),
        default="relative",
        help="Relative subtracts each log start time before overlap alignment.",
    )
    parser.add_argument("--joint", action="append", help="Limit output to selected joint(s).")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        aliases, limits = load_mapping(args.mapping)
        real_rows = canonicalize_rows(load_log(args.real), aliases)
        sim_rows = canonicalize_rows(load_log(args.sim), aliases)
        if args.time_mode == "relative":
            real_rows = relative_time(real_rows)
            sim_rows = relative_time(sim_rows)
    except (OSError, LogFormatError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    real_names = set(joint_names(real_rows))
    sim_names = set(joint_names(sim_rows))
    common_names = sorted(real_names & sim_names)
    if args.joint:
        requested = set(args.joint)
        common_names = [name for name in common_names if name in requested]
        absent = sorted(requested - set(common_names))
        if absent:
            print(f"WARNING: requested joints absent from one or both logs: {absent}")
    if not common_names:
        print("ERROR: logs have no joint names in common", file=sys.stderr)
        return 2

    output_dir = args.plots.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    results = []
    generated: list[Path] = []
    for name in common_names:
        try:
            result, timeline, aligned = compare_joint(
                name,
                joint_series(real_rows, name),
                joint_series(sim_rows, name),
                limits.get(name),
            )
        except LogFormatError as exc:
            print(f"WARNING: skipping {name}: {exc}")
            continue
        results.append(result)
        plot_path = output_dir / f"joint_{safe_filename(name)}.png"
        _plot_joint(name, timeline, aligned, plot_path)
        generated.append(plot_path)

    if not results:
        print("ERROR: no common joint had overlapping timestamps", file=sys.stderr)
        return 2
    generated.extend(_plot_imu(real_rows, sim_rows, output_dir))
    report_path = (args.report or output_dir / "comparison_report.md").resolve()
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report = markdown_report(
        str(args.real.resolve()),
        str(args.sim.resolve()),
        args.time_mode,
        results,
        sorted(sim_names - real_names),
        sorted(real_names - sim_names),
    )
    report_path.write_text(report, encoding="utf-8")
    json_path = report_path.with_suffix(".json")
    json_path.write_text(
        json.dumps([result.as_dict() for result in results], ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    flagged = sum(bool(result.flags) for result in results)
    print(f"Compared {len(results)} joint(s); {flagged} joint(s) have diagnostic flags.")
    print(f"Report: {report_path}")
    print(f"Metrics: {json_path}")
    print(f"Plots: {len(generated)} file(s) in {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

