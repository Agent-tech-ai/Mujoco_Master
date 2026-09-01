"""Plot and analyze one joint from a real log, optionally against simulation."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from calibration.analysis import compare_joint
from calibration.log_io import (
    LogFormatError,
    canonicalize_rows,
    joint_series,
    load_log,
    load_mapping,
    relative_time,
    safe_filename,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", required=True, type=Path, help="Real or primary CSV log")
    parser.add_argument("--joint", required=True)
    parser.add_argument("--sim", type=Path, help="Optional simulation CSV for aligned comparison")
    parser.add_argument(
        "--mapping",
        type=Path,
        default=PROJECT_ROOT / "calibration" / "joint_mapping.csv",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=PROJECT_ROOT / "calibration" / "plots",
    )
    parser.add_argument("--absolute-time", action="store_true")
    return parser.parse_args()


def _single_plot(name: str, series, path: Path) -> None:
    fields = (
        ("command_position", "Command position", "rad"),
        ("measured_position", "Measured position", "rad"),
        ("measured_velocity", "Measured velocity", "rad/s"),
        ("measured_torque", "Measured torque / effort", "N·m"),
    )
    figure, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    for axis, (field, title, unit) in zip(axes, fields):
        axis.plot(series.timestamp, getattr(series, field), linewidth=1.4)
        axis.set_title(title)
        axis.set_ylabel(unit)
        axis.grid(True, alpha=0.3)
    axes[-1].set_xlabel("time (s)")
    figure.suptitle(name)
    figure.tight_layout()
    figure.savefig(path, dpi=150)
    plt.close(figure)


def _comparison_plot(name: str, timeline: np.ndarray, values: dict[str, np.ndarray], path: Path) -> None:
    fields = (
        ("command_position", "Command position", "rad"),
        ("measured_position", "Measured position", "rad"),
        ("measured_velocity", "Measured velocity", "rad/s"),
        ("measured_torque", "Measured torque / effort", "N·m"),
    )
    figure, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    for axis, (field, title, unit) in zip(axes, fields):
        axis.plot(timeline, values[f"real_{field}"], label="primary/real")
        axis.plot(timeline, values[f"sim_{field}"], label="simulation")
        axis.set_title(title)
        axis.set_ylabel(unit)
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")
    axes[-1].set_xlabel("aligned time (s)")
    figure.suptitle(name)
    figure.tight_layout()
    figure.savefig(path, dpi=150)
    plt.close(figure)


def main() -> int:
    args = parse_args()
    try:
        aliases, limits = load_mapping(args.mapping)
        primary_rows = canonicalize_rows(load_log(args.log), aliases)
        canonical_name = aliases.get(args.joint, args.joint)
        if not args.absolute_time:
            primary_rows = relative_time(primary_rows)
        primary = joint_series(primary_rows, canonical_name)
    except (OSError, LogFormatError, KeyError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = safe_filename(canonical_name)
    plot_path = output_dir / f"single_joint_{stem}.png"
    report_path = output_dir / f"single_joint_{stem}.md"
    lines = [
        f"# Single-joint analysis: {canonical_name}",
        "",
        f"- Primary log: `{args.log.resolve()}`",
        "- Report only; no model parameters were changed.",
        "",
    ]
    if args.sim:
        try:
            sim_rows = canonicalize_rows(load_log(args.sim), aliases)
            if not args.absolute_time:
                sim_rows = relative_time(sim_rows)
            result, timeline, aligned = compare_joint(
                canonical_name,
                primary,
                joint_series(sim_rows, canonical_name),
                limits.get(canonical_name),
            )
        except (OSError, LogFormatError, KeyError) as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return 2
        _comparison_plot(canonical_name, timeline, aligned, plot_path)
        lines.extend(
            [
                f"- Correlation real/sim: `{result.correlation_real_sim:.6g}`",
                f"- Zero-offset candidate: `{result.zero_offset_candidate_rad:+.6g} rad`",
                f"- Position-scale candidate: `{result.position_scale_candidate:.6g}`",
                f"- Real response-delay candidate: `{result.real_response_delay_s:.6g} s`",
                f"- Simulation response-delay candidate: `{result.sim_response_delay_s:.6g} s`",
                "",
                "## Diagnostic flags",
                "",
            ]
        )
        lines.extend([f"- {flag}" for flag in result.flags] or ["- None from current thresholds."])
    else:
        _single_plot(canonical_name, primary, plot_path)
        finite_position = primary.measured_position[np.isfinite(primary.measured_position)]
        lines.extend(
            [
                f"- Samples: `{primary.timestamp.size}`",
                f"- Observed position minimum: `{np.min(finite_position):.6g} rad`" if finite_position.size else "- Observed position minimum: `N/A`",
                f"- Observed position maximum: `{np.max(finite_position):.6g} rad`" if finite_position.size else "- Observed position maximum: `N/A`",
            ]
        )
    lines.extend(["", f"Plot: `{plot_path}`", ""])
    report_path.write_text("\n".join(lines), encoding="utf-8")
    print(f"Report: {report_path}")
    print(f"Plot: {plot_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

