"""Numerical diagnostics shared by calibration command-line tools."""

from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np

from calibration.log_io import JointSeries, common_timeline, interpolate


@dataclass(frozen=True)
class JointComparison:
    joint_name: str
    samples: int
    correlation_real_sim: float
    zero_offset_candidate_rad: float
    position_scale_candidate: float
    real_observed_min_rad: float
    real_observed_max_rad: float
    sim_observed_min_rad: float
    sim_observed_max_rad: float
    real_response_delay_s: float
    sim_response_delay_s: float
    response_delay_difference_s: float
    flags: tuple[str, ...]

    def as_dict(self) -> dict[str, object]:
        return asdict(self)


def _finite_pair(first: np.ndarray, second: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    finite = np.isfinite(first) & np.isfinite(second)
    return first[finite], second[finite]


def correlation(first: np.ndarray, second: np.ndarray) -> float:
    first, second = _finite_pair(first, second)
    if first.size < 4 or np.std(first) < 1e-8 or np.std(second) < 1e-8:
        return float("nan")
    return float(np.corrcoef(first, second)[0, 1])


def response_delay(command: np.ndarray, measured: np.ndarray, dt: float) -> float:
    """Estimate non-negative command-to-measurement delay using correlation."""

    command, measured = _finite_pair(command, measured)
    if command.size < 8 or np.std(command) < 1e-5 or np.std(measured) < 1e-5:
        return float("nan")
    max_shift = min(command.size // 3, max(1, int(round(1.0 / max(dt, 1e-6)))))
    best_shift = 0
    best_score = -np.inf
    for shift in range(max_shift + 1):
        left = command[: command.size - shift] if shift else command
        right = measured[shift:] if shift else measured
        score = correlation(left, right)
        if np.isfinite(score) and abs(score) > best_score:
            best_score = abs(score)
            best_shift = shift
    return float(best_shift * dt) if np.isfinite(best_score) else float("nan")


def compare_joint(
    name: str,
    real: JointSeries,
    sim: JointSeries,
    limit: tuple[float, float] | None = None,
) -> tuple[JointComparison, np.ndarray, dict[str, np.ndarray]]:
    timeline = common_timeline(real.timestamp, sim.timestamp)
    aligned: dict[str, np.ndarray] = {}
    for source_name, series in (("real", real), ("sim", sim)):
        for field in (
            "command_position",
            "measured_position",
            "measured_velocity",
            "measured_torque",
        ):
            aligned[f"{source_name}_{field}"] = interpolate(
                series.timestamp, getattr(series, field), timeline
            )

    real_position = aligned["real_measured_position"]
    sim_position = aligned["sim_measured_position"]
    real_finite, sim_finite = _finite_pair(real_position, sim_position)
    flags: list[str] = []

    if real_finite.size:
        offset = float(np.median(real_finite - sim_finite))
        real_min = float(np.min(real_finite))
        real_max = float(np.max(real_finite))
        sim_min = float(np.min(sim_finite))
        sim_max = float(np.max(sim_finite))
    else:
        offset = real_min = real_max = sim_min = sim_max = float("nan")

    corr = correlation(real_position, sim_position)
    centered_sim = sim_finite - np.mean(sim_finite) if sim_finite.size else sim_finite
    centered_real = real_finite - np.mean(real_finite) if real_finite.size else real_finite
    denominator = float(np.dot(centered_sim, centered_sim))
    scale = (
        float(np.dot(centered_sim, centered_real) / denominator)
        if denominator > 1e-10
        else float("nan")
    )

    dt = float(np.median(np.diff(timeline)))
    real_delay = response_delay(
        aligned["real_command_position"], real_position, dt
    )
    sim_delay = response_delay(aligned["sim_command_position"], sim_position, dt)
    delay_difference = (
        real_delay - sim_delay
        if np.isfinite(real_delay) and np.isfinite(sim_delay)
        else float("nan")
    )

    real_command_corr = correlation(aligned["real_command_position"], real_position)
    if (np.isfinite(corr) and corr < -0.7) or (
        np.isfinite(real_command_corr) and real_command_corr < -0.7
    ):
        flags.append(
            "POSSIBLE_SIGN_MISMATCH: strong negative position correlation; verify hardware sign"
        )
    if np.isfinite(offset) and abs(offset) > 0.03:
        flags.append(
            f"POSSIBLE_ZERO_OFFSET_MISMATCH: median real-sim offset {offset:+.4f} rad"
        )
    if np.isfinite(scale) and abs(scale) > 0.05 and not 0.8 <= abs(scale) <= 1.2:
        flags.append(
            f"POSSIBLE_POSITION_SCALE_MISMATCH: fitted real/sim scale {scale:.3f}"
        )
    if np.isfinite(delay_difference) and abs(delay_difference) > max(0.05, 3 * dt):
        flags.append(
            "POSSIBLE_RESPONSE_DELAY: real and simulation command-response delays differ by "
            f"{delay_difference:+.4f} s"
        )

    real_span = real_max - real_min
    sim_span = sim_max - sim_min
    if (
        np.isfinite(real_span)
        and np.isfinite(sim_span)
        and max(real_span, sim_span) > 0.05
        and min(real_span, sim_span) / max(real_span, sim_span) < 0.75
    ):
        flags.append(
            "POSSIBLE_JOINT_RANGE_MISMATCH: observed real/sim position spans differ by more than 25%"
        )
    if limit is not None and np.isfinite(real_min):
        lower, upper = limit
        if real_min < lower - 0.01 or real_max > upper + 0.01:
            flags.append(
                "POSSIBLE_JOINT_RANGE_MISMATCH: real observation lies outside current MuJoCo range "
                f"[{lower:.4f}, {upper:.4f}] rad"
            )
        if sim_min < lower - 0.01 or sim_max > upper + 0.01:
            flags.append(
                "SIM_RANGE_VIOLATION: simulation observation lies outside configured range "
                f"[{lower:.4f}, {upper:.4f}] rad"
            )

    result = JointComparison(
        joint_name=name,
        samples=int(timeline.size),
        correlation_real_sim=corr,
        zero_offset_candidate_rad=offset,
        position_scale_candidate=scale,
        real_observed_min_rad=real_min,
        real_observed_max_rad=real_max,
        sim_observed_min_rad=sim_min,
        sim_observed_max_rad=sim_max,
        real_response_delay_s=real_delay,
        sim_response_delay_s=sim_delay,
        response_delay_difference_s=delay_difference,
        flags=tuple(flags),
    )
    return result, timeline, aligned


def markdown_report(
    real_path: str,
    sim_path: str,
    time_mode: str,
    results: list[JointComparison],
    missing_in_real: list[str],
    missing_in_sim: list[str],
) -> str:
    lines = [
        "# Real ↔ MuJoCo comparison report",
        "",
        f"- Real log: `{real_path}`",
        f"- Simulation log: `{sim_path}`",
        f"- Timestamp mode: `{time_mode}`",
        "- This report is diagnostic only. It does not modify mapping or MJCF parameters.",
        "",
    ]
    if missing_in_real:
        lines.append(f"- Only in simulation: `{', '.join(missing_in_real)}`")
    if missing_in_sim:
        lines.append(f"- Only in real log: `{', '.join(missing_in_sim)}`")
    if missing_in_real or missing_in_sim:
        lines.append("")
    lines.extend(
        [
            "| Joint | corr(real, sim) | offset candidate (rad) | scale candidate | real delay (s) | sim delay (s) | flags |",
            "|---|---:|---:|---:|---:|---:|---|",
        ]
    )
    for result in results:
        flags = "<br>".join(result.flags) if result.flags else "none"
        lines.append(
            f"| {result.joint_name} | {result.correlation_real_sim:.4g} | "
            f"{result.zero_offset_candidate_rad:+.5g} | "
            f"{result.position_scale_candidate:.5g} | "
            f"{result.real_response_delay_s:.5g} | {result.sim_response_delay_s:.5g} | {flags} |"
        )
    lines.extend(
        [
            "",
            "## Interpretation guardrails",
            "",
            "Flags are candidates based on signal correlation, offsets, spans, and cross-correlation delay. "
            "They are not calibrated values. Confirm them against robot zeroing, encoder conventions, "
            "clock provenance, controller mode, and repeated experiments before editing MuJoCo.",
            "",
        ]
    )
    return "\n".join(lines)
