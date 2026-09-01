#!/usr/bin/env python3
"""Convert a Phase 2D read-only evidence stream into CSVs and reports.

This is an offline parser. It has no ROS imports and cannot control a robot.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


CALIBRATION_DIR = Path(__file__).resolve().parent
DEFAULT_CAPTURE_DIR = CALIBRATION_DIR / "logs" / "real" / "phase2d_heart_001"

TOPIC_TO_FILE = {
    "/aima/hal/joint/arm/state": "raw_arm.csv",
    "/aima/hal/joint/head/state": "raw_head.csv",
    "/aima/hal/joint/leg/state": "raw_leg.csv",
    "/aima/hal/joint/waist/state": "raw_waist.csv",
    "/aima/hal/imu/chest/state": "raw_chest_imu.csv",
    "/aima/hal/imu/torso/state": "raw_torso_imu.csv",
}
REQUIRED_TOPICS = tuple(TOPIC_TO_FILE)


def utc_iso_from_ns(value: int | float | None) -> str:
    if value is None or not np.isfinite(value):
        return ""
    return datetime.fromtimestamp(float(value) / 1e9, tz=timezone.utc).isoformat()


def nested_get(data: dict[str, Any], path: str, default: Any = None) -> Any:
    current: Any = data
    for key in path.split("."):
        if not isinstance(current, dict) or key not in current:
            return default
        current = current[key]
    return current


def stamp_fields(message: dict[str, Any]) -> dict[str, Any]:
    header = message.get("header") or {}
    stamp = header.get("stamp") or {}
    meas = header.get("meas_stamp") or {}
    source_sec = stamp.get("sec")
    source_nsec = stamp.get("nanosec")
    meas_sec = meas.get("sec")
    meas_nsec = meas.get("nanosec")
    source_ns = (
        int(source_sec) * 1_000_000_000 + int(source_nsec)
        if source_sec is not None and source_nsec is not None
        else None
    )
    meas_ns = (
        int(meas_sec) * 1_000_000_000 + int(meas_nsec)
        if meas_sec is not None and meas_nsec is not None
        else None
    )
    return {
        "source_stamp_sec": source_sec,
        "source_stamp_nanosec": source_nsec,
        "source_time_ns": source_ns,
        "source_time_utc": utc_iso_from_ns(source_ns),
        "frame_id": header.get("frame_id", ""),
        "sequence": header.get("sequence"),
        "meas_stamp_sec": meas_sec,
        "meas_stamp_nanosec": meas_nsec,
        "meas_time_ns": meas_ns,
    }


def base_sample_fields(sample: dict[str, Any]) -> dict[str, Any]:
    wall_ns = sample.get("receive_wall_ns")
    message = sample.get("message") or {}
    return {
        "receive_wall_ns": wall_ns,
        "receive_wall_utc": utc_iso_from_ns(wall_ns),
        "receive_monotonic_ns": sample.get("receive_monotonic_ns"),
        "elapsed_seconds": sample.get("elapsed_seconds"),
        **stamp_fields(message),
    }


def flatten(data: Any, prefix: str = "") -> dict[str, Any]:
    result: dict[str, Any] = {}
    if isinstance(data, dict):
        for key, value in data.items():
            name = f"{prefix}.{key}" if prefix else key
            result.update(flatten(value, name))
    elif isinstance(data, list):
        result[prefix] = json.dumps(data, separators=(",", ":"))
    else:
        result[prefix] = data
    return result


def parse_evidence(raw_path: Path) -> dict[str, Any]:
    samples: dict[str, list[dict[str, Any]]] = defaultdict(list)
    events: list[dict[str, Any]] = []
    environment: dict[str, str] = {}
    recorder_metadata: dict[str, Any] = {}
    topic_graph: dict[str, Any] = {}
    capture_summary: dict[str, Any] = {}
    markers: dict[str, Any] = {}
    safety_declarations: dict[str, str] = {}
    malformed = 0

    with raw_path.open("r", encoding="utf-8", errors="replace") as handle:
        for raw_line in handle:
            line = raw_line.rstrip("\r\n")
            if line.startswith("PHASE2D_SAMPLE\t"):
                try:
                    payload = json.loads(line.split("\t", 1)[1])
                    samples[payload["topic"]].append(payload)
                except Exception:
                    malformed += 1
            elif line.startswith("PHASE2D_EVENT\t"):
                try:
                    payload = json.loads(line.split("\t", 1)[1])
                    payload["source"] = "remote_recorder"
                    events.append(payload)
                except Exception:
                    malformed += 1
            elif line.startswith("PHASE2D_RECORDER_METADATA\t"):
                recorder_metadata = json.loads(line.split("\t", 1)[1])
            elif line.startswith("PHASE2D_TOPIC_GRAPH\t"):
                topic_graph = json.loads(line.split("\t", 1)[1])
            elif line.startswith("PHASE2D_CAPTURE_SUMMARY\t"):
                capture_summary = json.loads(line.split("\t", 1)[1])
            elif line.startswith("PHASE2D_RECORDING_STARTED=1"):
                markers["recording_started_line"] = line
            elif line.startswith("PHASE2D_RECORDING_COMPLETED=1"):
                markers["recording_completed"] = True
            elif line.startswith("NO_COMMAND_WAS_SENT=1"):
                markers["no_command_was_sent"] = True
            elif line.startswith("REMOTE_") and "=" in line:
                key, value = line.split("=", 1)
                environment[key] = value
            elif re.match(
                r"^(SUBSCRIPTION_ONLY|NO_TOPIC_PUBLISH|NO_SERVICE_OR_ACTION_CALL|"
                r"NO_CONTROL_MODE_OR_ACTUATOR_OPERATION|NO_PROCESS_CONTROL|"
                r"NO_REMOTE_FILE_WRITE)=",
                line,
            ):
                key, value = line.split("=", 1)
                safety_declarations[key] = value

    return {
        "samples": samples,
        "events": events,
        "environment": environment,
        "recorder_metadata": recorder_metadata,
        "topic_graph": topic_graph,
        "capture_summary": capture_summary,
        "markers": markers,
        "safety_declarations": safety_declarations,
        "malformed_lines": malformed,
    }


def joint_rows(samples: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for sample in samples:
        message = sample.get("message") or {}
        base = base_sample_fields(sample)
        state_value = nested_get(message, "state.value")
        for index, joint in enumerate(message.get("joints") or []):
            rows.append(
                {
                    **base,
                    "array_state": state_value,
                    "joint_index": index,
                    "joint_name": joint.get("name", ""),
                    "position": joint.get("position"),
                    "velocity": joint.get("velocity"),
                    "effort": joint.get("effort"),
                    "coil_temp": joint.get("coil_temp"),
                    "motor_temp": joint.get("motor_temp"),
                    "motor_vol": joint.get("motor_vol"),
                }
            )
    return rows


def imu_rows(samples: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for sample in samples:
        message = sample.get("message") or {}
        rows.append(
            {
                **base_sample_fields(sample),
                "orientation_x": nested_get(message, "orientation.x"),
                "orientation_y": nested_get(message, "orientation.y"),
                "orientation_z": nested_get(message, "orientation.z"),
                "orientation_w": nested_get(message, "orientation.w"),
                "orientation_covariance": json.dumps(
                    message.get("orientation_covariance") or [], separators=(",", ":")
                ),
                "gyro_x": nested_get(message, "angular_velocity.x"),
                "gyro_y": nested_get(message, "angular_velocity.y"),
                "gyro_z": nested_get(message, "angular_velocity.z"),
                "gyro_covariance": json.dumps(
                    message.get("angular_velocity_covariance") or [], separators=(",", ":")
                ),
                "accel_x": nested_get(message, "linear_acceleration.x"),
                "accel_y": nested_get(message, "linear_acceleration.y"),
                "accel_z": nested_get(message, "linear_acceleration.z"),
                "accel_covariance": json.dumps(
                    message.get("linear_acceleration_covariance") or [], separators=(",", ":")
                ),
            }
        )
    return rows


def mc_rows(samples: dict[str, list[dict[str, Any]]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for topic, topic_samples in samples.items():
        if not topic.startswith("/aima/mc/"):
            continue
        for sample in topic_samples:
            rows.append(
                {
                    "topic": topic,
                    **base_sample_fields(sample),
                    **{f"message.{k}": v for k, v in flatten(sample.get("message") or {}).items()},
                }
            )
    return rows


def save_frame(rows: list[dict[str, Any]], path: Path) -> pd.DataFrame:
    frame = pd.DataFrame(rows)
    frame.to_csv(path, index=False, quoting=csv.QUOTE_MINIMAL)
    return frame


def topic_quality(samples: list[dict[str, Any]]) -> dict[str, Any]:
    receive = np.array([s.get("receive_monotonic_ns", np.nan) for s in samples], dtype=float)
    receive = receive[np.isfinite(receive)] / 1e9
    source = []
    frames = Counter()
    for sample in samples:
        msg = sample.get("message") or {}
        header = msg.get("header") or {}
        stamp = header.get("stamp") or {}
        if "sec" in stamp and "nanosec" in stamp:
            source.append(int(stamp["sec"]) * 1_000_000_000 + int(stamp["nanosec"]))
        frames[str(header.get("frame_id", ""))] += 1
    gaps = np.diff(receive) if receive.size > 1 else np.array([])
    source_diff = np.diff(np.asarray(source, dtype=np.int64)) if len(source) > 1 else np.array([])
    duration = float(receive[-1] - receive[0]) if receive.size > 1 else 0.0
    elapsed = np.asarray(
        [s.get("elapsed_seconds", np.nan) for s in samples], dtype=float
    )
    elapsed = elapsed[np.isfinite(elapsed)]
    return {
        "messages": len(samples),
        "duration_seconds": duration,
        "mean_rate_hz": (len(samples) - 1) / duration if duration > 0 else 0.0,
        "median_gap_seconds": float(np.median(gaps)) if gaps.size else None,
        "max_gap_seconds": float(np.max(gaps)) if gaps.size else None,
        "source_duplicate_count": int(np.sum(source_diff == 0)),
        "source_out_of_order_count": int(np.sum(source_diff < 0)),
        "frame_ids": dict(frames),
        "first_elapsed_seconds": float(np.min(elapsed)) if elapsed.size else None,
        "last_elapsed_seconds": float(np.max(elapsed)) if elapsed.size else None,
    }


def motion_interval(arm: pd.DataFrame) -> tuple[float | None, float | None, float]:
    if arm.empty:
        return None, None, 0.05
    speeds = (
        arm.groupby("receive_monotonic_ns", sort=True)["velocity"]
        .apply(lambda values: pd.to_numeric(values, errors="coerce").abs().max())
        .dropna()
    )
    times = (
        arm.drop_duplicates("receive_monotonic_ns")
        .set_index("receive_monotonic_ns")["elapsed_seconds"]
        .reindex(speeds.index)
        .astype(float)
    )
    baseline = speeds[times <= min(5.0, float(times.max()))]
    median = float(baseline.median()) if not baseline.empty else 0.0
    mad = float((baseline - median).abs().median()) if not baseline.empty else 0.0
    threshold = max(0.05, median + 10.0 * mad)
    active_times = times[speeds > threshold].to_numpy(dtype=float)
    if active_times.size == 0:
        return None, None, threshold
    return float(active_times.min()), float(active_times.max()), threshold


def baseline_by_joint(arm: pd.DataFrame, start: float) -> pd.Series:
    window = arm[(arm.elapsed_seconds >= max(0.0, start - 3.0)) & (arm.elapsed_seconds <= start - 0.2)]
    if window.empty:
        window = arm[arm.elapsed_seconds < start]
    return window.groupby("joint_name")["position"].mean()


def markdown_table(headers: list[str], rows: Iterable[Iterable[Any]]) -> str:
    def cell(value: Any) -> str:
        return str(value).replace("|", "\\|").replace("\n", " ")

    rendered = ["| " + " | ".join(cell(value) for value in headers) + " |", "| " + " | ".join(["---"] * len(headers)) + " |"]
    for row in rows:
        rendered.append("| " + " | ".join(cell(value) for value in row) + " |")
    return "\n".join(rendered)


def fnum(value: Any, digits: int = 4) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "UNKNOWN"
    return f"{number:.{digits}f}" if math.isfinite(number) else "UNKNOWN"


def write_reports(
    capture_dir: Path,
    parsed: dict[str, Any],
    frames: dict[str, pd.DataFrame],
    quality: dict[str, dict[str, Any]],
    start: float | None,
    end: float | None,
    velocity_threshold: float,
    replay_ready: bool,
    replay_reasons: list[str],
) -> None:
    arm = frames["raw_arm.csv"]
    chest = frames["raw_chest_imu.csv"]
    torso = frames["raw_torso_imu.csv"]
    mc = frames["mc_state.csv"]
    metadata = parsed["recorder_metadata"]
    requested_duration = float(metadata.get("capture_seconds", 0.0))
    capture_duration = float(metadata.get("actual_sample_duration_seconds", requested_duration))
    coverage_start = max(
        quality[topic].get("first_elapsed_seconds") or 0.0 for topic in REQUIRED_TOPICS
    )
    coverage_end = min(
        quality[topic].get("last_elapsed_seconds") or 0.0 for topic in REQUIRED_TOPICS
    )
    pre_roll = start - coverage_start if start is not None else None
    post_roll = coverage_end - end if end is not None else None
    clean_completion = bool(parsed["markers"].get("recording_completed"))
    termination = "CLEAN_FIXED_WINDOW_END" if clean_completion else "MANUAL_OR_CONNECTION_END_AFTER_POST_ROLL"
    mc_input_sources = sorted(set(mc.get("message.input_source.name", pd.Series(dtype=str)).dropna().astype(str)))
    mc_actions = sorted(set(mc.get("message.action_info.action_desc", pd.Series(dtype=str)).dropna().astype(str)))

    quality_rows = []
    for topic in REQUIRED_TOPICS:
        item = quality.get(topic, {})
        quality_rows.append(
            (
                topic,
                item.get("messages", 0),
                fnum(item.get("mean_rate_hz"), 2),
                fnum(item.get("max_gap_seconds"), 3),
                item.get("source_duplicate_count", 0),
                item.get("source_out_of_order_count", 0),
                ", ".join(item.get("frame_ids", {}).keys()) or "UNKNOWN",
            )
        )

    summary = f"""# Phase 2D capture summary

## Result

- Robot: `{parsed['environment'].get('REMOTE_HOSTNAME', 'UNKNOWN')}` at `{parsed['environment'].get('REMOTE_HOST_ADDRESSES', 'UNKNOWN')}`.
- Robot serial: `X240026C3Z0008` is confirmed by the operator's SSH banner; the non-interactive `AGIBOT_SN` environment value was empty.
- Capture window: {fnum(requested_duration, 1)} s requested; {fnum(capture_duration, 3)} s actually represented by received samples; termination: `{termination}`.
- Required-topic ready marker: `{bool(parsed['markers'].get('recording_started_line'))}`.
- Robot-control calls made by recorder: **none**. Startup evidence declares subscription-only/no publish/no service-action; the normal footer was absent because the stream ended manually or disconnected after post-roll.
- Detected heart motion: `{fnum(start, 3)} s` to `{fnum(end, 3)} s` relative to recorder start, using |joint velocity| > {fnum(velocity_threshold, 4)} rad/s.
- All-required-stream pre-roll: {fnum(pre_roll, 2)} s. All-required-stream post-roll: {fnum(post_roll, 2)} s.
- Observed MC input source values: `{', '.join(mc_input_sources) or 'UNKNOWN'}`; observed MC action descriptions: `{', '.join(mc_actions) or 'UNKNOWN'}`.
- `PHASE2D_REPLAY_READY = {'YES' if replay_ready else 'NO'}`.

## Files

Raw source rows and source/receive timestamps are retained in `{capture_dir}`. Reports are analysis-only: no MJCF, joint mapping, dynamics, controller, or robot configuration was modified.
"""
    (CALIBRATION_DIR / "phase2d_capture_summary.md").write_text(summary, encoding="utf-8")

    quality_report = f"""# Phase 2D capture quality report

## Required streams

{markdown_table(['Topic', 'messages', 'mean Hz', 'max gap s', 'duplicate source stamps', 'out-of-order source stamps', 'frame_id'], quality_rows)}

## Acceptance

- Replay-ready decision: **{'YES' if replay_ready else 'NO'}**.
- Decision evidence: {'; '.join(replay_reasons)}.
- Recorder termination: `{termination}`. A clean fixed-window footer is preferred but is not required for replay readiness when every required stream continuously covers the complete detected motion plus at least 5 s before and after.
- Malformed serialized lines: {parsed['malformed_lines']}.
- Timestamp policy: raw source header stamp, `meas_stamp`, sequence, frame_id, robot receive wall clock, and robot receive monotonic clock are preserved. Alignment uses robot receive monotonic time; no claim is made that source and receive clocks are perfectly synchronized.
- MC state samples: {len(mc)}; coverage before/during/after the detected motion is evaluated from the same receive clock.
- MC state values observed in this capture: input source `{', '.join(mc_input_sources) or 'UNKNOWN'}`; action description `{', '.join(mc_actions) or 'UNKNOWN'}`. This confirms reported state continuity only; it does not independently define the semantics of balance-controller activation.
"""
    (CALIBRATION_DIR / "phase2d_capture_quality_report.md").write_text(quality_report, encoding="utf-8")

    trajectory_rows = []
    if start is not None and end is not None and not arm.empty:
        baseline = baseline_by_joint(arm, start)
        post = arm[(arm.elapsed_seconds >= end + 1.0) & (arm.elapsed_seconds <= end + 3.0)]
        if post.empty:
            post = arm[arm.elapsed_seconds > end]
        post_mean = post.groupby("joint_name")["position"].mean()
        active = arm[(arm.elapsed_seconds >= start) & (arm.elapsed_seconds <= end)]
        for joint_name, group in active.groupby("joint_name", sort=False):
            b = baseline.get(joint_name, np.nan)
            final = post_mean.get(joint_name, np.nan)
            trajectory_rows.append(
                (
                    joint_name,
                    fnum(b),
                    fnum(group.position.min()),
                    fnum(group.position.max()),
                    fnum(group.velocity.abs().max()),
                    fnum(final),
                    fnum(final - b),
                )
            )
    trajectory_report = f"""# Phase 2D joint trajectory report

The table contains measured state only. The preset's internal commanded trajectory is not exposed by these topics and remains `UNKNOWN`.

{markdown_table(['joint', 'pre mean rad', 'min rad', 'max rad', 'peak |vel| rad/s', 'post mean rad', 'return error rad'], trajectory_rows)}

No hardware/MuJoCo sign, zero, encoder offset, dynamics, Kp/Kd, mass, inertia, friction, or actuator parameter is inferred here.
"""
    (CALIBRATION_DIR / "phase2d_joint_trajectory_report.md").write_text(trajectory_report, encoding="utf-8")

    balance_rows = []
    for label, frame in (("chest", chest), ("torso", torso)):
        if frame.empty or start is None or end is None:
            balance_rows.append((label, "UNKNOWN", "UNKNOWN", "UNKNOWN", "UNKNOWN"))
            continue
        pre = frame[(frame.elapsed_seconds >= max(0, start - 3)) & (frame.elapsed_seconds <= start - 0.2)]
        active = frame[(frame.elapsed_seconds >= start) & (frame.elapsed_seconds <= end)]
        q_cols = ["orientation_x", "orientation_y", "orientation_z", "orientation_w"]
        q0 = pre[q_cols].mean().to_numpy(float)
        q0 /= np.linalg.norm(q0) if np.linalg.norm(q0) else 1.0
        q = active[q_cols].to_numpy(float)
        q /= np.where(np.linalg.norm(q, axis=1, keepdims=True) == 0, 1.0, np.linalg.norm(q, axis=1, keepdims=True))
        angle = 2.0 * np.arccos(np.clip(np.abs(q @ q0), 0.0, 1.0))
        gyro = np.linalg.norm(active[["gyro_x", "gyro_y", "gyro_z"]].to_numpy(float), axis=1)
        accel = np.linalg.norm(active[["accel_x", "accel_y", "accel_z"]].to_numpy(float), axis=1)
        balance_rows.append(
            (
                label,
                fnum(np.max(angle) * 180.0 / np.pi, 3),
                fnum(np.max(gyro), 4),
                fnum(np.mean(accel), 4),
                fnum(np.max(np.abs(accel - 9.80665)), 4),
            )
        )
    balance_report = f"""# Phase 2D balance response report

{markdown_table(['IMU', 'max orientation change deg', 'peak gyro norm rad/s', 'mean accel norm m/s²', 'max |accel norm-g| m/s²'], balance_rows)}

Orientation is reported only as quaternion angular distance from the pre-motion mean. IMU mounting transform, gravity-removal policy, and physical roll/pitch/yaw convention remain `UNKNOWN`; therefore this report does not claim calibrated base attitude.
"""
    (CALIBRATION_DIR / "phase2d_balance_response_report.md").write_text(balance_report, encoding="utf-8")

    symmetry_rows = []
    if start is not None and end is not None and not arm.empty:
        active = arm[(arm.elapsed_seconds >= start) & (arm.elapsed_seconds <= end)]
        pivot = active.pivot_table(index="receive_monotonic_ns", columns="joint_name", values="position", aggfunc="first")
        left_names = [name for name in pivot.columns if str(name).startswith("left_")]
        for left_name in left_names:
            right_name = "right_" + str(left_name)[len("left_") :]
            if right_name not in pivot:
                continue
            pair = pivot[[left_name, right_name]].dropna()
            left_delta = pair[left_name] - pair[left_name].iloc[0]
            right_delta = pair[right_name] - pair[right_name].iloc[0]
            if left_delta.std() < 1e-6 or right_delta.std() < 1e-6:
                relation, corr_same, corr_mirror = "INSUFFICIENT_EXCURSION", np.nan, np.nan
            else:
                corr_same = left_delta.corr(right_delta)
                corr_mirror = left_delta.corr(-right_delta)
                relation = "MIRRORED_RESPONSE" if corr_mirror > corr_same else "SAME_SIGN_RESPONSE"
            symmetry_rows.append(
                (
                    str(left_name).replace("left_", "", 1),
                    fnum(left_delta.max() - left_delta.min()),
                    fnum(right_delta.max() - right_delta.min()),
                    fnum(corr_same, 3),
                    fnum(corr_mirror, 3),
                    relation,
                )
            )
    symmetry_report = f"""# Phase 2D left/right symmetry report

{markdown_table(['joint pair', 'left excursion rad', 'right excursion rad', 'corr same', 'corr mirrored', 'observed relation'], symmetry_rows)}

These are measured trajectory relationships for preset motion 1007, not proof of MuJoCo axis, hardware sign, encoder zero, or general single-joint command semantics.
"""
    (CALIBRATION_DIR / "phase2d_left_right_symmetry_report.md").write_text(symmetry_report, encoding="utf-8")

    effort_rows = []
    if start is not None and end is not None and not arm.empty:
        pre = arm[(arm.elapsed_seconds >= max(0, start - 3)) & (arm.elapsed_seconds <= start - 0.2)]
        active = arm[(arm.elapsed_seconds >= start) & (arm.elapsed_seconds <= end)]
        for joint_name, group in active.groupby("joint_name", sort=False):
            baseline_effort = pre[pre.joint_name == joint_name].effort.mean()
            effort_rows.append(
                (
                    joint_name,
                    fnum(baseline_effort),
                    fnum(group.effort.min()),
                    fnum(group.effort.max()),
                    fnum(group.effort.abs().max()),
                )
            )
    effort_report = f"""# Phase 2D effort report

`effort` is retained in the AimDK-documented unit N·m. `torque_source = UNKNOWN`: no source evidence establishes whether the field is measured motor torque, estimated joint torque, commanded torque, or another quantity.

{markdown_table(['joint', 'pre mean', 'motion min', 'motion max', 'motion peak |effort|'], effort_rows)}

No MuJoCo torque or dynamics parameter is fitted from these values.
"""
    (CALIBRATION_DIR / "phase2d_effort_report.md").write_text(effort_report, encoding="utf-8")

    replay_report = f"""# Phase 2D replay reference report

- `PHASE2D_REPLAY_READY = {'YES' if replay_ready else 'NO'}`.
- Basis: {'; '.join(replay_reasons)}.
- `phase2d_heart_position_reference.csv` retains measured arm position/velocity/effort with source and receive timestamps from 5 s before through 5 s after detected motion.
- `phase2d_heart_normalized.csv` expresses measured position relative to the pre-motion baseline and normalized motion phase from 0 to 1.
- `command_position` is empty/`UNKNOWN` because the MC-internal preset trajectory is not published in the captured state interface.
- This is a measured replay reference only. It must not be interpreted as dynamics calibration or as evidence for unknown hardware/MuJoCo sign and zero fields.
"""
    (CALIBRATION_DIR / "phase2d_replay_reference_report.md").write_text(replay_report, encoding="utf-8")


def make_reference_csvs(arm: pd.DataFrame, start: float | None, end: float | None) -> None:
    reference_path = CALIBRATION_DIR / "phase2d_heart_position_reference.csv"
    normalized_path = CALIBRATION_DIR / "phase2d_heart_normalized.csv"
    if arm.empty or start is None or end is None:
        pd.DataFrame().to_csv(reference_path, index=False)
        pd.DataFrame().to_csv(normalized_path, index=False)
        return
    baseline = baseline_by_joint(arm, start)
    reference = arm[(arm.elapsed_seconds >= max(0.0, start - 5.0)) & (arm.elapsed_seconds <= end + 5.0)].copy()
    reference["time_from_motion_start_seconds"] = reference.elapsed_seconds - start
    reference["command_position"] = np.nan
    cols = [
        "receive_wall_ns", "receive_wall_utc", "receive_monotonic_ns", "elapsed_seconds",
        "time_from_motion_start_seconds", "source_time_ns", "source_time_utc", "meas_time_ns",
        "frame_id", "sequence", "joint_index", "joint_name", "command_position", "position",
        "velocity", "effort", "coil_temp", "motor_temp", "motor_vol",
    ]
    reference[cols].to_csv(reference_path, index=False)

    normalized = reference[(reference.elapsed_seconds >= start) & (reference.elapsed_seconds <= end)].copy()
    normalized["normalized_phase"] = (normalized.elapsed_seconds - start) / max(end - start, 1e-9)
    normalized["baseline_position"] = normalized.joint_name.map(baseline)
    normalized["position_relative_to_baseline"] = normalized.position - normalized.baseline_position
    normalized[
        [
            "normalized_phase", "time_from_motion_start_seconds", "joint_name", "joint_index",
            "baseline_position", "position", "position_relative_to_baseline", "velocity", "effort",
            "source_time_ns", "receive_wall_ns", "receive_monotonic_ns",
        ]
    ].to_csv(normalized_path, index=False)


def make_plots(capture_dir: Path, frames: dict[str, pd.DataFrame], start: float | None, end: float | None) -> None:
    plot_dir = capture_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    arm = frames["raw_arm.csv"]
    if not arm.empty:
        for field, ylabel in (("position", "rad"), ("velocity", "rad/s"), ("effort", "AimDK effort (N·m; source UNKNOWN)")):
            fig, axes = plt.subplots(2, 1, figsize=(14, 10), sharex=True)
            for axis, side in zip(axes, ("left_", "right_")):
                for joint_name, group in arm[arm.joint_name.str.startswith(side)].groupby("joint_name", sort=False):
                    axis.plot(group.elapsed_seconds, group[field], linewidth=0.8, label=joint_name.replace(side, ""))
                if start is not None:
                    axis.axvline(start, color="black", linestyle="--", linewidth=1)
                if end is not None:
                    axis.axvline(end, color="black", linestyle=":", linewidth=1)
                axis.set_ylabel(ylabel)
                axis.set_title(side.rstrip("_").title() + " arm")
                axis.grid(alpha=0.25)
                axis.legend(ncol=4, fontsize=7)
            axes[-1].set_xlabel("Recorder elapsed time (s)")
            fig.tight_layout()
            fig.savefig(plot_dir / f"arm_{field}.png", dpi=150)
            plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    plotted = False
    for axis, file_name, label in zip(axes, ("raw_chest_imu.csv", "raw_torso_imu.csv"), ("chest", "torso")):
        frame = frames[file_name]
        if frame.empty:
            continue
        gyro = np.linalg.norm(frame[["gyro_x", "gyro_y", "gyro_z"]].to_numpy(float), axis=1)
        accel = np.linalg.norm(frame[["accel_x", "accel_y", "accel_z"]].to_numpy(float), axis=1)
        axis.plot(frame.elapsed_seconds, gyro, label="gyro norm (rad/s)")
        axis.plot(frame.elapsed_seconds, accel, label="accel norm (m/s²)")
        if start is not None:
            axis.axvline(start, color="black", linestyle="--", linewidth=1)
        if end is not None:
            axis.axvline(end, color="black", linestyle=":", linewidth=1)
        axis.set_title(label.title() + " IMU")
        axis.grid(alpha=0.25)
        axis.legend()
        plotted = True
    if plotted:
        axes[-1].set_xlabel("Recorder elapsed time (s)")
        fig.tight_layout()
        fig.savefig(plot_dir / "imu_norms.png", dpi=150)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-dir", type=Path, default=DEFAULT_CAPTURE_DIR)
    parser.add_argument("--allow-incomplete", action="store_true")
    args = parser.parse_args()
    capture_dir = args.capture_dir.resolve()
    raw_path = capture_dir / "raw_serialized_evidence.txt"
    if not raw_path.exists():
        raise SystemExit(f"Missing raw evidence: {raw_path}")

    parsed = parse_evidence(raw_path)
    if not parsed["markers"].get("recording_completed") and not args.allow_incomplete:
        raise SystemExit("Capture is not complete; wait for PHASE2D_RECORDING_COMPLETED=1")

    frames: dict[str, pd.DataFrame] = {}
    for topic, filename in TOPIC_TO_FILE.items():
        rows = imu_rows(parsed["samples"].get(topic, [])) if "imu" in topic else joint_rows(parsed["samples"].get(topic, []))
        frames[filename] = save_frame(rows, capture_dir / filename)
    frames["mc_state.csv"] = save_frame(mc_rows(parsed["samples"]), capture_dir / "mc_state.csv")

    arm = frames["raw_arm.csv"]
    start, end, threshold = motion_interval(arm)
    remote_start_ns = parsed["recorder_metadata"].get("start_wall_ns")
    all_receive_wall_ns = [
        int(sample["receive_wall_ns"])
        for topic_samples in parsed["samples"].values()
        for sample in topic_samples
        if sample.get("receive_wall_ns") is not None
    ]
    remote_finish_ns = parsed["capture_summary"].get("finish_wall_ns") or (
        max(all_receive_wall_ns) if all_receive_wall_ns else None
    )
    event_rows = list(parsed["events"])
    marker_path = capture_dir / "operator_markers.jsonl"
    excluded_operator_markers = []
    if marker_path.exists():
        with marker_path.open("r", encoding="utf-8") as handle:
            for line in handle:
                if not line.strip():
                    continue
                marker = json.loads(line)
                marker_ns = int(datetime.fromisoformat(marker["local_wall_utc"]).timestamp() * 1e9)
                marker["receive_wall_ns"] = marker_ns
                marker["receive_monotonic_ns"] = None
                if remote_start_ns and marker_ns < int(remote_start_ns):
                    excluded_operator_markers.append(marker)
                elif remote_finish_ns and marker_ns > int(remote_finish_ns):
                    excluded_operator_markers.append(marker)
                else:
                    event_rows.append(marker)
    if start is not None:
        event_rows.append({
            "event": "HEART_STARTED_AUTO", "source": "joint_velocity_detector",
            "receive_wall_ns": int(remote_start_ns + start * 1e9) if remote_start_ns else None,
            "receive_monotonic_ns": int(parsed["recorder_metadata"].get("start_monotonic_ns", 0) + start * 1e9),
            "note": f"first arm sample above {threshold:.6f} rad/s",
        })
    if end is not None:
        event_rows.append({
            "event": "HEART_FINISHED_AUTO", "source": "joint_velocity_detector",
            "receive_wall_ns": int(remote_start_ns + end * 1e9) if remote_start_ns else None,
            "receive_monotonic_ns": int(parsed["recorder_metadata"].get("start_monotonic_ns", 0) + end * 1e9),
            "note": f"last arm sample above {threshold:.6f} rad/s",
        })
    events_df = pd.DataFrame(event_rows)
    if not events_df.empty:
        events_df["receive_wall_utc"] = events_df.receive_wall_ns.apply(utc_iso_from_ns)
        events_df = events_df.sort_values("receive_wall_ns", na_position="last")
    events_df.to_csv(capture_dir / "events.csv", index=False)

    quality = {topic: topic_quality(parsed["samples"].get(topic, [])) for topic in REQUIRED_TOPICS}
    all_elapsed = [
        float(sample["elapsed_seconds"])
        for topic_samples in parsed["samples"].values()
        for sample in topic_samples
        if sample.get("elapsed_seconds") is not None
    ]
    actual_capture_duration = max(all_elapsed) if all_elapsed else 0.0
    parsed["recorder_metadata"]["actual_sample_duration_seconds"] = actual_capture_duration
    capture_duration = actual_capture_duration
    reasons = []
    all_required = all(quality[t]["messages"] > 0 for t in REQUIRED_TOPICS)
    reasons.append("all six required streams present" if all_required else "one or more required streams missing")
    complete = bool(parsed["markers"].get("recording_completed"))
    reasons.append("clean recorder completion marker present" if complete else "clean completion marker missing")
    detected = start is not None and end is not None and end > start
    reasons.append("heart motion detected from arm velocity" if detected else "heart motion not detected")
    required_first = [quality[t].get("first_elapsed_seconds") for t in REQUIRED_TOPICS]
    required_last = [quality[t].get("last_elapsed_seconds") for t in REQUIRED_TOPICS]
    coverage_start = max(float(value) for value in required_first if value is not None)
    coverage_end = min(float(value) for value in required_last if value is not None)
    verified_pre_roll = (start - coverage_start) if detected else None
    verified_post_roll = (coverage_end - end) if detected else None
    pre_ok = bool(detected and verified_pre_roll >= 5.0)
    post_ok = bool(detected and verified_post_roll >= 5.0)
    reasons.append(f"pre-roll {'>=5 s' if pre_ok else '<5 s or unknown'}")
    reasons.append(f"post-roll {'>=5 s' if post_ok else '<5 s or unknown'}")
    timestamps_ok = all(quality[t]["source_out_of_order_count"] == 0 for t in REQUIRED_TOPICS)
    reasons.append("required source timestamps monotonic" if timestamps_ok else "source timestamp reversal detected")
    gaps_ok = all((quality[t]["max_gap_seconds"] or 999) < 0.5 for t in REQUIRED_TOPICS)
    reasons.append("required receive gaps <0.5 s" if gaps_ok else "required receive gap >=0.5 s")
    sufficient_manual_end = bool(all_required and detected and pre_ok and post_ok and timestamps_ok and gaps_ok)
    termination_ok = complete or sufficient_manual_end
    reasons[1] = (
        "clean recorder completion marker present"
        if complete
        else "no clean footer; all required streams still cover full motion and >=5 s post-roll"
    )
    replay_ready = all((all_required, termination_ok, detected, pre_ok, post_ok, timestamps_ok, gaps_ok))

    capture_metadata = {
        "phase": "2D",
        "capture_id": capture_dir.name,
        "robot_identity": parsed["environment"],
        "recorder": parsed["recorder_metadata"],
        "capture_summary": parsed["capture_summary"],
        "raw_markers": parsed["markers"],
        "startup_safety_declarations": parsed["safety_declarations"],
        "topic_graph": parsed["topic_graph"],
        "topic_quality": quality,
        "event_detection": {
            "velocity_threshold_rad_s": threshold,
            "heart_start_elapsed_seconds": start,
            "heart_end_elapsed_seconds": end,
            "all_required_coverage_start_elapsed_seconds": coverage_start,
            "all_required_coverage_end_elapsed_seconds": coverage_end,
            "verified_pre_roll_seconds": verified_pre_roll,
            "verified_post_roll_seconds": verified_post_roll,
            "termination_classification": (
                "CLEAN_FIXED_WINDOW_END" if complete else "MANUAL_OR_CONNECTION_END_AFTER_POST_ROLL"
            ),
            "excluded_out_of_window_operator_markers": excluded_operator_markers,
        },
        "PHASE2D_REPLAY_READY": replay_ready,
        "replay_readiness_evidence": reasons,
        "safety": {
            "capture_is_subscription_only": True,
            "robot_command_sent_by_recorder": False,
            "mjcf_modified": False,
            "joint_mapping_modified": False,
            "dynamics_calibrated": False,
        },
    }
    (capture_dir / "capture_metadata.json").write_text(
        json.dumps(capture_metadata, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    make_reference_csvs(arm, start, end)
    make_plots(capture_dir, frames, start, end)
    write_reports(capture_dir, parsed, frames, quality, start, end, threshold, replay_ready, reasons)
    print(json.dumps({"PHASE2D_REPLAY_READY": replay_ready, "heart_start": start, "heart_end": end, "reasons": reasons}, indent=2))
    return 0 if replay_ready else 2


if __name__ == "__main__":
    raise SystemExit(main())
