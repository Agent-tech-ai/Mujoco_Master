#!/usr/bin/env python3
"""Quality-gate and extract a Phase 3A-V read-only independent-motion capture.

The recorder intentionally retains the Phase 2D wire markers so the already
validated parser can be reused.  This module is offline-only: it has no ROS,
SSH, SDK, publisher, service, action, or robot-control imports.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import Counter
from pathlib import Path

import numpy as np
import pandas as pd


HERE = Path(__file__).resolve().parent
CALIBRATION = HERE.parent
PROJECT = CALIBRATION.parent
if str(CALIBRATION) not in sys.path:
    sys.path.insert(0, str(CALIBRATION))

import process_phase2d_capture as common


DEFAULT_CAPTURE = HERE / "capture" / "phase3av_wave_right_001"
GROUP_FILES = {
    "arm": "raw_arm.csv",
    "head": "raw_head.csv",
    "leg": "raw_leg.csv",
    "waist": "raw_waist.csv",
}
REQUIRED_TOPICS = tuple(common.TOPIC_TO_FILE)
RATE_HZ = 50.0


def fnum(value: object, digits: int = 5) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "UNKNOWN"
    return f"{number:.{digits}f}" if math.isfinite(number) else "UNKNOWN"


def table(headers: list[str], rows: list[list[object]]) -> str:
    clean = lambda value: str(value).replace("|", "\\|").replace("\n", " ")
    rendered = ["| " + " | ".join(map(clean, headers)) + " |", "| " + " | ".join(["---"] * len(headers)) + " |"]
    rendered.extend("| " + " | ".join(clean(value) for value in row) + " |" for row in rows)
    return "\n".join(rendered)


def quaternion_rpy(x: np.ndarray, y: np.ndarray, z: np.ndarray, w: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    norm = np.sqrt(x * x + y * y + z * z + w * w)
    norm = np.where(norm > 1e-12, norm, 1.0)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def stable_joint_names(frames: dict[str, pd.DataFrame]) -> tuple[bool, list[str]]:
    issues: list[str] = []
    for group, filename in GROUP_FILES.items():
        frame = frames[filename]
        if frame.empty:
            issues.append(f"{group}: no rows")
            continue
        for index, rows in frame.groupby("joint_index"):
            names = {str(value) for value in rows.joint_name.dropna() if str(value)}
            if len(names) != 1:
                issues.append(f"{group}[{index}]: names={sorted(names)}")
    return not issues, issues


def mc_mode_quality(mc: pd.DataFrame, start: float | None, end: float | None) -> tuple[bool, dict[str, object]]:
    if mc.empty:
        return False, {"status": "MC_STATE_UNOBSERVED"}
    during = mc
    if start is not None and end is not None and "elapsed_seconds" in mc:
        during = mc[mc.elapsed_seconds.between(max(0.0, start - 1.0), end + 1.0)]
    action_col = "message.action_info.action_desc"
    source_col = "message.input_source.name"
    actions = sorted({str(value) for value in during.get(action_col, pd.Series(dtype=object)).dropna()})
    sources = sorted({str(value) for value in during.get(source_col, pd.Series(dtype=object)).dropna()})
    status = bool(len(during) and actions and set(actions) == {"STAND_DEFAULT"})
    return status, {
        "status": "EXPECTED_STANDING_MODE_CONFIRMED" if status else "EXPECTED_STANDING_MODE_NOT_CONFIRMED",
        "messages_during_motion_window": int(len(during)),
        "action_desc_values": actions,
        "input_source_values": sources,
        "motion_status_values": sorted({str(value) for value in during.get("message.motion_status.motion", pd.Series(dtype=object)).dropna()}),
        "player_state_values": sorted({str(value) for value in during.get("message.motion_status.player_state.value", pd.Series(dtype=object)).dropna()}),
    }


def joint_metrics(joints: pd.DataFrame, start: float, end: float) -> pd.DataFrame:
    rows: list[dict[str, object]] = []
    for joint_name, frame in joints.groupby("joint_name"):
        frame = frame.sort_values("elapsed_seconds")
        pre = frame[frame.elapsed_seconds.between(max(0.0, start - 3.0), start - 0.2)]
        motion = frame[frame.elapsed_seconds.between(start, end)]
        post = frame[frame.elapsed_seconds.between(end + 0.2, end + 3.0)]
        if motion.empty:
            continue
        initial = float(pd.to_numeric(pre.position, errors="coerce").mean()) if len(pre) else float(motion.position.iloc[0])
        final = float(pd.to_numeric(post.position, errors="coerce").mean()) if len(post) else float(motion.position.iloc[-1])
        position = pd.to_numeric(motion.position, errors="coerce")
        velocity = pd.to_numeric(motion.velocity, errors="coerce")
        effort = pd.to_numeric(motion.effort, errors="coerce")
        rows.append(
            {
                "joint_name": joint_name,
                "joint_group": str(frame.joint_group.iloc[0]),
                "q_initial": initial,
                "q_min": float(position.min()),
                "q_max": float(position.max()),
                "excursion": float(position.max() - position.min()),
                "peak_abs_dq": float(velocity.abs().max()),
                "motion_onset": 0.0,
                "motion_end": end - start,
                "final_minus_initial": final - initial,
                "reported_effort_baseline": float(pd.to_numeric(pre.effort, errors="coerce").mean()) if len(pre) else np.nan,
                "peak_abs_reported_effort": float(effort.abs().max()),
                "reported_effort_usage": "QUALITATIVE_RECORD_ONLY_EFFORT_SEMANTICS_UNKNOWN",
            }
        )
    result = pd.DataFrame(rows)
    arm = result[result.joint_group == "arm"]
    max_excursion = float(arm.excursion.max()) if len(arm) else 0.0
    max_velocity = float(arm.peak_abs_dq.max()) if len(arm) else 0.0
    primary_excursion = max(0.08, 0.25 * max_excursion)
    primary_velocity = max(0.25, 0.30 * max_velocity)
    classifications = []
    bases = []
    for row in result.itertuples(index=False):
        if row.joint_group == "arm" and (row.excursion >= primary_excursion or row.peak_abs_dq >= primary_velocity):
            classification = "GESTURE_PRIMARY"
        elif row.joint_group in {"arm", "head"} and (row.excursion >= 0.02 or row.peak_abs_dq >= 0.10):
            classification = "GESTURE_SECONDARY"
        elif row.joint_group in {"leg", "waist"} and (row.excursion >= 0.005 or row.peak_abs_dq >= 0.05):
            classification = "BALANCE_COMPENSATION_CANDIDATE"
        elif row.excursion < 0.005 and row.peak_abs_dq < 0.05:
            classification = "STATIC"
        else:
            classification = "UNKNOWN"
        classifications.append(classification)
        bases.append(
            f"group={row.joint_group}; excursion={row.excursion:.6f} rad; "
            f"peak|dq|={row.peak_abs_dq:.6f} rad/s; measured state only"
        )
    result["classification"] = classifications
    result["classification_basis"] = bases
    return result.sort_values(["joint_group", "joint_name"]).reset_index(drop=True)


def measured_reference(joints: pd.DataFrame, start: float, end: float) -> pd.DataFrame:
    timeline = np.arange(-5.0, (end - start) + 5.0 + 0.5 / RATE_HZ, 1.0 / RATE_HZ)
    rows: list[pd.DataFrame] = []
    for joint_name, frame in joints.groupby("joint_name"):
        frame = frame.sort_values("elapsed_seconds")
        numeric = frame.groupby("elapsed_seconds", as_index=False).agg(
            position=("position", "mean"),
            velocity=("velocity", "mean"),
            reported_effort=("effort", "mean"),
        )
        source_t = numeric.elapsed_seconds.to_numpy(float) - start
        if len(source_t) < 2 or timeline[0] < source_t[0] or timeline[-1] > source_t[-1]:
            continue
        rows.append(
            pd.DataFrame(
                {
                    "t": timeline,
                    "joint_name": joint_name,
                    "joint_group": str(frame.joint_group.iloc[0]),
                    "position": np.interp(timeline, source_t, numeric.position),
                    "velocity": np.interp(timeline, source_t, numeric.velocity),
                    "reported_effort": np.interp(timeline, source_t, numeric.reported_effort),
                    "trajectory_type": "MEASURED_REAL_TRAJECTORY",
                    "command_semantics": "MC_INTERNAL_COMMAND_UNOBSERVABLE",
                }
            )
        )
    return pd.concat(rows, ignore_index=True) if rows else pd.DataFrame()


def aligned_imu(frames: dict[str, pd.DataFrame], start: float, end: float) -> pd.DataFrame:
    timeline = np.arange(-5.0, (end - start) + 5.0 + 0.5 / RATE_HZ, 1.0 / RATE_HZ)
    output = []
    for imu, filename in (("chest", "raw_chest_imu.csv"), ("torso", "raw_torso_imu.csv")):
        frame = frames[filename].sort_values("elapsed_seconds")
        if len(frame) < 2:
            continue
        t = frame.elapsed_seconds.to_numpy(float) - start
        roll, pitch, yaw = quaternion_rpy(
            frame.orientation_x.to_numpy(float), frame.orientation_y.to_numpy(float),
            frame.orientation_z.to_numpy(float), frame.orientation_w.to_numpy(float),
        )
        if timeline[0] < t[0] or timeline[-1] > t[-1]:
            continue
        roll_i, pitch_i, yaw_i = (np.interp(timeline, t, values) for values in (roll, pitch, yaw))
        pre = (timeline >= -3.0) & (timeline <= -0.2)
        gx = np.interp(timeline, t, frame.gyro_x)
        gy = np.interp(timeline, t, frame.gyro_y)
        gz = np.interp(timeline, t, frame.gyro_z)
        output.append(
            pd.DataFrame(
                {
                    "t": timeline,
                    "imu": imu,
                    "relative_roll_rad": roll_i - float(np.mean(roll_i[pre])),
                    "relative_pitch_rad": pitch_i - float(np.mean(pitch_i[pre])),
                    "relative_yaw_rad_diagnostic_only": yaw_i - float(np.mean(yaw_i[pre])),
                    "gyro_x": gx,
                    "gyro_y": gy,
                    "gyro_z": gz,
                    "gyro_norm": np.sqrt(gx * gx + gy * gy + gz * gz),
                    "comparison_scope": "RELATIVE_ROLL_PITCH_AND_GYRO_ONLY_IMU_TRANSFORM_PARTIAL",
                }
            )
        )
    return pd.concat(output, ignore_index=True) if output else pd.DataFrame()


def independence(metrics: pd.DataFrame, duration: float) -> dict[str, object]:
    heart = pd.read_csv(CALIBRATION / "phase2e_replay" / "phase2e_joint_metrics.csv")
    gesture_classes = {"GESTURE_PRIMARY", "GESTURE_SECONDARY"}
    current_set = set(metrics.loc[metrics.classification.isin(gesture_classes), "joint_name"])
    heart_set = set(heart.loc[heart.classification.isin(gesture_classes), "joint_name"])
    union = sorted(current_set | heart_set)
    intersection = current_set & heart_set
    jaccard = len(intersection) / len(union) if union else 1.0
    current_exc = metrics.set_index("joint_name").excursion.reindex(union, fill_value=0.0).to_numpy(float)
    heart_exc = heart.set_index("joint_name").excursion.reindex(union, fill_value=0.0).to_numpy(float)
    cosine = float(np.dot(current_exc, heart_exc) / (np.linalg.norm(current_exc) * np.linalg.norm(heart_exc))) if np.linalg.norm(current_exc) and np.linalg.norm(heart_exc) else np.nan
    left = float(metrics.loc[metrics.joint_name.str.startswith("left_") & (metrics.joint_group == "arm"), "excursion"].sum())
    right = float(metrics.loc[metrics.joint_name.str.startswith("right_") & (metrics.joint_group == "arm"), "excursion"].sum())
    asymmetry = abs(left - right) / max(left + right, 1e-12)
    heart_duration = float(heart.motion_end.dropna().max() - heart.motion_onset.dropna().min())
    sufficient = bool((jaccard < 0.80 or (math.isfinite(cosine) and cosine < 0.90) or asymmetry > 0.35) and current_set)
    return {
        "selected_motion": "wave_right",
        "heart_motion": "heart_both",
        "current_active_joint_set": sorted(current_set),
        "heart_active_joint_set": sorted(heart_set),
        "active_set_jaccard": jaccard,
        "excursion_vector_cosine_similarity": cosine,
        "current_duration_s": duration,
        "heart_duration_s": heart_duration,
        "current_left_arm_excursion_sum_rad": left,
        "current_right_arm_excursion_sum_rad": right,
        "left_right_excursion_asymmetry": asymmetry,
        "decision": "SUFFICIENTLY_INDEPENDENT_VALIDATION_MOTION" if sufficient else "INSUFFICIENTLY_INDEPENDENT_VALIDATION_MOTION",
    }


def write_pending(reason: str, *, replace_capture_report: bool = True) -> None:
    if replace_capture_report:
        (HERE / "phase3av_capture_quality_report.md").write_text(
            "# Phase 3A-V capture quality\n\n"
            "`PHASE3AV_VALIDATION_DATA_READY = NO`\n\n"
            f"Stopped before replay: {reason}\n",
            encoding="utf-8",
        )
    (HERE / "phase3av_final_gate.md").write_text(
        "# Phase 3A-V final gate\n\n"
        "`VALIDATED_SIM_CONTROLLER_BASELINE = NO`\n\n"
        "This is a pending-data gate, not a validation failure. No replay or optimization was performed.\n",
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-dir", type=Path, default=DEFAULT_CAPTURE)
    args = parser.parse_args()
    capture_dir = args.capture_dir.resolve()
    raw_path = capture_dir / "raw_serialized_evidence.txt"
    if not raw_path.exists():
        write_pending(f"missing raw evidence: {raw_path}")
        return 2

    parsed = common.parse_evidence(raw_path)
    frames: dict[str, pd.DataFrame] = {}
    for topic, filename in common.TOPIC_TO_FILE.items():
        rows = common.imu_rows(parsed["samples"].get(topic, [])) if "imu" in topic else common.joint_rows(parsed["samples"].get(topic, []))
        frames[filename] = common.save_frame(rows, capture_dir / filename)
    mc = common.save_frame(common.mc_rows(parsed["samples"]), capture_dir / "mc_state.csv")

    joint_frames = []
    for group, filename in GROUP_FILES.items():
        frame = frames[filename].copy()
        frame["joint_group"] = group
        joint_frames.append(frame)
    joints = pd.concat(joint_frames, ignore_index=True)
    start, end, threshold = common.motion_interval(frames["raw_arm.csv"])
    quality = {topic: common.topic_quality(parsed["samples"].get(topic, [])) for topic in REQUIRED_TOPICS}
    all_required = all(item["messages"] > 0 for item in quality.values())
    detected = start is not None and end is not None and end > start
    names_ok, name_issues = stable_joint_names(frames)
    if all_required:
        coverage_start = max(float(quality[topic]["first_elapsed_seconds"]) for topic in REQUIRED_TOPICS)
        coverage_end = min(float(quality[topic]["last_elapsed_seconds"]) for topic in REQUIRED_TOPICS)
    else:
        coverage_start = coverage_end = np.nan
    pre_roll = float(start - coverage_start) if detected and math.isfinite(coverage_start) else np.nan
    post_roll = float(coverage_end - end) if detected and math.isfinite(coverage_end) else np.nan
    timestamps_ok = all(item["source_out_of_order_count"] == 0 for item in quality.values())
    gaps_ok = all((item["max_gap_seconds"] is not None and item["max_gap_seconds"] < 0.5) for item in quality.values())
    mc_ok, mc_detail = mc_mode_quality(mc, start, end)
    complete = bool(parsed["markers"].get("recording_completed"))
    # A clean footer is preferred but is not a data requirement. Phase 2D showed
    # that an SSH wrapper can exit after all samples are safely retained. Full
    # required-stream coverage through >=5 s post-roll is the recoverable gate.
    termination_ok = bool(complete or (all_required and detected and post_roll >= 5.0))
    ready = bool(all_required and termination_ok and detected and pre_roll >= 5.0 and post_roll >= 5.0 and timestamps_ok and gaps_ok and names_ok and mc_ok)

    quality_rows = [[topic, item["messages"], fnum(item["mean_rate_hz"], 2), fnum(item["max_gap_seconds"], 4), item["source_out_of_order_count"], item["frame_ids"]] for topic, item in quality.items()]
    report = f"""# Phase 3A-V capture quality report

`PHASE3AV_VALIDATION_DATA_READY = {'YES' if ready else 'NO'}`

- capture: `{capture_dir.name}`
- selected preset: `wave(right)`, native MC preset 1002 / area 2
- complete recorder footer: `{complete}`
- acceptable complete data window/footer: `{termination_ok}`
- detected motion: `{fnum(start, 3)} s -> {fnum(end, 3)} s`; velocity threshold `{fnum(threshold, 4)} rad/s`
- verified pre-roll/post-roll: `{fnum(pre_roll, 3)} / {fnum(post_roll, 3)} s`
- stable joint names: `{names_ok}`; issues: `{name_issues or 'none'}`
- required source timestamps monotonic: `{timestamps_ok}`
- all required max receive gaps below 0.5 s: `{gaps_ok}`
- MC expected standing mode: `{mc_detail}`
- motion invocation: operator/soft-engineer only; recorder sent no command

{table(['topic', 'samples', 'mean Hz', 'max gap s', 'source reversals', 'frame IDs'], quality_rows)}
"""
    (HERE / "phase3av_capture_quality_report.md").write_text(report, encoding="utf-8")
    metadata = {
        "phase": "3A-V",
        "capture_dir": str(capture_dir),
        "data_ready": ready,
        "motion_start_elapsed_seconds": start,
        "motion_end_elapsed_seconds": end,
        "motion_duration_seconds": (end - start) if detected else None,
        "pre_roll_seconds": pre_roll,
        "post_roll_seconds": post_roll,
        "quality": quality,
        "mc": mc_detail,
        "effort_semantics": "UNKNOWN",
        "imu_transform": "PARTIAL",
    }
    (HERE / "phase3av_capture_metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    if not ready:
        write_pending(
            "capture quality gate failed; see phase3av_capture_quality_report.md",
            replace_capture_report=False,
        )
        return 3

    assert start is not None and end is not None
    metrics = joint_metrics(joints, start, end)
    metrics.to_csv(HERE / "phase3av_joint_metrics.csv", index=False)
    reference = measured_reference(joints, start, end)
    reference.to_csv(HERE / "phase3av_measured_reference.csv", index=False)
    aligned_joint = reference[["t", "joint_name", "joint_group", "position", "velocity"]].copy()
    aligned_joint.to_csv(HERE / "phase3av_aligned_joint_data.csv", index=False)
    imu = aligned_imu(frames, start, end)
    imu.to_csv(HERE / "phase3av_aligned_imu_data.csv", index=False)
    independent = independence(metrics, end - start)
    (HERE / "phase3av_independence.json").write_text(json.dumps(independent, indent=2), encoding="utf-8")

    class_rows = [[row.joint_name, row.joint_group, row.classification, fnum(row.excursion), fnum(row.peak_abs_dq), row.classification_basis] for row in metrics.itertuples(index=False)]
    classification_report = f"""# Phase 3A-V measured-joint classification

All positions and velocities are measured output state: **MEASURED_REAL_TRAJECTORY**. They are not MC internal commands.

Reported effort is retained only as `reported_effort`; semantics remain UNKNOWN and it is not used for fitting.

{table(['joint', 'group', 'classification', 'excursion rad', 'peak |dq| rad/s', 'basis'], class_rows)}

## Independence from heart

- decision: **{independent['decision']}**
- active-set Jaccard: `{fnum(independent['active_set_jaccard'], 3)}`
- excursion-vector cosine similarity: `{fnum(independent['excursion_vector_cosine_similarity'], 3)}`
- left/right excursion asymmetry: `{fnum(independent['left_right_excursion_asymmetry'], 3)}`
- duration wave/heart: `{fnum(independent['current_duration_s'], 3)} / {fnum(independent['heart_duration_s'], 3)} s`
"""
    (HERE / "phase3av_joint_classification.md").write_text(classification_report, encoding="utf-8")
    print(json.dumps({"PHASE3AV_VALIDATION_DATA_READY": "YES", "independence": independent["decision"], "motion_duration_s": end - start}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
