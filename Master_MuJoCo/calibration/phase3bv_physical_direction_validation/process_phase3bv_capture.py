#!/usr/bin/env python3
"""Offline quality gate for a Phase 3B-V subscription-only capture.

This module has no SSH, ROS, SDK, publisher, service/action, or robot-control
imports. Reported effort is neither loaded into validation references nor used.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
import pandas as pd


HERE = Path(__file__).resolve().parent
CALIBRATION = HERE.parent
PROJECT = CALIBRATION.parent
AV_DIR = CALIBRATION / "phase3av_validation"
for path in (CALIBRATION, AV_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
import process_phase2d_capture as common  # noqa: E402
import process_phase3av_capture as av  # noqa: E402


DEFAULT_CAPTURE = HERE / "capture" / "phase3bv_clap_001"
GROUP_FILES = av.GROUP_FILES
REQUIRED_TOPICS = tuple(common.TOPIC_TO_FILE)


def joint_metrics_no_effort(joints: pd.DataFrame, start: float, end: float) -> pd.DataFrame:
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
        rows.append({
            "joint_name": joint_name,
            "joint_group": str(frame.joint_group.iloc[0]),
            "q_initial": initial,
            "q_min": float(position.min()), "q_max": float(position.max()),
            "excursion": float(position.max() - position.min()),
            "peak_abs_dq": float(velocity.abs().max()),
            "motion_onset": 0.0, "motion_end": end - start,
            "final_minus_initial": final - initial,
        })
    result = pd.DataFrame(rows)
    arm = result[result.joint_group == "arm"]
    max_excursion = float(arm.excursion.max()) if len(arm) else 0.0
    max_velocity = float(arm.peak_abs_dq.max()) if len(arm) else 0.0
    primary_excursion = max(0.08, 0.25 * max_excursion)
    primary_velocity = max(0.25, 0.30 * max_velocity)
    classes, bases = [], []
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
        classes.append(classification)
        bases.append(
            f"group={row.joint_group}; excursion={row.excursion:.6f} rad; "
            f"peak|dq|={row.peak_abs_dq:.6f} rad/s; measured state only"
        )
    result["classification"] = classes
    result["classification_basis"] = bases
    return result.sort_values(["joint_group", "joint_name"]).reset_index(drop=True)


def measured_reference_no_effort(joints: pd.DataFrame, start: float, end: float) -> pd.DataFrame:
    timeline = np.arange(-5.0, (end - start) + 5.0 + 0.5 / av.RATE_HZ, 1.0 / av.RATE_HZ)
    rows = []
    for joint_name, frame in joints.groupby("joint_name"):
        frame = frame.sort_values("elapsed_seconds")
        numeric = frame.groupby("elapsed_seconds", as_index=False).agg(
            position=("position", "mean"), velocity=("velocity", "mean"),
        )
        source_t = numeric.elapsed_seconds.to_numpy(float) - start
        if len(source_t) < 2 or timeline[0] < source_t[0] or timeline[-1] > source_t[-1]:
            continue
        rows.append(pd.DataFrame({
            "t": timeline, "joint_name": joint_name,
            "joint_group": str(frame.joint_group.iloc[0]),
            "position": np.interp(timeline, source_t, numeric.position),
            "velocity": np.interp(timeline, source_t, numeric.velocity),
            "trajectory_type": "MEASURED_REAL_TRAJECTORY",
            "command_semantics": "MC_INTERNAL_COMMAND_UNOBSERVABLE",
        }))
    return pd.concat(rows, ignore_index=True) if rows else pd.DataFrame()


def prior_motion_comparison(metrics: pd.DataFrame, duration_s: float) -> dict[str, object]:
    gesture_classes = {"GESTURE_PRIMARY", "GESTURE_SECONDARY"}
    current_set = set(metrics.loc[metrics.classification.isin(gesture_classes), "joint_name"])
    current_exc = metrics.set_index("joint_name").excursion
    prior_paths = {
        "heart_both": CALIBRATION / "phase2e_replay" / "phase2e_joint_metrics.csv",
        "wave_right": AV_DIR / "phase3av_joint_metrics.csv",
    }
    comparisons = []
    all_independent = bool(current_set)
    for name, path in prior_paths.items():
        prior = pd.read_csv(path)
        prior_set = set(prior.loc[prior.classification.isin(gesture_classes), "joint_name"])
        union = sorted(current_set | prior_set)
        intersection = current_set & prior_set
        jaccard = len(intersection) / len(union) if union else 1.0
        x = current_exc.reindex(union, fill_value=0.0).to_numpy(float)
        y = prior.set_index("joint_name").excursion.reindex(union, fill_value=0.0).to_numpy(float)
        cosine = float(np.dot(x, y) / (np.linalg.norm(x) * np.linalg.norm(y))) if np.linalg.norm(x) and np.linalg.norm(y) else np.nan
        prior_duration = float(prior.motion_end.dropna().max() - prior.motion_onset.dropna().min())
        duration_fraction = abs(duration_s - prior_duration) / max(prior_duration, 1e-12)
        independent = bool(
            current_set
            and (jaccard < 0.80 or (math.isfinite(cosine) and cosine < 0.90) or duration_fraction >= 0.20)
        )
        all_independent = all_independent and independent
        comparisons.append({
            "prior_motion": name,
            "active_set_jaccard": jaccard,
            "excursion_vector_cosine_similarity": cosine,
            "current_duration_s": duration_s,
            "prior_duration_s": prior_duration,
            "duration_fraction_difference": duration_fraction,
            "independent": independent,
        })
    left = float(metrics.loc[metrics.joint_name.str.startswith("left_") & (metrics.joint_group == "arm"), "excursion"].sum())
    right = float(metrics.loc[metrics.joint_name.str.startswith("right_") & (metrics.joint_group == "arm"), "excursion"].sum())
    return {
        "selected_motion": "clap",
        "preset_id": 3017,
        "preset_area": 11,
        "current_active_joint_set": sorted(current_set),
        "left_arm_excursion_sum_rad": left,
        "right_arm_excursion_sum_rad": right,
        "left_right_excursion_asymmetry": abs(left - right) / max(left + right, 1e-12),
        "comparisons": comparisons,
        "decision": "SUFFICIENTLY_INDEPENDENT_FROM_HEART_AND_WAVE" if all_independent else "INSUFFICIENTLY_INDEPENDENT",
    }


def pending(reason: str) -> None:
    (HERE / "phase3bv_capture_quality_report.md").write_text(
        "# Phase 3B-V capture quality\n\n"
        "`PHASE3BV_VALIDATION_DATA_READY = NO`\n\n"
        f"Stopped before replay: {reason}\n",
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-dir", type=Path, default=DEFAULT_CAPTURE)
    args = parser.parse_args()
    capture_dir = args.capture_dir.resolve()
    raw_path = capture_dir / "raw_serialized_evidence.txt"
    if not raw_path.exists():
        pending(f"missing raw evidence: {raw_path}")
        return 2

    parsed = common.parse_evidence(raw_path)
    frames: dict[str, pd.DataFrame] = {}
    for topic, filename in common.TOPIC_TO_FILE.items():
        samples = parsed["samples"].get(topic, [])
        rows = common.imu_rows(samples) if "imu" in topic else common.joint_rows(samples)
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
    names_ok, name_issues = av.stable_joint_names(frames)
    if all_required:
        coverage_start = max(float(quality[topic]["first_elapsed_seconds"]) for topic in REQUIRED_TOPICS)
        coverage_end = min(float(quality[topic]["last_elapsed_seconds"]) for topic in REQUIRED_TOPICS)
    else:
        coverage_start = coverage_end = np.nan
    pre_roll = float(start - coverage_start) if detected and math.isfinite(coverage_start) else np.nan
    post_roll = float(coverage_end - end) if detected and math.isfinite(coverage_end) else np.nan
    timestamps_ok = all(item["source_out_of_order_count"] == 0 for item in quality.values())
    gaps_ok = all(item["max_gap_seconds"] is not None and item["max_gap_seconds"] < 0.5 for item in quality.values())
    mc_ok, mc_detail = av.mc_mode_quality(mc, start, end)
    complete = bool(parsed["markers"].get("recording_completed"))
    termination_ok = bool(complete or (all_required and detected and post_roll >= 5.0))
    ready = bool(
        all_required and termination_ok and detected
        and pre_roll >= 5.0 and post_roll >= 5.0
        and timestamps_ok and gaps_ok and names_ok and mc_ok
    )
    rows = [[
        topic, item["messages"], av.fnum(item["mean_rate_hz"], 2),
        av.fnum(item["max_gap_seconds"], 4), item["source_out_of_order_count"], item["frame_ids"],
    ] for topic, item in quality.items()]
    report = f"""# Phase 3B-V capture quality

`PHASE3BV_VALIDATION_DATA_READY = {'YES' if ready else 'NO'}`

- capture: `{capture_dir.name}`
- selected candidate: `clap`, native MC preset 3017 / area 11
- recorder complete/data-window accepted: `{complete}` / `{termination_ok}`
- detected motion: `{av.fnum(start, 3)} s -> {av.fnum(end, 3)} s`; threshold `{av.fnum(threshold, 4)} rad/s`
- pre-roll/post-roll: `{av.fnum(pre_roll, 3)} / {av.fnum(post_roll, 3)} s`
- stable joint names: `{names_ok}`; issues: `{name_issues or 'none'}`
- source timestamps monotonic: `{timestamps_ok}`
- max receive gaps below 0.5 s: `{gaps_ok}`
- MC expected standing mode: `{mc_detail}`
- motion invocation: operator only; recorder sent no command
- reported effort: excluded from all validation references and metrics

{av.table(['topic', 'samples', 'mean Hz', 'max gap s', 'source reversals', 'frame IDs'], rows)}
"""
    (HERE / "phase3bv_capture_quality_report.md").write_text(report, encoding="utf-8")
    metadata = {
        "phase": "3B-V",
        "capture_dir": str(capture_dir),
        "selected_motion": "clap",
        "preset_id": 3017,
        "preset_area": 11,
        "data_ready": ready,
        "motion_start_elapsed_seconds": start,
        "motion_end_elapsed_seconds": end,
        "motion_duration_seconds": (end - start) if detected else None,
        "pre_roll_seconds": pre_roll,
        "post_roll_seconds": post_roll,
        "quality": quality,
        "mc": mc_detail,
        "reported_effort_loaded_for_validation": False,
        "imu_transform": "PARTIAL",
    }
    (HERE / "phase3bv_capture_metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    if not ready:
        return 3

    assert start is not None and end is not None
    metrics = joint_metrics_no_effort(joints, start, end)
    metrics.to_csv(HERE / "phase3bv_joint_metrics.csv", index=False)
    reference = measured_reference_no_effort(joints, start, end)
    reference.to_csv(HERE / "phase3bv_measured_reference.csv", index=False)
    reference[["t", "joint_name", "joint_group", "position", "velocity"]].to_csv(
        HERE / "phase3bv_aligned_joint_data.csv", index=False
    )
    imu = av.aligned_imu(frames, start, end)
    imu.to_csv(HERE / "phase3bv_aligned_imu_data.csv", index=False)
    independent = prior_motion_comparison(metrics, end - start)
    (HERE / "phase3bv_independence.json").write_text(json.dumps(independent, indent=2), encoding="utf-8")
    if independent["decision"] != "SUFFICIENTLY_INDEPENDENT_FROM_HEART_AND_WAVE":
        print(json.dumps({"PHASE3BV_VALIDATION_DATA_READY": "NO", "reason": independent["decision"]}, indent=2))
        return 4
    print(json.dumps({"PHASE3BV_VALIDATION_DATA_READY": "YES", "independence": independent["decision"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
