#!/usr/bin/env python3
"""Extract heart/wave output-response targets without identifying MC gains."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pandas as pd


HERE = Path(__file__).resolve().parent
CALIBRATION = HERE.parent
BALANCE_JOINTS = (
    "left_ankle_pitch_joint",
    "right_ankle_pitch_joint",
    "left_hip_pitch_joint",
    "right_hip_pitch_joint",
    "left_knee_joint",
    "right_knee_joint",
    "waist_pitch_joint",
    "waist_roll_joint",
)


def recovery_time(t: np.ndarray, y: np.ndarray, motion_end: float, tolerance: float) -> float | None:
    indices = np.flatnonzero(t >= motion_end)
    for index in indices:
        if np.all(np.abs(y[index:]) <= tolerance):
            return float(t[index] - motion_end)
    return None


def extract(dataset: str, path: Path, motion_end: float) -> list[dict[str, object]]:
    frame = pd.read_csv(path, usecols=["t", "joint_name", "position", "velocity"])
    rows: list[dict[str, object]] = []
    for joint in BALANCE_JOINTS:
        group = frame[frame.joint_name == joint].sort_values("t")
        if group.empty:
            continue
        pre = group[group.t.between(-3.0, -0.2)]
        motion = group[group.t.between(0.0, motion_end)]
        post = group[group.t.between(motion_end, motion_end + 3.0)]
        if pre.empty or motion.empty:
            continue
        baseline = float(pre.position.mean())
        relative = motion.position.to_numpy(float) - baseline
        times = motion.t.to_numpy(float)
        peak_index = int(np.argmax(np.abs(relative)))
        excursion = float(relative.max() - relative.min())
        tolerance = max(0.002, 0.10 * excursion)
        combined = pd.concat([motion, post]).drop_duplicates("t").sort_values("t")
        combined_relative = combined.position.to_numpy(float) - baseline
        rows.append(
            {
                "dataset": dataset,
                "joint_name": joint,
                "baseline_rad": baseline,
                "relative_excursion_rad": excursion,
                "peak_relative_rad": float(relative[peak_index]),
                "peak_time_s": float(times[peak_index]),
                "peak_sign": int(np.sign(relative[peak_index])),
                "peak_abs_velocity_rad_s": float(motion.velocity.abs().max()),
                "recovery_time_s": recovery_time(
                    combined.t.to_numpy(float), combined_relative, motion_end, tolerance
                ),
                "classification": "OUTPUT_RESPONSE_DESIGN_TARGET",
            }
        )
    return rows


def main() -> int:
    heart_lock = json.loads(
        (CALIBRATION / "phase2e_replay" / "source_data_lock.json").read_text(encoding="utf-8")
    )
    wave_lock = json.loads(
        (CALIBRATION / "phase3av_validation" / "phase3av_capture_metadata.json").read_text(encoding="utf-8")
    )
    rows = extract(
        "heart",
        CALIBRATION / "phase2e_replay" / "phase2e_aligned_joint_data.csv",
        float(heart_lock["motion_duration_seconds"]),
    )
    rows += extract(
        "wave_right",
        CALIBRATION / "phase3av_validation" / "phase3av_aligned_joint_data.csv",
        float(wave_lock["motion_duration_seconds"]),
    )
    frame = pd.DataFrame(rows)
    frame.to_csv(HERE / "phase3ar_real_balance_targets.csv", index=False)

    pivot = frame.pivot(index="joint_name", columns="dataset", values="relative_excursion_rad")
    lines = [
        "# Phase 3A-R real balance output-response targets",
        "",
        "These are `OUTPUT_RESPONSE_DESIGN_TARGET` measurements. They are not MC gain identification.",
        "",
        "| joint | heart excursion rad | wave excursion rad | wave/heart | heart peak sign | wave peak sign |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for joint in BALANCE_JOINTS:
        heart = frame[(frame.dataset == "heart") & (frame.joint_name == joint)].iloc[0]
        wave = frame[(frame.dataset == "wave_right") & (frame.joint_name == joint)].iloc[0]
        ratio = float(wave.relative_excursion_rad / heart.relative_excursion_rad) if heart.relative_excursion_rad else np.nan
        lines.append(
            f"| `{joint}` | {heart.relative_excursion_rad:.5f} | {wave.relative_excursion_rad:.5f} | {ratio:.3f} | {int(heart.peak_sign)} | {int(wave.peak_sign)} |"
        )
    lines += [
        "",
        "The two motions do not exhibit one fixed ankle/knee/hip/waist excursion ratio. "
        "This supports channel-specific allocation and hard safety constraints rather than a single global gain.",
    ]
    (HERE / "phase3ar_real_balance_targets.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(frame[["dataset", "joint_name", "relative_excursion_rad", "peak_time_s", "peak_sign"]].to_string(index=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
