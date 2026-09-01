#!/usr/bin/env python3
"""Create the immutable input manifest for Phase 3A-R."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pandas as pd


HERE = Path(__file__).resolve().parent
CALIBRATION = HERE.parent
PROJECT = CALIBRATION.parent
WORKSPACE = PROJECT.parent


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def add_file(rows: list[dict[str, object]], category: str, path: Path) -> None:
    if not path.is_file():
        raise FileNotFoundError(path)
    rows.append(
        {
            "category": category,
            "path": str(path.relative_to(WORKSPACE)),
            "bytes": path.stat().st_size,
            "sha256": sha256(path),
        }
    )


def main() -> int:
    HERE.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, object]] = []

    phase2d = CALIBRATION / "logs" / "real" / "phase2d_heart_001"
    for path in sorted(phase2d.rglob("*")):
        if path.is_file():
            add_file(rows, "phase2d_heart_raw", path)

    phase2e = CALIBRATION / "phase2e_replay"
    for name in (
        "phase2e_heart_measured_reference.csv",
        "phase2e_aligned_joint_data.csv",
        "phase2e_aligned_imu_data.csv",
        "phase2e_joint_metrics.csv",
        "source_data_lock.json",
        "source_sha256_manifest.csv",
        "replay1_arm_only_joint_log.csv",
        "replay1_arm_only_base_log.csv",
        "replay1_arm_only_summary.json",
        "replay2_whole_body_joint_log.csv",
        "replay2_whole_body_base_log.csv",
        "replay2_whole_body_summary.json",
    ):
        add_file(rows, "phase2e_phase2f_heart", phase2e / name)

    phase3a = CALIBRATION / "phase3a_position_only"
    for name in (
        "simulation_controller_alignment_candidate.json",
        "run_phase3a_experiments.py",
        "free_final_candidate_joint_log.csv",
        "free_final_candidate_base_log.csv",
        "free_final_candidate_summary.json",
        "free_base_10s_standing_validation.json",
        "rehearsal_12_joint_regression.csv",
        "rehearsal_12_joint_regression.json",
        "phase3a_before_after_report.md",
    ):
        add_file(rows, "phase3a_candidate_and_validation", phase3a / name)

    phase3av = CALIBRATION / "phase3av_validation"
    for name in (
        "phase3av_measured_reference.csv",
        "phase3av_aligned_joint_data.csv",
        "phase3av_aligned_imu_data.csv",
        "phase3av_capture_metadata.json",
        "phase3av_joint_metrics.csv",
        "phase3av_independence.json",
        "phase3av_legacy_arm_only_joint_log.csv",
        "phase3av_legacy_arm_only_base_log.csv",
        "phase3av_legacy_arm_only_summary.json",
        "phase3av_candidate_arm_only_joint_log.csv",
        "phase3av_candidate_arm_only_base_log.csv",
        "phase3av_candidate_arm_only_summary.json",
        "phase3av_legacy_whole_body_joint_log.csv",
        "phase3av_legacy_whole_body_base_log.csv",
        "phase3av_legacy_whole_body_summary.json",
        "phase3av_candidate_whole_body_joint_log.csv",
        "phase3av_candidate_whole_body_base_log.csv",
        "phase3av_candidate_whole_body_summary.json",
        "phase3av_replay_lock.json",
        "phase3av_final_gate.md",
    ):
        add_file(rows, "phase3av_wave_and_validation", phase3av / name)

    for name in (
        "ff_master_ultra.xml",
        "ff_master_ultra_x2_limits.xml",
        "scene_x2_fixed.xml",
        "scene_x2_free.xml",
    ):
        add_file(rows, "immutable_mjcf", PROJECT / "assets" / "Master" / name)
    for name in ("controller.py", "model.py"):
        add_file(rows, "simulation_controller_source", PROJECT / "master_sim" / name)
    add_file(rows, "hardware_mapping_read_only", CALIBRATION / "joint_mapping.csv")

    manifest = pd.DataFrame(rows).sort_values(["category", "path"])
    manifest.to_csv(HERE / "phase3ar_source_manifest.csv", index=False)
    lock = {
        "status": "PHASE3AR_SOURCE_LOCKED",
        "files": len(manifest),
        "warning": "SIMULATION CONTROLLER DESIGN; NOT HARDWARE CALIBRATION",
        "phase2_or_phase3_inputs_overwritten": False,
        "mjcf_modified": False,
        "hardware_mapping_modified": False,
        "reported_effort_used_for_fitting": False,
        "physical_parameters_modified": False,
        "manifest": "phase3ar_source_manifest.csv",
    }
    (HERE / "phase3ar_source_lock.json").write_text(json.dumps(lock, indent=2), encoding="utf-8")
    print(json.dumps(lock, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
