#!/usr/bin/env python3
"""Run frozen legacy/candidate Phase 3A-V replays after the capture gate passes.

No optimization is implemented here. The Phase 3A candidate is loaded from its
locked JSON and the legacy configuration is fixed at the pre-Phase-3A values.
Only position/velocity columns are loaded from real data.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import sys
from dataclasses import replace
from pathlib import Path

import pandas as pd


HERE = Path(__file__).resolve().parent
CALIBRATION = HERE.parent
PROJECT = CALIBRATION.parent
WORKSPACE = PROJECT.parent
PHASE3A = CALIBRATION / "phase3a_position_only"
MANIFEST = HERE / "phase3av_frozen_source_manifest.csv"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_frozen_sources() -> pd.DataFrame:
    manifest = pd.read_csv(MANIFEST)
    records = []
    for row in manifest.itertuples(index=False):
        path = WORKSPACE / Path(row.path)
        current = sha256(path) if path.exists() else "MISSING"
        records.append({"path": row.path, "locked_sha256": row.sha256, "current_sha256": current, "status": "VERIFIED_UNCHANGED" if current == row.sha256 else "CHANGED_OR_MISSING"})
    result = pd.DataFrame(records)
    result.to_csv(HERE / "phase3av_source_verification.csv", index=False)
    if not (result.status == "VERIFIED_UNCHANGED").all():
        bad = result[result.status != "VERIFIED_UNCHANGED"].path.tolist()
        raise RuntimeError(f"Phase 3A-V frozen source mismatch: {bad}")
    return result


def load_runner():
    path = PHASE3A / "run_phase3a_experiments.py"
    spec = importlib.util.spec_from_file_location("phase3av_frozen_phase3a_runner", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load frozen replay implementation: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def main() -> int:
    verification = verify_frozen_sources()
    metadata_path = HERE / "phase3av_capture_metadata.json"
    if not metadata_path.exists():
        raise SystemExit("PHASE3AV_VALIDATION_DATA_READY = NO: capture metadata missing")
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if not metadata.get("data_ready"):
        raise SystemExit("PHASE3AV_VALIDATION_DATA_READY = NO: replay prohibited")

    # The usecols are an explicit guard: reported_effort is never loaded here.
    reference = pd.read_csv(HERE / "phase3av_measured_reference.csv", usecols=["t", "joint_name", "joint_group", "position", "velocity"])
    real_joint = pd.read_csv(HERE / "phase3av_aligned_joint_data.csv", usecols=["t", "joint_name", "joint_group", "position", "velocity"])
    classification = pd.read_csv(HERE / "phase3av_joint_metrics.csv", usecols=["joint_name", "joint_group", "classification"])
    candidate_json = json.loads((PHASE3A / "simulation_controller_alignment_candidate.json").read_text(encoding="utf-8"))

    runner = load_runner()
    runner.HERE = HERE
    runner.MOTION_END = float(metadata["motion_duration_seconds"])
    runner.PRE_WINDOW = (-3.0, -0.2)
    runner.POST_WINDOW = (runner.MOTION_END + 0.5, runner.MOTION_END + 3.0)

    legacy = runner.Experiment(
        name="phase3av_legacy_arm_only",
        parent="PHASE3A_PRE_ALIGNMENT_BASELINE",
        free_base=True,
        changed_category="FROZEN_LEGACY_BASELINE",
        classification="DIAGNOSTIC_ONLY",
        interpolation="linear",
        reference_rate_hz=50.0,
        controller_rate_hz=1000.0,
        timestep_s=0.001,
        shoulder_gain_scale=1.0,
        wrist_gain_scale=1.0,
        balance_gain_scale=1.0,
        standing_reference_scale=0.0,
        velocity_limit_rad_s=None,
    )
    frozen = runner.Experiment(**candidate_json["parameters"])
    candidate = replace(
        frozen,
        name="phase3av_candidate_arm_only",
        parent="PHASE3A_FROZEN_CANDIDATE",
        free_base=True,
        changed_category="NO_CHANGE_BLIND_VALIDATION",
        classification="VALIDATION_ONLY",
    )
    legacy_whole = replace(legacy, name="phase3av_legacy_whole_body", changed_category="WHOLE_BODY_INFRASTRUCTURE_CHECK")
    candidate_whole = replace(candidate, name="phase3av_candidate_whole_body", changed_category="WHOLE_BODY_INFRASTRUCTURE_CHECK")

    arm_joints = set(classification.loc[classification.joint_group == "arm", "joint_name"])
    all_joints = set(classification.joint_name)
    offsets = {str(key): float(value) for key, value in candidate_json["standing_reference_offsets_rad"].items()}
    results = []
    for experiment, controlled, inherited in (
        (legacy, arm_joints, None),
        (candidate, arm_joints, offsets),
        (legacy_whole, all_joints, None),
        (candidate_whole, all_joints, offsets),
    ):
        print(f"RUN {experiment.name}", flush=True)
        summary = runner.run_replay(experiment, reference, real_joint, controlled, inherited_offsets=inherited)
        results.append(summary)

    lock = {
        "phase": "3A-V",
        "purpose": "BLIND_VALIDATION_NO_OPTIMIZATION",
        "source_verification": f"{int((verification.status == 'VERIFIED_UNCHANGED').sum())}/{len(verification)}",
        "reported_effort_loaded": False,
        "candidate_parameters_changed": False,
        "legacy_arm_only": legacy.name,
        "candidate_arm_only": candidate.name,
        "legacy_whole_body": legacy_whole.name,
        "candidate_whole_body": candidate_whole.name,
        "motion_duration_seconds": runner.MOTION_END,
        "experiments": results,
    }
    (HERE / "phase3av_replay_lock.json").write_text(json.dumps(lock, indent=2), encoding="utf-8")
    print(json.dumps({"replays_complete": [item["name"] for item in results], "reported_effort_loaded": False}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
