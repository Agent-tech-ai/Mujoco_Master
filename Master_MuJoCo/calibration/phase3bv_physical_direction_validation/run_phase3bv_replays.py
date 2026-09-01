#!/usr/bin/env python3
"""Blind offline dual replay for Phase 3B-V.

Runs the identical frozen Phase 3A-Y controller against (A) the original
physical baseline and (B) the Phase 3B-S mass sensitivity direction. No robot,
reported effort, torque fitting, controller tuning, or source-MJCF write exists.
"""

from __future__ import annotations

from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys


HERE = Path(__file__).resolve().parent
CALIBRATION = HERE.parent
PROJECT = CALIBRATION.parent
WORKSPACE = PROJECT.parent
BS_DIR = CALIBRATION / "phase3bs_physical_sensitivity"
if str(BS_DIR) not in sys.path:
    sys.path.insert(0, str(BS_DIR))
from phase3bs_core import (  # noqa: E402
    AX, P3AR, Y, PhysicalExperiment, apply_runtime_override, ay_candidate,
)


RUNS = HERE / "runs"
BASELINE = PhysicalExperiment(
    "phase3bv_original_physical_baseline", "BASELINE", "none", "baseline", 1.0, 0.0,
    "Frozen original physical baseline",
    classification="BLIND_VALIDATION_BASELINE_NOT_HARDWARE_CALIBRATION",
)
MASS_DIRECTION = PhysicalExperiment(
    "phase3bv_bs_mass_lower_plus08", "MASS_DISTRIBUTION",
    "lower_limb_mass_scale_total_mass_preserved", "plus", 1.08, 0.08,
    "Phase 3B-S shared sensitivity direction; not an identified hardware parameter",
    classification="SHARED_PHYSICAL_SENSITIVITY_DIRECTION_NOT_IDENTIFIED_HARDWARE_PARAMETER",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def verify_manifest() -> None:
    manifest = HERE / "phase3bv_source_manifest.csv"
    if not manifest.exists():
        raise RuntimeError(f"missing source lock: {manifest}")
    import pandas as pd
    failures = []
    for row in pd.read_csv(manifest).itertuples(index=False):
        path = WORKSPACE / str(row.path)
        if not path.exists():
            failures.append(f"MISSING {row.path}")
        elif sha256(path) != str(row.sha256):
            failures.append(f"HASH_MISMATCH {row.path}")
    if failures:
        raise RuntimeError("Phase 3B-V source lock failed: " + "; ".join(failures))


def dataset_from_capture():
    metadata_path = HERE / "phase3bv_capture_metadata.json"
    independence_path = HERE / "phase3bv_independence.json"
    if not metadata_path.exists() or not independence_path.exists():
        raise FileNotFoundError("processed third-motion capture is missing")
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    independence = json.loads(independence_path.read_text(encoding="utf-8"))
    if not metadata.get("data_ready"):
        raise RuntimeError("capture quality gate is not READY")
    if independence.get("decision") != "SUFFICIENTLY_INDEPENDENT_FROM_HEART_AND_WAVE":
        raise RuntimeError("capture is not independent from both prior motions")
    return P3AR.Dataset(
        "phase3bv_clap",
        HERE / "phase3bv_measured_reference.csv",
        HERE / "phase3bv_aligned_joint_data.csv",
        float(metadata["motion_duration_seconds"]),
    )


def run_one(experiment: PhysicalExperiment, dataset, mode: str) -> dict:
    design = ay_candidate(experiment.experiment_id)
    old_load = P3AR.load_model
    old_runs = Y.RUNS
    audit: dict[str, object] = {}

    def loader(*, free_base: bool):
        model = old_load(free_base=free_base)
        audit.update(apply_runtime_override(model, experiment))
        return model

    P3AR.load_model = loader
    Y.RUNS = RUNS
    try:
        summary = Y.run_replay(design, dataset, mode, pre_s=5.0, post_s=5.0, save_detail=True)
    finally:
        P3AR.load_model = old_load
        Y.RUNS = old_runs
    summary.update({
        "physical_experiment": asdict(experiment),
        "runtime_override_audit": audit,
        "physical_override_scope": "DERIVED_RUNTIME_MODEL_ONLY",
        "source_mjcf_modified": False,
        "reported_effort_loaded": False,
        "robot_connected": False,
        "hardware_parameter_identified": False,
        "warning": "SHARED PHYSICAL SENSITIVITY DIRECTION; NOT IDENTIFIED HARDWARE PARAMETER",
    })
    path = RUNS / f"{experiment.experiment_id}__{dataset.name}__{mode}_summary.json"
    path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def main() -> int:
    verify_manifest()
    try:
        dataset = dataset_from_capture()
    except (FileNotFoundError, RuntimeError) as exc:
        print(json.dumps({
            "PHASE3BV_REPLAY_READY": "NO",
            "reason": str(exc),
            "robot_connected": False,
            "motion_invoked": False,
        }, indent=2))
        return 2
    RUNS.mkdir(parents=True, exist_ok=True)
    baseline_design = asdict(ay_candidate(BASELINE.experiment_id))
    candidate_design = asdict(ay_candidate(MASS_DIRECTION.experiment_id))
    baseline_design.pop("experiment_id", None)
    candidate_design.pop("experiment_id", None)
    if baseline_design != candidate_design:
        raise RuntimeError("controller designs differ between baseline and mass-direction replay")
    results = []
    for experiment in (BASELINE, MASS_DIRECTION):
        for mode in ("arm_only", "whole_body"):
            print(f"REPLAY {experiment.experiment_id} {mode}", flush=True)
            results.append(run_one(experiment, dataset, mode))
    payload = {
        "controller_config_identical": True,
        "source_mjcf_modified": False,
        "reported_effort_loaded": False,
        "robot_connected": False,
        "baseline": asdict(BASELINE),
        "mass_direction": asdict(MASS_DIRECTION),
        "results": [{
            "experiment_id": item["experiment_id"], "mode": item["mode"],
            "safety_pass": item["safety_pass"], "stable_no_fall": item["stable_no_fall"],
        } for item in results],
    }
    (HERE / "phase3bv_replay_execution.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
