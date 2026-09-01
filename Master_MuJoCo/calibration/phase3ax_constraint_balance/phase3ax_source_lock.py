#!/usr/bin/env python3
"""Freeze and verify every input used by Phase 3A-X.

The manifest extends the already verified Phase 3A-R lock.  Generated Phase
3A-X files are deliberately excluded so experiments can be reproduced without
changing the immutable-input record.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pandas as pd


HERE = Path(__file__).resolve().parent
CALIBRATION = HERE.parent
PROJECT = CALIBRATION.parent
WORKSPACE = PROJECT.parent
P3AR = CALIBRATION / "phase3ar_controller_redesign"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def add(rows: list[dict[str, object]], category: str, path: Path) -> None:
    if not path.is_file():
        raise FileNotFoundError(path)
    rows.append({
        "category": category,
        "path": str(path.relative_to(WORKSPACE)),
        "bytes": path.stat().st_size,
        "sha256": sha256(path),
    })


def main() -> int:
    HERE.mkdir(parents=True, exist_ok=True)
    prior = pd.read_csv(P3AR / "phase3ar_source_manifest.csv")
    rows: list[dict[str, object]] = []
    prior_verification: list[dict[str, object]] = []
    for row in prior.itertuples(index=False):
        path = WORKSPACE / str(row.path)
        current = sha256(path) if path.is_file() else "MISSING"
        prior_verification.append({
            "path": row.path,
            "expected_sha256": row.sha256,
            "actual_sha256": current,
            "status": "VERIFIED_UNCHANGED" if current == row.sha256 else "CHANGED",
        })
        add(rows, f"phase3ar_inherited::{row.category}", path)

    phase3ar_files = (
        "phase3ar_core.py",
        "simulation_controller_robustness_candidate.json",
        "phase3ar_final_validation_summary.json",
        "phase3ar_contact_timeline.csv",
        "phase3ar_experiments.csv",
        "phase3ar_real_balance_targets.csv",
        "phase3ar_final_gate.md",
        "rehearsal_12_joint_regression.csv",
        "rehearsal_12_joint_regression.json",
    )
    for name in phase3ar_files:
        add(rows, "phase3ar_architecture_and_results", P3AR / name)

    verification = pd.DataFrame(prior_verification)
    verification.to_csv(HERE / "phase3ax_inherited_source_verification.csv", index=False)
    if not (verification.status == "VERIFIED_UNCHANGED").all():
        raise RuntimeError("A Phase 3A-R inherited source changed")

    manifest = pd.DataFrame(rows).drop_duplicates("path").sort_values(["category", "path"])
    manifest.to_csv(HERE / "phase3ax_source_manifest.csv", index=False)
    lock = {
        "status": "PHASE3AX_SOURCE_LOCKED",
        "files": int(len(manifest)),
        "inherited_phase3ar_verification": f"{len(verification)}/{len(verification)} VERIFIED_UNCHANGED",
        "arm_tracking_status": "INDEPENDENTLY_VALIDATED_ARM_TRACKING",
        "warning": "SIMULATION CONTROLLER DESIGN; NOT HARDWARE CALIBRATION",
        "robot_connected": False,
        "reported_effort_used": False,
        "mjcf_modified": False,
        "physical_parameters_modified": False,
        "hardware_mapping_modified": False,
    }
    (HERE / "phase3ax_source_lock.json").write_text(json.dumps(lock, indent=2), encoding="utf-8")
    print(json.dumps(lock, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
