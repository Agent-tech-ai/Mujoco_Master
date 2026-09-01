#!/usr/bin/env python3
"""Read-only offline contact-pair diagnosis for the frozen Phase 3A-V replay.

This diagnostic reruns the already frozen candidate arm-only simulation while
observing MuJoCo contacts.  It does not change candidate parameters, MJCF,
hardware mapping, or any robot state.
"""

from __future__ import annotations

import csv
import json
from collections import defaultdict
from dataclasses import replace
from pathlib import Path

import mujoco
import pandas as pd

import run_phase3av_replays as phase3av


HERE = Path(__file__).resolve().parent
DIAGNOSTICS = HERE / "diagnostics"


def main() -> int:
    phase3av.verify_frozen_sources()
    metadata = json.loads((HERE / "phase3av_capture_metadata.json").read_text(encoding="utf-8"))
    candidate_json = json.loads(
        (phase3av.PHASE3A / "simulation_controller_alignment_candidate.json").read_text(encoding="utf-8")
    )
    reference = pd.read_csv(
        HERE / "phase3av_measured_reference.csv",
        usecols=["t", "joint_name", "joint_group", "position", "velocity"],
    )
    real_joint = pd.read_csv(
        HERE / "phase3av_aligned_joint_data.csv",
        usecols=["t", "joint_name", "joint_group", "position", "velocity"],
    )
    classification = pd.read_csv(
        HERE / "phase3av_joint_metrics.csv",
        usecols=["joint_name", "joint_group", "classification"],
    )

    runner = phase3av.load_runner()
    DIAGNOSTICS.mkdir(exist_ok=True)
    runner.HERE = DIAGNOSTICS
    runner.MOTION_END = float(metadata["motion_duration_seconds"])
    runner.PRE_WINDOW = (-3.0, -0.2)
    runner.POST_WINDOW = (runner.MOTION_END + 0.5, runner.MOTION_END + 3.0)

    frozen = runner.Experiment(**candidate_json["parameters"])
    experiment = replace(
        frozen,
        name="phase3av_candidate_arm_only_contact_diagnostic",
        parent="PHASE3A_FROZEN_CANDIDATE",
        changed_category="OBSERVATION_ONLY_CONTACT_DIAGNOSTIC",
        classification="DIAGNOSTIC_ONLY",
    )
    offsets = {str(key): float(value) for key, value in candidate_json["standing_reference_offsets_rad"].items()}
    arm_joints = set(classification.loc[classification.joint_group == "arm", "joint_name"])

    original_contact_state = runner.contact_state
    observations: dict[tuple[str, str, str, str], dict[str, object]] = defaultdict(
        lambda: {"sample_times": set(), "first_time_s": float("inf"), "last_time_s": 0.0, "maximum_penetration_m": 0.0}
    )

    def observed_contact_state(model: mujoco.MjModel, data: mujoco.MjData):
        state = original_contact_state(model, data)
        floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        sample_time = round(float(data.time), 6)
        for index in range(data.ncon):
            contact = data.contact[index]
            g1, g2 = int(contact.geom1), int(contact.geom2)
            if g1 == floor or g2 == floor:
                continue
            b1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g1])) or "UNNAMED_BODY"
            b2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g2])) or "UNNAMED_BODY"
            if b1 == b2:
                continue
            geom1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g1) or f"geom_{g1}"
            geom2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g2) or f"geom_{g2}"
            left = (b1, geom1)
            right = (b2, geom2)
            if right < left:
                left, right = right, left
            key = (left[0], left[1], right[0], right[1])
            row = observations[key]
            row["sample_times"].add(sample_time)
            row["first_time_s"] = min(float(row["first_time_s"]), sample_time)
            row["last_time_s"] = max(float(row["last_time_s"]), sample_time)
            row["maximum_penetration_m"] = max(
                float(row["maximum_penetration_m"]), max(0.0, -float(contact.dist))
            )
        return state

    runner.contact_state = observed_contact_state
    summary = runner.run_replay(
        experiment,
        reference,
        real_joint,
        arm_joints,
        inherited_offsets=offsets,
    )

    rows = []
    for (body1, geom1, body2, geom2), values in observations.items():
        rows.append(
            {
                "body_1": body1,
                "geom_1": geom1,
                "body_2": body2,
                "geom_2": geom2,
                "sample_count": len(values["sample_times"]),
                "first_sim_time_s": values["first_time_s"],
                "last_sim_time_s": values["last_time_s"],
                "maximum_penetration_m": values["maximum_penetration_m"],
            }
        )
    rows.sort(key=lambda item: (-int(item["sample_count"]), item["body_1"], item["body_2"]))
    output_csv = DIAGNOSTICS / "phase3av_candidate_arm_only_contact_pairs.csv"
    with output_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]) if rows else ["body_1", "geom_1", "body_2", "geom_2", "sample_count", "first_sim_time_s", "last_sim_time_s", "maximum_penetration_m"])
        writer.writeheader()
        writer.writerows(rows)

    result = {
        "classification": "OFFLINE_DIAGNOSTIC_ONLY",
        "candidate_parameters_changed": False,
        "robot_connected": False,
        "contact_pair_count": len(rows),
        "summary": summary,
        "contact_pairs_csv": str(output_csv),
    }
    (DIAGNOSTICS / "phase3av_contact_diagnostic.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps({"contact_pair_count": len(rows), "self_collision_samples": summary["self_collision_samples"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
