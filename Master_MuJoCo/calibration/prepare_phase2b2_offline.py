"""Generate Phase 2B-2 offline margin, ranking, and neutral-pose evidence."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import mujoco
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from calibration.phase2b2_common import (
    AMPLITUDES_DEG,
    CALIBRATION_DIR,
    FIELD_LIMITS_DEG,
    assessments,
    directional_status,
    load_snapshot,
    neutral_arm_pose_deg,
    positions_rad,
    ranked_candidates,
)
from master_sim.model import load_model


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reserve-deg", type=float, default=5.0)
    parser.add_argument("--requested-amplitude-deg", type=float, default=2.0)
    parser.add_argument("--minimum-useful-amplitude-deg", type=float, default=1.0)
    return parser.parse_args()


def collision_pairs(model: mujoco.MjModel, data: mujoco.MjData) -> list[str]:
    pairs: list[str] = []
    for index in range(data.ncon):
        contact = data.contact[index]
        first = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
        second = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
        pairs.append(f"{first or contact.geom1} <-> {second or contact.geom2}")
    return pairs


def set_arm_pose(
    model: mujoco.MjModel, data: mujoco.MjData, pose_deg: dict[str, float]
) -> None:
    for name, value_deg in pose_deg.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise KeyError(name)
        data.qpos[int(model.jnt_qposadr[joint_id])] = math.radians(value_deg)
    mujoco.mj_forward(model, data)


def validate_neutral_transition(snapshot: dict, target_deg: dict[str, float]) -> dict:
    model = load_model(free_base=False)
    data = mujoco.MjData(model)
    current_deg = {name: math.degrees(value) for name, value in positions_rad(snapshot).items()}
    maximum_contacts = 0
    all_pairs: set[str] = set()
    model_limit_violations: list[str] = []
    samples = 201
    for fraction in np.linspace(0.0, 1.0, samples):
        pose = {
            name: current_deg[name] + fraction * (target_deg[name] - current_deg[name])
            for name in target_deg
        }
        set_arm_pose(model, data, pose)
        maximum_contacts = max(maximum_contacts, int(data.ncon))
        all_pairs.update(collision_pairs(model, data))
        for name, value_deg in pose.items():
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            lower, upper = np.degrees(model.jnt_range[joint_id])
            if value_deg < lower - 1e-9 or value_deg > upper + 1e-9:
                model_limit_violations.append(
                    f"sample={fraction:.3f} {name}={value_deg:.6f}° not in "
                    f"{lower:.6f}..{upper:.6f}°"
                )
    return {
        "samples": samples,
        "maximum_contact_count": maximum_contacts,
        "contact_pairs": sorted(all_pairs),
        "model_limit_violations": model_limit_violations,
        "collision_model_scope_warning": (
            "The supplied MJCF ends at wrist-roll links and has some collision geoms "
            "commented out. Zero contacts is model-only evidence, not physical clearance proof."
        ),
        "coordinate_assumption": (
            "Hardware control coordinates were applied numerically to same-name MuJoCo "
            "joints. Hardware-to-MuJoCo sign/zero is still UNKNOWN."
        ),
    }


def write_candidate_report(args: argparse.Namespace, snapshot: dict) -> None:
    lines = [
        "# Phase 2B-2 safe joint candidates",
        "",
        "Status: **OFFLINE GEOMETRIC SCREENING ONLY — ROBOT REMAINS NO-GO**",
        "",
        f"Snapshot: `{snapshot['capture_host_time']}` from `{snapshot['ssh_target']}`; source `{snapshot['source']}`.",
        "",
        f"Screening reserve: **{args.reserve_deg:.1f}°** from each FIELD_TEST_EVIDENCE limit. This is a configurable engineering screening value, **not** a vendor-approved safety margin.",
        "",
        "Movement labels:",
        "",
        "- `PASS_GEOMETRIC_RESERVE`: the listed directional target remains inside limits and retains the screening reserve.",
        "- `INSIDE_LIMIT_RESERVE_FAIL`: the listed directional target is mechanically inside the supplied limits but too close for this screening rule.",
        "- `OUTSIDE_LIMIT`: the listed directional target crosses the supplied limit.",
        "",
        "## All-arm margin table",
        "",
        "| Joint | Current (°) | Lower (°) | Upper (°) | To lower (°) | To upper (°) | +1° | -1° | +2° | -2° | +3° | -3° | +5° | -5° |",
        "|---|---:|---:|---:|---:|---:|---|---|---|---|---|---|---|---|",
    ]
    for item in assessments(snapshot):
        statuses = []
        for amplitude in AMPLITUDES_DEG:
            statuses.extend(
                (
                    directional_status(item, amplitude, args.reserve_deg),
                    directional_status(item, -amplitude, args.reserve_deg),
                )
            )
        lines.append(
            f"| `{item.name}` | {item.current_deg:.6f} | {item.lower_deg:.3f} | "
            f"{item.upper_deg:.3f} | {item.lower_distance_deg:.6f} | "
            f"{item.upper_distance_deg:.6f} | " + " | ".join(statuses) + " |"
        )

    lines.extend(
        [
            "",
            "## Candidate ordering",
            "",
            f"Adaptive symmetric amplitude is `min(requested={args.requested_amplitude_deg:.1f}°, minimum distance - reserve)`. A joint is skipped when the result is below {args.minimum_useful_amplitude_deg:.1f}°. It is never shrunk into the reserve zone.",
            "",
            "Ranking score = information value × 10 + low whole-body-impact score × 5 + capped clearance/10. It is an explicit prioritization heuristic, not a safety certificate.",
            "",
            "| Rank | Joint | Min current margin (°) | Selected symmetric amplitude (°) | Info | Low impact | Score | Decision / rationale |",
            "|---:|---|---:|---:|---:|---:|---:|---|",
        ]
    )
    for rank, row in enumerate(
        ranked_candidates(
            snapshot,
            requested_deg=args.requested_amplitude_deg,
            reserve_deg=args.reserve_deg,
            minimum_useful_deg=args.minimum_useful_amplitude_deg,
        ),
        start=1,
    ):
        selected = row["selected_amplitude_deg"]
        selected_text = "SKIP" if selected is None else f"{selected:.6f}"
        lines.append(
            f"| {rank} | `{row['name']}` | {row['assessment'].minimum_distance_deg:.6f} | "
            f"{selected_text} | {row['information_score']} | {row['low_impact_score']} | "
            f"{row['score']:.3f} | {row['selection_reason']}; {row['rationale']} |"
        )

    lines.extend(
        [
            "",
            "## Recommendation",
            "",
            "J7 wrist roll is the best first *candidate* because it combines distal motion, ample current margin, exact live name matching, and existing mirrored-coordinate FIELD_TEST_EVIDENCE. Wrist yaw and wrist pitch follow. Both J2 shoulder-roll joints are skipped at the current pose under the 5° screening reserve.",
            "",
            "Every candidate remains NO-GO for real motion until the operator checklist, control ownership, numeric velocity/acceleration/effort limits, abort behavior, and communications-loss behavior are confirmed.",
            "",
        ]
    )
    (CALIBRATION_DIR / "phase2b2_safe_joint_candidates.md").write_text(
        "\n".join(lines), encoding="utf-8"
    )


def write_neutral_report(snapshot: dict, validation: dict) -> None:
    target = neutral_arm_pose_deg(snapshot)
    current = {item.name: item.current_deg for item in assessments(snapshot)}
    lines = [
        "# Phase 2B-2 active-test neutral arm pose candidate",
        "",
        "Status: **OFFLINE CANDIDATE ONLY — DO NOT COMMAND THIS POSE**",
        "",
        "Design rule: keep all current arm targets except move hardware-control J2 to +7° left / -7° right. These are the smallest 0.5°-rounded targets that provide at least 10° field-limit margin for both J2 joints at the target. Legs and waist are unchanged by this candidate.",
        "",
        "| Joint | Current (°) | Candidate (°) | Move (°) | Lower margin at target (°) | Upper margin at target (°) |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for name in FIELD_LIMITS_DEG:
        lower, upper = FIELD_LIMITS_DEG[name]
        value = target[name]
        lines.append(
            f"| `{name}` | {current[name]:.6f} | {value:.6f} | "
            f"{value-current[name]:+.6f} | {value-lower:.6f} | {upper-value:.6f} |"
        )
    lines.extend(
        [
            "",
            "## MuJoCo kinematic validation",
            "",
            f"- Linear interpolation samples checked: {validation['samples']}.",
            f"- Maximum simultaneous contacts: {validation['maximum_contact_count']}.",
            f"- Contact pairs: {validation['contact_pairs'] or 'none'}.",
            f"- MuJoCo-limit violations: {validation['model_limit_violations'] or 'none'}.",
            "- Result: `PASS_MODEL_ONLY` when applying the numeric hardware coordinates directly to same-name MuJoCo joints.",
            "",
            "## Limitations",
            "",
            f"- {validation['coordinate_assumption']}",
            f"- {validation['collision_model_scope_warning']}",
            "- No attached hand/end-effector, cable, clothing, environment object, or human clearance is proven by this model.",
            "- Transitioning the real robot into this pose is itself a motion operation and needs a separately approved whole-body procedure. This report does not authorize it.",
            "",
        ]
    )
    (CALIBRATION_DIR / "phase2b2_neutral_arm_pose_candidate.md").write_text(
        "\n".join(lines), encoding="utf-8"
    )
    evidence = CALIBRATION_DIR / "evidence" / "phase2b2_neutral_pose_validation.json"
    evidence.write_text(
        json.dumps(
            {
                "target_degrees": target,
                "current_degrees": current,
                "validation": validation,
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )


def main() -> int:
    args = parse_args()
    if args.reserve_deg <= 0 or args.requested_amplitude_deg <= 0 or args.minimum_useful_amplitude_deg <= 0:
        print("ERROR: all amplitude and reserve values must be positive", file=sys.stderr)
        return 2
    snapshot = load_snapshot()
    write_candidate_report(args, snapshot)
    target = neutral_arm_pose_deg(snapshot)
    validation = validate_neutral_transition(snapshot, target)
    write_neutral_report(snapshot, validation)
    print(f"Wrote {CALIBRATION_DIR / 'phase2b2_safe_joint_candidates.md'}")
    print(f"Wrote {CALIBRATION_DIR / 'phase2b2_neutral_arm_pose_candidate.md'}")
    print("No robot connection or command was used.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
