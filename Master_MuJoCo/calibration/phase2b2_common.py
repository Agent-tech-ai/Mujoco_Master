"""Shared, evidence-labelled calculations for Phase 2B-2 offline preparation."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CALIBRATION_DIR = PROJECT_ROOT / "calibration"
DEFAULT_SNAPSHOT = CALIBRATION_DIR / "evidence" / "phase2b2_latest_arm_snapshot.json"

# Operator-supplied FIELD_TEST_EVIDENCE, 2026-08-11. These are hardware control
# coordinates in degrees. They are not proof of a MuJoCo axis/sign mapping.
FIELD_LIMITS_DEG: dict[str, tuple[float, float]] = {
    "left_shoulder_pitch_joint": (-176.471, 116.883),
    "left_shoulder_roll_joint": (-3.495, 171.486),
    "left_shoulder_yaw_joint": (-146.448, 146.448),
    "left_elbow_joint": (-134.965, 0.0),
    "left_wrist_yaw_joint": (-146.448, 146.448),
    "left_wrist_pitch_joint": (-31.971, 31.971),
    "left_wrist_roll_joint": (-90.012, 41.482),
    "right_shoulder_pitch_joint": (-176.471, 116.883),
    "right_shoulder_roll_joint": (-171.486, 3.495),
    "right_shoulder_yaw_joint": (-146.448, 146.448),
    "right_elbow_joint": (-134.965, 0.0),
    "right_wrist_yaw_joint": (-146.448, 146.448),
    "right_wrist_pitch_joint": (-31.971, 31.971),
    "right_wrist_roll_joint": (-41.482, 90.012),
}

AMPLITUDES_DEG = (1.0, 2.0, 3.0, 5.0)


@dataclass(frozen=True)
class MarginAssessment:
    name: str
    current_deg: float
    lower_deg: float
    upper_deg: float
    lower_distance_deg: float
    upper_distance_deg: float

    @property
    def minimum_distance_deg(self) -> float:
        return min(self.lower_distance_deg, self.upper_distance_deg)


def load_snapshot(path: Path = DEFAULT_SNAPSHOT) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        payload = json.load(stream)
    names = [joint["name"] for joint in payload["joints"]]
    if len(names) != 14 or len(set(names)) != 14:
        raise ValueError(f"expected 14 unique arm joints, got {names}")
    missing = sorted(set(FIELD_LIMITS_DEG) - set(names))
    if missing:
        raise ValueError(f"snapshot lacks field-limit joints: {missing}")
    return payload


def positions_rad(snapshot: dict) -> dict[str, float]:
    return {joint["name"]: float(joint["position"]) for joint in snapshot["joints"]}


def assessments(snapshot: dict) -> list[MarginAssessment]:
    result: list[MarginAssessment] = []
    for joint in snapshot["joints"]:
        name = joint["name"]
        current = math.degrees(float(joint["position"]))
        lower, upper = FIELD_LIMITS_DEG[name]
        result.append(
            MarginAssessment(
                name=name,
                current_deg=current,
                lower_deg=lower,
                upper_deg=upper,
                lower_distance_deg=current - lower,
                upper_distance_deg=upper - current,
            )
        )
    return result


def amplitude_status(
    assessment: MarginAssessment, amplitude_deg: float, reserve_deg: float
) -> str:
    plus = assessment.current_deg + amplitude_deg
    minus = assessment.current_deg - amplitude_deg
    if minus < assessment.lower_deg or plus > assessment.upper_deg:
        return "OUTSIDE_LIMIT"
    residual = min(
        minus - assessment.lower_deg,
        assessment.upper_deg - plus,
    )
    if residual + 1e-12 < reserve_deg:
        return "INSIDE_LIMIT_RESERVE_FAIL"
    return "PASS_GEOMETRIC_RESERVE"


def directional_status(
    assessment: MarginAssessment, signed_delta_deg: float, reserve_deg: float
) -> str:
    target = assessment.current_deg + signed_delta_deg
    if target < assessment.lower_deg or target > assessment.upper_deg:
        return "OUTSIDE_LIMIT"
    residual = min(target - assessment.lower_deg, assessment.upper_deg - target)
    if residual + 1e-12 < reserve_deg:
        return "INSIDE_LIMIT_RESERVE_FAIL"
    return "PASS_GEOMETRIC_RESERVE"


def adaptive_symmetric_amplitude(
    assessment: MarginAssessment,
    *,
    requested_deg: float,
    reserve_deg: float,
    minimum_useful_deg: float,
) -> tuple[float | None, str]:
    available = assessment.minimum_distance_deg - reserve_deg
    if available + 1e-12 < minimum_useful_deg:
        return None, (
            f"SKIP: symmetric available amplitude {available:.6f}° after "
            f"{reserve_deg:.3f}° reserve is below {minimum_useful_deg:.3f}°"
        )
    selected = min(requested_deg, available)
    return selected, (
        f"SELECT: min(requested={requested_deg:.3f}°, available={available:.6f}°)"
    )


def candidate_attributes(name: str) -> tuple[int, int, str]:
    """Return information value, low-impact score, and rationale (3 is best)."""

    if "wrist_roll" in name:
        return 3, 3, "J7 mirrored FIELD_TEST_EVIDENCE; distal, high sign information"
    if "shoulder_roll" in name:
        return 3, 1, "J2 mirrored FIELD_TEST_EVIDENCE; proximal and posture-sensitive"
    if "wrist_yaw" in name or "wrist_pitch" in name:
        return 2, 3, "distal joint; low expected whole-body influence"
    if "elbow" in name:
        return 2, 2, "moderate distal motion and broad current margin"
    if "shoulder_yaw" in name:
        return 2, 1, "proximal joint; collision envelope more posture-dependent"
    return 2, 1, "proximal shoulder pitch; larger whole-body/collision influence"


def ranked_candidates(
    snapshot: dict,
    *,
    requested_deg: float = 2.0,
    reserve_deg: float = 5.0,
    minimum_useful_deg: float = 1.0,
) -> list[dict]:
    rows: list[dict] = []
    for assessment in assessments(snapshot):
        selected, selection_reason = adaptive_symmetric_amplitude(
            assessment,
            requested_deg=requested_deg,
            reserve_deg=reserve_deg,
            minimum_useful_deg=minimum_useful_deg,
        )
        information, low_impact, rationale = candidate_attributes(assessment.name)
        # The score is only an offline ordering rubric, not a safety certification.
        clearance_component = min(assessment.minimum_distance_deg, 50.0) / 10.0
        score = information * 10.0 + low_impact * 5.0 + clearance_component
        rows.append(
            {
                "name": assessment.name,
                "assessment": assessment,
                "selected_amplitude_deg": selected,
                "selection_reason": selection_reason,
                "information_score": information,
                "low_impact_score": low_impact,
                "score": score,
                "rationale": rationale,
            }
        )
    return sorted(
        rows,
        key=lambda row: (
            row["selected_amplitude_deg"] is not None,
            row["score"],
            row["assessment"].minimum_distance_deg,
        ),
        reverse=True,
    )


def neutral_arm_pose_deg(snapshot: dict) -> dict[str, float]:
    """Minimal J2-only candidate that gives both J2 joints >=10° margin."""

    pose = {
        item.name: item.current_deg
        for item in assessments(snapshot)
    }
    pose["left_shoulder_roll_joint"] = 7.0
    pose["right_shoulder_roll_joint"] = -7.0
    return pose
