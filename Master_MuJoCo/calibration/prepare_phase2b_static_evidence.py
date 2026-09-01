"""Record operator-supplied arm evidence and compare it with the current X2 MJCF.

This is a report-only/static-data tool.  It never edits an MJCF and never
connects to a robot.  The only project data it updates is joint_mapping.csv,
where field-test evidence is kept separate from documented hardware limits.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CALIBRATION = PROJECT_ROOT / "calibration"
MAPPING = CALIBRATION / "joint_mapping.csv"
MJCF = PROJECT_ROOT / "assets" / "Master" / "ff_master_ultra_x2_limits.xml"

OFFICIAL_JOINT_DOC = (
    "https://x2-aimdk.agibot.com/en/v0.9.0/Interface/control_mod/"
    "joint_control.html"
)
FIELD_SOURCE = (
    "FIELD_TEST_EVIDENCE: operator-supplied soft-engineer arm limits and "
    "Agentech.heart() endpoint observations, 2026-08-11"
)


@dataclass(frozen=True)
class FieldArmEvidence:
    minimum_deg: float
    maximum_deg: float
    relation: str
    endpoint_deg: float | None = None


# These values are transcribed exactly from the Phase 2B-1 request.  They are
# control-coordinate evidence, not proof of a MuJoCo axis direction.
FIELD_EVIDENCE: dict[str, FieldArmEvidence] = {
    "left_shoulder_pitch": FieldArmEvidence(-176.471, 116.883, "NOT_ESTABLISHED"),
    "right_shoulder_pitch": FieldArmEvidence(-176.471, 116.883, "NOT_ESTABLISHED"),
    "left_shoulder_roll": FieldArmEvidence(-3.495, 171.486, "LEFT_RIGHT_MIRRORED", 126.042),
    "right_shoulder_roll": FieldArmEvidence(-171.486, 3.495, "LEFT_RIGHT_MIRRORED", -126.042),
    "left_shoulder_yaw": FieldArmEvidence(-146.448, 146.448, "NOT_ESTABLISHED"),
    "right_shoulder_yaw": FieldArmEvidence(-146.448, 146.448, "NOT_ESTABLISHED"),
    "left_elbow": FieldArmEvidence(-134.965, 0.0, "NOT_ESTABLISHED"),
    "right_elbow": FieldArmEvidence(-134.965, 0.0, "NOT_ESTABLISHED"),
    "left_wrist_yaw": FieldArmEvidence(-146.448, 146.448, "NOT_ESTABLISHED"),
    "right_wrist_yaw": FieldArmEvidence(-146.448, 146.448, "NOT_ESTABLISHED"),
    "left_wrist_pitch": FieldArmEvidence(-31.971, 31.971, "NOT_ESTABLISHED"),
    "right_wrist_pitch": FieldArmEvidence(-31.971, 31.971, "NOT_ESTABLISHED"),
    "left_wrist_roll": FieldArmEvidence(-90.012, 41.482, "LEFT_RIGHT_MIRRORED", -63.021),
    "right_wrist_roll": FieldArmEvidence(-41.482, 90.012, "LEFT_RIGHT_MIRRORED", 63.021),
}

NEW_COLUMNS = [
    "field_test_min",
    "field_test_max",
    "field_test_unit",
    "field_test_coordinate_relation",
    "field_test_evidence_source",
    "array_index_status",
    "array_index_evidence_source",
    "hardware_mujoco_mapping_status",
    "hardware_mujoco_mapping_evidence_source",
]


def fmt(value: float) -> str:
    return f"{value:.6f}".rstrip("0").rstrip(".")


def load_mjcf_arm() -> dict[str, dict[str, str]]:
    root = ET.parse(MJCF).getroot()
    actuator_by_joint: dict[str, tuple[str, str]] = {}
    actuator_root = root.find("actuator")
    if actuator_root is not None:
        for actuator in actuator_root:
            joint = actuator.get("joint")
            if joint:
                actuator_by_joint[joint] = (
                    actuator.get("name", "UNKNOWN"),
                    actuator.get("ctrlrange", "UNKNOWN"),
                )

    result: dict[str, dict[str, str]] = {}
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError(f"No worldbody in {MJCF}")

    def visit(body: ET.Element) -> None:
        for joint in body.findall("joint"):
            name = joint.get("name", "")
            hardware_name = name.removesuffix("_joint")
            if hardware_name in FIELD_EVIDENCE:
                actuator_name, ctrlrange = actuator_by_joint.get(
                    name, ("NONE", "UNKNOWN")
                )
                result[hardware_name] = {
                    "joint": name,
                    "body": body.get("name", "UNKNOWN"),
                    "axis": joint.get("axis", "0 0 1"),
                    "range": joint.get("range", "UNKNOWN"),
                    "actuator": actuator_name,
                    "ctrlrange": ctrlrange,
                }
        for child in body.findall("body"):
            visit(child)

    for body in worldbody.findall("body"):
        visit(body)
    return result


def update_mapping() -> None:
    with MAPPING.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        fieldnames = list(reader.fieldnames or [])
    for column in NEW_COLUMNS:
        if column not in fieldnames:
            fieldnames.append(column)

    for row in rows:
        for column in NEW_COLUMNS:
            row.setdefault(column, "UNKNOWN")
        evidence = FIELD_EVIDENCE.get(row["hardware_joint_name"])
        if evidence is not None:
            row["field_test_min"] = fmt(evidence.minimum_deg)
            row["field_test_max"] = fmt(evidence.maximum_deg)
            row["field_test_unit"] = "deg"
            row["field_test_coordinate_relation"] = evidence.relation
            row["field_test_evidence_source"] = FIELD_SOURCE
            row["hardware_mujoco_mapping_status"] = "INFERRED_CANDIDATE"
            row["hardware_mujoco_mapping_evidence_source"] = (
                "Semantic joint-name correspondence plus FIELD_TEST_EVIDENCE; "
                "no hardware-to-MuJoCo motion correlation"
            )
        if not row.get("array_index_evidence_source") or row["array_index_evidence_source"] == "UNKNOWN":
            row["array_index_status"] = "DOCUMENTED_NOT_LIVE_VERIFIED"
            row["array_index_evidence_source"] = (
                f"AimDK X2 v0.9.0 documented JointStateArray order: {OFFICIAL_JOINT_DOC}; "
                "awaiting decoded live array"
            )

    with MAPPING.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def write_arm_evidence_report() -> None:
    lines = [
        "# Arm control coordinate evidence",
        "",
        "## Evidence scope",
        "",
        "The values below were supplied by the operator as on-site soft-engineer data. "
        "They are recorded as `FIELD_TEST_EVIDENCE`; they do not confirm a MuJoCo axis, "
        "hardware zero, encoder offset, or hardware-to-MuJoCo sign.",
        "",
        f"- Source: `{FIELD_SOURCE}`",
        "- No robot movement was requested or performed by this calibration workflow.",
        "",
        "| Joint | Side | field minimum (deg) | field maximum (deg) | observed endpoint (deg) | relation |",
        "|---|---|---:|---:|---:|---|",
    ]
    for name, evidence in FIELD_EVIDENCE.items():
        side, joint = name.split("_", 1)
        endpoint = "—" if evidence.endpoint_deg is None else fmt(evidence.endpoint_deg)
        lines.append(
            f"| {joint} | {side} | {fmt(evidence.minimum_deg)} | "
            f"{fmt(evidence.maximum_deg)} | {endpoint} | {evidence.relation} |"
        )
    lines.extend(
        [
            "",
            "## FIELD_TEST_EVIDENCE",
            "",
            "- J2 shoulder-roll endpoint: left `+126.042°`, right `-126.042°`.",
            "- J7 wrist-roll endpoint: left `-63.021°`, right `+63.021°`.",
            "- Therefore the left/right hardware control-coordinate signs are mirrored for J2 and J7.",
            "",
            "## UNKNOWN",
            "",
            "- Whether each hardware coordinate has sign `+1` or `-1` relative to its MuJoCo joint.",
            "- Hardware zero, encoder offset, and the physical pose at coordinate zero.",
            "- Whether the field limits are firmware-enforced, soft limits, or a UI/control-layer limit.",
            "",
            "## NEEDS_PHYSICAL_VERIFICATION",
            "",
            "- Correlate passive/operational logs with an independently observed physical joint direction in a later authorized phase.",
            "- Verify exact limit behavior without commanding a limit approach during Phase 2B-1.",
            "",
        ]
    )
    (CALIBRATION / "arm_control_coordinate_evidence.md").write_text(
        "\n".join(lines), encoding="utf-8"
    )


def write_comparison_report(mjcf: dict[str, dict[str, str]]) -> None:
    lines = [
        "# Hardware vs current MuJoCo arm coordinates",
        "",
        f"- MJCF inspected: `{MJCF.relative_to(PROJECT_ROOT).as_posix()}`",
        "- No MJCF value was changed.",
        "- MuJoCo `motor.ctrlrange` is reported verbatim; for these motor actuators it is not a position range.",
        "- Range comparison tolerance: `0.1°` at each endpoint.",
        "",
        "| Hardware coordinate | MuJoCo joint | body | axis | hardware field range (deg) | MuJoCo range (rad) | MuJoCo range (deg) | actuator | ctrlrange | range result | sign result | zero result | overall |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for name, evidence in FIELD_EVIDENCE.items():
        item = mjcf[name]
        low_rad, high_rad = (float(value) for value in item["range"].split())
        low_deg, high_deg = math.degrees(low_rad), math.degrees(high_rad)
        range_result = (
            "MATCH"
            if abs(low_deg - evidence.minimum_deg) <= 0.1
            and abs(high_deg - evidence.maximum_deg) <= 0.1
            else "RANGE_MISMATCH"
        )
        sign_result = "INSUFFICIENT_EVIDENCE"
        zero_result = "ZERO_UNKNOWN"
        overall = range_result if range_result == "RANGE_MISMATCH" else sign_result
        lines.append(
            f"| {name} | {item['joint']} | {item['body']} | `{item['axis']}` | "
            f"[{fmt(evidence.minimum_deg)}, {fmt(evidence.maximum_deg)}] | "
            f"`[{item['range'].replace(' ', ', ')}]` | "
            f"[{low_deg:.3f}, {high_deg:.3f}] | {item['actuator']} | "
            f"`{item['ctrlrange']}` | {range_result} | {sign_result} | "
            f"{zero_result} | {overall} |"
        )

    lines.extend(
        [
            "",
            "## Classification notes",
            "",
            "- `MATCH` applies only to numeric range endpoints within tolerance; it does not confirm sign or zero.",
            "- `SIGN_MISMATCH_CANDIDATE` is not assigned from the available static evidence. J2/J7 mirror evidence concerns left versus right hardware control coordinates, not hardware versus MuJoCo.",
            "- Joints with otherwise matching ranges remain `INSUFFICIENT_EVIDENCE` overall because hardware-to-MuJoCo sign and zero are unresolved.",
            "- `ctrlrange` values are actuator control limits and must not be compared numerically with joint angular ranges.",
            "",
            "## CONFIRMED",
            "",
            "- The table's MuJoCo joint, body, axis, range, actuator, and ctrlrange values were parsed directly from the current X2-limit MJCF.",
            "",
            "## FIELD_TEST_EVIDENCE",
            "",
            "- The hardware control-coordinate limits and J2/J7 mirrored endpoint observations are transcribed exactly from the operator's Phase 2B-1 request.",
            "",
            "## INFERRED_CANDIDATE",
            "",
            "- Hardware-to-MuJoCo rows use semantic name correspondence only and remain candidate mappings until correlated evidence exists.",
            "",
            "## UNKNOWN",
            "",
            "- Hardware-to-MuJoCo sign, zero, encoder offset, and exact physical effort origin.",
            "",
            "## NEEDS_PHYSICAL_VERIFICATION",
            "",
            "- A later, separately authorized physical-direction verification is required before changing any mapping sign or MJCF axis.",
            "",
        ]
    )
    (CALIBRATION / "hardware_vs_mujoco_arm_report.md").write_text(
        "\n".join(lines), encoding="utf-8"
    )


def main() -> int:
    mjcf = load_mjcf_arm()
    missing = sorted(set(FIELD_EVIDENCE) - set(mjcf))
    if missing:
        raise RuntimeError(f"Arm joints missing from MJCF: {missing}")
    update_mapping()
    write_arm_evidence_report()
    write_comparison_report(mjcf)
    print("Updated mapping field-evidence columns and generated arm reports.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
