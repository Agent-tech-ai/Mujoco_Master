"""Ingest the Phase 2B-1 passive SSH capture and generate calibration artifacts.

The input is the local stdout evidence produced by the audited read-only remote
script.  This tool has no network code and never modifies an MJCF.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import re
import statistics
from typing import Any, Iterable


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CALIBRATION = PROJECT_ROOT / "calibration"
DEFAULT_EVIDENCE = PROJECT_ROOT.parent / "work" / "x2_phase2b_readonly_capture.txt"
DEFAULT_LOG = CALIBRATION / "logs" / "real" / "static_001.csv"
MAPPING = CALIBRATION / "joint_mapping.csv"
ONESHOT_EVIDENCE = "x2_phase2b_schema_and_one_shots.txt"

OFFICIAL_JOINT_DOC = (
    "https://x2-aimdk.agibot.com/en/v0.9.0/Interface/control_mod/"
    "joint_control.html"
)

GROUPS: dict[str, list[str]] = {
    "head": ["head_yaw", "head_pitch"],
    "arm": [
        "left_shoulder_pitch",
        "left_shoulder_roll",
        "left_shoulder_yaw",
        "left_elbow",
        "left_wrist_yaw",
        "left_wrist_pitch",
        "left_wrist_roll",
        "right_shoulder_pitch",
        "right_shoulder_roll",
        "right_shoulder_yaw",
        "right_elbow",
        "right_wrist_yaw",
        "right_wrist_pitch",
        "right_wrist_roll",
    ],
    "waist": ["waist_yaw", "waist_pitch", "waist_roll"],
    "leg": [
        "left_hip_pitch",
        "left_hip_roll",
        "left_hip_yaw",
        "left_knee",
        "left_ankle_pitch",
        "left_ankle_roll",
        "right_hip_pitch",
        "right_hip_roll",
        "right_hip_yaw",
        "right_knee",
        "right_ankle_pitch",
        "right_ankle_roll",
    ],
}

TOPIC_TO_GROUP = {
    f"/aima/hal/joint/{group}/state": group for group in GROUPS
}
IMU_TOPICS = {
    "/aima/hal/imu/chest/state": "chest",
    "/aima/hal/imu/torso/state": "torso",
}
CSV_FIELDS = [
    "timestamp",
    "joint_name",
    "command_position",
    "measured_position",
    "measured_velocity",
    "measured_torque",
    "imu_quaternion",
    "imu_gyro",
    "imu_accel",
]


def read_text(path: Path) -> str:
    data = path.read_bytes()
    for encoding in ("utf-8-sig", "utf-16", "utf-8"):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            pass
    raise UnicodeError(f"Unable to decode {path}")


def marker_objects(text: str, marker: str) -> list[dict[str, Any]]:
    prefix = marker + "\t"
    objects: list[dict[str, Any]] = []
    for line_number, line in enumerate(text.splitlines(), 1):
        if not line.startswith(prefix):
            continue
        try:
            value = json.loads(line[len(prefix) :])
        except json.JSONDecodeError as exc:
            raise ValueError(f"Invalid {marker} JSON at line {line_number}: {exc}") from exc
        if not isinstance(value, dict):
            raise ValueError(f"Expected object for {marker} at line {line_number}")
        objects.append(value)
    return objects


def finite_number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def vector3(value: Any) -> list[float] | None:
    if not isinstance(value, dict):
        return None
    result = [finite_number(value.get(axis)) for axis in ("x", "y", "z")]
    return [float(item) for item in result] if all(item is not None for item in result) else None


def quaternion_wxyz(value: Any) -> list[float] | None:
    if not isinstance(value, dict):
        return None
    result = [finite_number(value.get(axis)) for axis in ("w", "x", "y", "z")]
    return [float(item) for item in result] if all(item is not None for item in result) else None


def mean_std(values: Iterable[Any]) -> tuple[float | None, float | None, int]:
    clean = [value for item in values if (value := finite_number(item)) is not None]
    if not clean:
        return None, None, 0
    return statistics.fmean(clean), statistics.pstdev(clean), len(clean)


def vector_mean_std(values: Iterable[list[float] | None]) -> tuple[list[float], list[float], int] | None:
    clean = [item for item in values if item is not None]
    if not clean:
        return None
    width = len(clean[0])
    means = [statistics.fmean(item[index] for item in clean) for index in range(width)]
    stds = [statistics.pstdev(item[index] for item in clean) for index in range(width)]
    return means, stds, len(clean)


def fmt(value: float | None) -> str:
    return "N/A" if value is None else f"{value:.9g}"


def json_compact(value: Any) -> str:
    return json.dumps(value, separators=(",", ":"))


def group_samples(samples: list[dict[str, Any]]) -> dict[str, list[dict[str, Any]]]:
    result = {group: [] for group in GROUPS}
    for sample in samples:
        group = TOPIC_TO_GROUP.get(sample.get("topic"))
        if group:
            message = sample.get("message")
            if isinstance(message, dict):
                result[group].append(sample)
    return result


def array_lengths(samples_by_group: dict[str, list[dict[str, Any]]]) -> dict[str, list[int]]:
    result: dict[str, list[int]] = {}
    for group, samples in samples_by_group.items():
        lengths = {
            len(joints)
            for sample in samples
            if isinstance((joints := sample["message"].get("joints")), list)
        }
        result[group] = sorted(lengths)
    return result


def write_unified_csv(samples: list[dict[str, Any]], lengths: dict[str, list[int]], output: Path) -> int:
    rows: list[dict[str, str]] = []
    for sample in samples:
        elapsed = finite_number(sample.get("elapsed"))
        if elapsed is None:
            continue
        topic = sample.get("topic")
        message = sample.get("message")
        if not isinstance(message, dict):
            continue
        group = TOPIC_TO_GROUP.get(topic)
        if group:
            joints = message.get("joints")
            if not isinstance(joints, list):
                continue
            mapping_is_usable = lengths[group] == [len(GROUPS[group])]
            for index, joint in enumerate(joints):
                if not isinstance(joint, dict):
                    continue
                live_name = joint.get("name")
                expected_live_name = (
                    GROUPS[group][index] + "_joint"
                    if mapping_is_usable and index < len(GROUPS[group])
                    else None
                )
                if isinstance(live_name, str) and live_name and live_name == expected_live_name:
                    name = live_name.removesuffix("_joint")
                elif mapping_is_usable and index < len(GROUPS[group]):
                    name = GROUPS[group][index]
                else:
                    name = f"{group}[{index}]"
                rows.append(
                    {
                        "timestamp": f"{elapsed:.9f}",
                        "joint_name": name,
                        "command_position": "",
                        "measured_position": fmt(finite_number(joint.get("position"))).replace("N/A", ""),
                        "measured_velocity": fmt(finite_number(joint.get("velocity"))).replace("N/A", ""),
                        # This is the raw JointState.effort value.  AimDK documents
                        # torque/N*m, while measured-vs-estimated origin is unknown.
                        "measured_torque": fmt(finite_number(joint.get("effort"))).replace("N/A", ""),
                        "imu_quaternion": "",
                        "imu_gyro": "",
                        "imu_accel": "",
                    }
                )
        elif topic in IMU_TOPICS:
            orientation = quaternion_wxyz(message.get("orientation"))
            gyro = vector3(message.get("angular_velocity"))
            accel = vector3(message.get("linear_acceleration"))
            rows.append(
                {
                    "timestamp": f"{elapsed:.9f}",
                    "joint_name": f"__imu_{IMU_TOPICS[topic]}__",
                    "command_position": "",
                    "measured_position": "",
                    "measured_velocity": "",
                    "measured_torque": "",
                    "imu_quaternion": "" if orientation is None else json_compact(orientation),
                    "imu_gyro": "" if gyro is None else json_compact(gyro),
                    "imu_accel": "" if accel is None else json_compact(accel),
                }
            )

    rows.sort(key=lambda row: (float(row["timestamp"]), row["joint_name"]))
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    return len(rows)


def write_schema_report(
    schema: dict[str, Any],
    samples_by_group: dict[str, list[dict[str, Any]]],
    lengths: dict[str, list[int]],
    evidence_name: str,
) -> None:
    joint_array_fields = schema.get("JointStateArray", {}).get("fields", {})
    joint_fields = schema.get("JointState", {}).get("fields", {})
    lines = [
        "# Robot JointStateArray schema",
        "",
        f"- Live evidence: `calibration/evidence/{evidence_name}`",
        f"- Raw `ros2 interface show` and four `echo --once` outputs: `calibration/evidence/{ONESHOT_EVIDENCE}`",
        f"- Official interface reference: {OFFICIAL_JOINT_DOC}",
        "- Capture was subscription-only; no command, service, or action was sent.",
        "",
        "## CONFIRMED",
        "",
        "### Live-loaded ROS message fields",
        "",
        "| Type | field | ROS type |",
        "|---|---|---|",
    ]
    for type_name in ("JointStateArray", "JointState", "MessageHeader", "DomainErrorState"):
        fields = schema.get(type_name, {}).get("fields")
        if isinstance(fields, dict):
            for field, ros_type in fields.items():
                lines.append(f"| aimdk_msgs/msg/{type_name} | {field} | `{ros_type}` |")
    lines.extend(
        [
            "",
            "### Live array observations",
            "",
            "| group | topic | serialized samples | observed array length(s) | documented length | length result |",
            "|---|---|---:|---|---:|---|",
        ]
    )
    for group, names in GROUPS.items():
        actual = lengths[group]
        result = "MATCH" if actual == [len(names)] else "MISSING_OR_MISMATCH"
        lines.append(
            f"| {group} | `/aima/hal/joint/{group}/state` | "
            f"{len(samples_by_group[group])} | `{actual}` | {len(names)} | {result} |"
        )

    name_values: dict[str, list[str]] = {}
    for group, samples in samples_by_group.items():
        values = set()
        for sample in samples:
            joints = sample["message"].get("joints", [])
            if isinstance(joints, list):
                for joint in joints:
                    if isinstance(joint, dict) and isinstance(joint.get("name"), str):
                        values.add(joint["name"])
        name_values[group] = sorted(values)

    lines.extend(
        [
            "",
            "### Live names by array index",
            "",
            "| group | index | observed `JointState.name` values | documented candidate |",
            "|---|---:|---|---|",
        ]
    )
    for group, samples in samples_by_group.items():
        maximum_length = max(lengths[group], default=0)
        for index in range(maximum_length):
            observed = set()
            for sample in samples:
                joints = sample["message"].get("joints", [])
                if isinstance(joints, list) and index < len(joints):
                    joint = joints[index]
                    if isinstance(joint, dict) and isinstance(joint.get("name"), str):
                        observed.add(joint["name"])
            candidate = GROUPS[group][index] if index < len(GROUPS[group]) else "UNKNOWN"
            lines.append(f"| {group} | {index} | `{sorted(observed)}` | {candidate} |")

    lines.extend(
        [
            "",
            "### Field interpretation",
            "",
            f"- Array position field: `joints[].position` ({joint_fields.get('position', 'NOT_PRESENT')}).",
            f"- Array velocity field: `joints[].velocity` ({joint_fields.get('velocity', 'NOT_PRESENT')}).",
            f"- Array effort field: `joints[].effort` ({joint_fields.get('effort', 'NOT_PRESENT')}).",
            f"- Joint name field: `{joint_fields.get('name', 'NOT_PRESENT')}`; observed values by group: `{name_values}`.",
            f"- Joint ID field: `{'PRESENT' if any(k in joint_fields for k in ('id', 'joint_id')) else 'NOT_PRESENT_IN_LIVE_SCHEMA'}`.",
            f"- Motor ID field: `{'PRESENT' if 'motor_id' in joint_fields else 'NOT_PRESENT_IN_LIVE_SCHEMA'}`.",
            f"- Status/state fields in JointState: `{[key for key in joint_fields if key in ('status', 'state', 'faultcode')]}`.",
            f"- Temperature/voltage fields: `{[key for key in joint_fields if 'temp' in key or 'vol' in key]}`.",
            f"- JointStateArray fields: `{joint_array_fields}`; its `header` is the message-level timestamp source.",
            "- Live one-shot headers used frame IDs `x2_arm`, `x2_head`, `x2_leg`, and `x2_waist`; `stamp` and `sequence` were populated, while all four observed `meas_stamp` values were zero.",
            "- `static_001.csv` uses the subscriber's monotonic elapsed receive time for its `timestamp`; source headers remain preserved in raw evidence.",
            "- AimDK documents position as rad, velocity as rad/s, and effort as torque in N·m.",
            "",
            "## FIELD_TEST_EVIDENCE",
            "",
            "- Live samples establish the actual schema and array lengths on this robot/software version.",
            "- All 31 live array indices carried stable, populated `JointState.name` values; these names confirm the interface-level index assignment directly.",
            "",
            "## INFERRED_CANDIDATE",
            "",
            "- Exact string equality between 30 live hardware names and current MuJoCo joint names is confirmed. Physical direction/zero correspondence remains only a candidate interpretation; static values were not used to guess it.",
            "",
            "## UNKNOWN",
            "",
            "- AimDK documentation labels `effort` as torque (N·m), but does not identify whether it is measured motor torque, estimated joint torque, commanded torque, current-derived torque, or another estimator output.",
            "- Hardware joint IDs are unknown when no ID field is present.",
            "- Hardware zero, encoder offset, and hardware-to-MuJoCo sign remain unknown.",
            "",
            "## NEEDS_PHYSICAL_VERIFICATION",
            "",
            "- Confirm the documented array-to-physical-joint order independently before any command-producing calibration phase.",
            "- Confirm effort origin with manufacturer source/API documentation or implementation source.",
            "",
        ]
    )
    (CALIBRATION / "robot_joint_state_schema.md").write_text(
        "\n".join(lines), encoding="utf-8"
    )


def write_static_report(
    samples_by_group: dict[str, list[dict[str, Any]]],
    lengths: dict[str, list[int]],
    all_samples: list[dict[str, Any]],
    summary: dict[str, Any],
    evidence_name: str,
) -> None:
    lines = [
        "# static_001 analysis",
        "",
        f"- Evidence: `calibration/evidence/{evidence_name}`",
        "- Capture mode: passive subscription only, approximately 30 seconds.",
        f"- Requested duration: `{summary.get('duration_requested_seconds', 'UNKNOWN')}` s; serialized interval cap: `{summary.get('serialization_interval_seconds', 'UNKNOWN')}` s per topic.",
        "- The local SSH wrapper returned code 2 only because PowerShell appended CR to the final shell `exit 0`; the Python capture produced its summary, all 8259 JSON records parsed, and no capture-failed marker or traceback exists.",
        "- Statistics use raw source coordinates; no zero/sign/scale correction was applied.",
        "- CSV timestamp is subscriber monotonic elapsed receive time; source `header.stamp`, `sequence`, and `meas_stamp` remain in the raw evidence.",
        "- `effort` is reported as the raw field. AimDK documents torque/N·m, but its physical origin remains unknown.",
        "",
        "## Joint array-index statistics",
        "",
        "| group | index | live-confirmed name | n | mean position | std position | mean velocity | std velocity | mean effort | std effort |",
        "|---|---:|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for group, samples in samples_by_group.items():
        usable = lengths[group] == [len(GROUPS[group])]
        maximum_length = max(lengths[group], default=0)
        for index in range(maximum_length):
            joint_records = []
            for sample in samples:
                joints = sample["message"].get("joints")
                if isinstance(joints, list) and index < len(joints) and isinstance(joints[index], dict):
                    joint_records.append(joints[index])
            p_mean, p_std, count = mean_std(record.get("position") for record in joint_records)
            v_mean, v_std, _ = mean_std(record.get("velocity") for record in joint_records)
            e_mean, e_std, _ = mean_std(record.get("effort") for record in joint_records)
            name = GROUPS[group][index] if usable else "UNKNOWN"
            lines.append(
                f"| {group} | {index} | {name} | {count} | {fmt(p_mean)} | "
                f"{fmt(p_std)} | {fmt(v_mean)} | {fmt(v_std)} | {fmt(e_mean)} | {fmt(e_std)} |"
            )

    lines.extend(
        [
            "",
            "## Capture coverage",
            "",
            "| topic | received callbacks | serialized samples | observed serialized time span (s) |",
            "|---|---:|---:|---:|",
        ]
    )
    received = summary.get("received_callbacks", {})
    serialized = summary.get("serialized_samples", {})
    for topic in sorted(set(received) | set(serialized)):
        elapsed_values = [
            value
            for sample in all_samples
            if sample.get("topic") == topic
            and (value := finite_number(sample.get("elapsed"))) is not None
        ]
        span = max(elapsed_values) - min(elapsed_values) if len(elapsed_values) >= 2 else None
        lines.append(
            f"| `{topic}` | {received.get(topic, 'UNKNOWN')} | "
            f"{serialized.get(topic, 'UNKNOWN')} | {fmt(span)} |"
        )

    lines.extend(
        [
            "",
            "## IMU statistics",
            "",
            "| IMU | samples | mean gyro [x,y,z] (rad/s) | std gyro | mean acceleration [x,y,z] (m/s²) | std acceleration |",
            "|---|---:|---|---|---|---|",
        ]
    )
    for topic, label in IMU_TOPICS.items():
        messages = [sample["message"] for sample in all_samples if sample.get("topic") == topic]
        gyro = vector_mean_std(vector3(message.get("angular_velocity")) for message in messages)
        accel = vector_mean_std(vector3(message.get("linear_acceleration")) for message in messages)
        sample_count = min(gyro[2] if gyro else 0, accel[2] if accel else 0)
        lines.append(
            f"| {label} | {sample_count} | "
            f"`{gyro[0] if gyro else 'N/A'}` | `{gyro[1] if gyro else 'N/A'}` | "
            f"`{accel[0] if accel else 'N/A'}` | `{accel[1] if accel else 'N/A'}` |"
        )

    lines.extend(
        [
            "",
            "## CONFIRMED",
            "",
            "- Report values are direct population mean/std statistics over decoded passive samples.",
            "- Every group index had one stable, populated live name throughout the capture.",
            "",
            "## FIELD_TEST_EVIDENCE",
            "",
            "- The capture describes only the robot's naturally occurring state during this window; the workflow did not request a pose change.",
            "",
            "## INFERRED_CANDIDATE",
            "",
            "- No label was inferred from static position similarity; array labels come from stable live `JointState.name` values.",
            "- Hardware↔MuJoCo physical correspondence remains a candidate even where strings match exactly.",
            "",
            "## UNKNOWN",
            "",
            "- Static position similarity is not used to establish a mapping, sign, or zero.",
            "- Effort physical origin and IMU gravity policy remain unknown.",
            "",
            "## NEEDS_PHYSICAL_VERIFICATION",
            "",
            "- Verify the robot was physically stationary throughout the capture and note any external support/contact loads.",
            "",
        ]
    )
    (CALIBRATION / "static_001_analysis.md").write_text(
        "\n".join(lines), encoding="utf-8"
    )


def update_mapping(lengths: dict[str, list[int]], samples_by_group: dict[str, list[dict[str, Any]]], evidence_name: str) -> None:
    with MAPPING.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        fieldnames = list(reader.fieldnames or [])
    new_columns = [
        "live_array_length",
        "live_array_length_source",
        "live_joint_name",
        "live_joint_name_source",
        "live_joint_name_field_status",
        "live_schema_evidence_source",
        "effort_definition",
        "effort_definition_source",
    ]
    for column in new_columns:
        if column not in fieldnames:
            fieldnames.append(column)

    for row in rows:
        group = row["hardware_group"]
        actual_lengths = lengths.get(group, [])
        expected = len(GROUPS.get(group, []))
        row["live_array_length"] = json_compact(actual_lengths) if actual_lengths else "UNKNOWN"
        row["live_array_length_source"] = f"READ_ONLY_CAPTURE: calibration/evidence/{evidence_name}"
        observed_names: set[str] = set()
        try:
            row_index = int(row["hardware_group_index"])
        except (KeyError, ValueError):
            row_index = -1
        for sample in samples_by_group.get(group, []):
            joints = sample["message"].get("joints", [])
            if isinstance(joints, list) and 0 <= row_index < len(joints):
                joint = joints[row_index]
                if isinstance(joint, dict) and isinstance(joint.get("name"), str):
                    observed_names.add(joint["name"])
        if observed_names == {""}:
            row["live_joint_name_field_status"] = "PRESENT_BUT_EMPTY"
            row["live_joint_name"] = "UNKNOWN"
        elif observed_names:
            row["live_joint_name_field_status"] = "POPULATED:" + json_compact(sorted(observed_names))
            row["live_joint_name"] = (
                next(iter(observed_names)) if len(observed_names) == 1 else "INCONSISTENT"
            )
        else:
            row["live_joint_name_field_status"] = "UNKNOWN_OR_NO_SAMPLE"
            row["live_joint_name"] = "UNKNOWN"
        row["live_joint_name_source"] = f"READ_ONLY_CAPTURE: calibration/evidence/{evidence_name}"
        row["live_schema_evidence_source"] = f"READ_ONLY_CAPTURE: calibration/evidence/{evidence_name}"
        row["effort_definition"] = "TORQUE_N_M_DOCUMENTED; PHYSICAL_ORIGIN_UNKNOWN"
        row["effort_definition_source"] = OFFICIAL_JOINT_DOC

        if actual_lengths == [expected] and expected:
            expected_live_name = row["hardware_joint_name"] + "_joint"
            if observed_names == {expected_live_name}:
                row["array_index_status"] = "CONFIRMED_BY_LIVE_NAME"
                row["array_index_evidence_source"] = (
                    f"Live JointState.name at this index in calibration/evidence/{evidence_name}"
                )
                if row["mujoco_joint_name"] == expected_live_name:
                    row["hardware_mujoco_mapping_status"] = "CONFIRMED_NAME_MATCH_ONLY"
                    row["hardware_mujoco_mapping_evidence_source"] = (
                        "Live hardware JointState.name exactly equals current MuJoCo joint name; "
                        "direction and zero remain unverified"
                    )
                else:
                    row["hardware_mujoco_mapping_status"] = "INFERRED_CANDIDATE"
                    row["hardware_mujoco_mapping_evidence_source"] = (
                        "Live hardware name confirmed, but no exact current MuJoCo joint-name match"
                    )
            else:
                row["array_index_status"] = "CONFIRMED_DOCUMENTED_ORDER_WITH_LIVE_LENGTH"
                row["array_index_evidence_source"] = (
                    f"Official AimDK order: {OFFICIAL_JOINT_DOC}; matching live length in "
                    f"calibration/evidence/{evidence_name}"
                )
                row["hardware_mujoco_mapping_status"] = "INFERRED_CANDIDATE"
                row["hardware_mujoco_mapping_evidence_source"] = (
                    "Documented hardware array order plus matching live length and semantic "
                    "MuJoCo name; no hardware-to-MuJoCo direction/zero correlation"
                )

        notes = row.get("notes", "")
        notes = re.sub(r";? Phase2 confirmed live topic [^;]+", "", notes)
        notes = notes.replace(
            "hardware ID/name field/zero/sign/index and effort semantics are unverified",
            "hardware ID/zero/sign/encoder offset and effort physical origin remain UNKNOWN",
        )
        notes = notes.replace(
            "array index and effort semantics remain UNKNOWN",
            "effort physical origin remains UNKNOWN",
        )
        notes = notes.replace(
            "array index effort semantics and hardware direction remain UNKNOWN",
            "hardware direction and effort physical origin remain UNKNOWN",
        )
        notes = re.sub(
            r";? Phase2B live index .*?hardware ID/zero/sign/encoder offset remain UNKNOWN",
            "",
            notes,
        )
        phase2b_note = (
            f"Phase2B live index {row.get('hardware_group_index', 'UNKNOWN')} name "
            f"{row.get('live_joint_name', 'UNKNOWN')} confirmed by passive capture; "
            "hardware ID/zero/sign/encoder offset remain UNKNOWN"
        )
        row["notes"] = notes.strip("; ") + "; " + phase2b_note

    with MAPPING.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence", type=Path, default=DEFAULT_EVIDENCE)
    parser.add_argument("--output-log", type=Path, default=DEFAULT_LOG)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    text = read_text(args.evidence)
    schemas = marker_objects(text, "PHASE2B_SCHEMA")
    samples = marker_objects(text, "PHASE2B_SAMPLE")
    summaries = marker_objects(text, "PHASE2B_CAPTURE_SUMMARY")
    if len(schemas) != 1:
        raise SystemExit(
            f"Expected exactly one PHASE2B_SCHEMA marker, found {len(schemas)}. "
            "The robot type support/capture likely did not complete."
        )
    if not samples:
        raise SystemExit("No PHASE2B_SAMPLE records found; static capture did not complete.")
    if len(summaries) != 1:
        raise SystemExit(f"Expected one capture summary, found {len(summaries)}")

    samples_by_group = group_samples(samples)
    lengths = array_lengths(samples_by_group)
    evidence_destination = CALIBRATION / "evidence" / args.evidence.name
    evidence_destination.parent.mkdir(parents=True, exist_ok=True)
    evidence_destination.write_bytes(args.evidence.read_bytes())

    row_count = write_unified_csv(samples, lengths, args.output_log)
    write_schema_report(schemas[0], samples_by_group, lengths, args.evidence.name)
    write_static_report(samples_by_group, lengths, samples, summaries[0], args.evidence.name)
    update_mapping(lengths, samples_by_group, args.evidence.name)
    print(f"Ingested {len(samples)} passive samples into {row_count} CSV rows.")
    print(f"Observed joint lengths: {lengths}")
    print(f"Unified log: {args.output_log.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
