"""I/O and timestamp alignment for the shared real/simulation CSV schema."""

from __future__ import annotations

import csv
from dataclasses import dataclass
import json
from pathlib import Path
from typing import Iterable

import numpy as np


REQUIRED_COLUMNS = (
    "timestamp",
    "joint_name",
    "command_position",
    "measured_position",
    "measured_velocity",
    "measured_torque",
    "imu_quaternion",
    "imu_gyro",
    "imu_accel",
)
SCALAR_FIELDS = (
    "command_position",
    "measured_position",
    "measured_velocity",
    "measured_torque",
)
VECTOR_FIELDS = {"imu_quaternion": 4, "imu_gyro": 3, "imu_accel": 3}


class LogFormatError(ValueError):
    """Raised when a calibration log does not follow the shared schema."""


@dataclass(frozen=True)
class LogRow:
    timestamp: float
    joint_name: str
    command_position: float
    measured_position: float
    measured_velocity: float
    measured_torque: float
    imu_quaternion: np.ndarray
    imu_gyro: np.ndarray
    imu_accel: np.ndarray


@dataclass(frozen=True)
class JointSeries:
    timestamp: np.ndarray
    command_position: np.ndarray
    measured_position: np.ndarray
    measured_velocity: np.ndarray
    measured_torque: np.ndarray


def _parse_optional_float(value: str, *, field: str, line_number: int) -> float:
    value = value.strip()
    if not value:
        return float("nan")
    try:
        return float(value)
    except ValueError as exc:
        raise LogFormatError(
            f"line {line_number}: {field} must be a number or blank, got {value!r}"
        ) from exc


def _parse_vector(value: str, *, field: str, line_number: int) -> np.ndarray:
    size = VECTOR_FIELDS[field]
    value = value.strip()
    if not value:
        return np.full(size, np.nan, dtype=float)
    try:
        parsed = json.loads(value)
        array = np.asarray(parsed, dtype=float)
    except (json.JSONDecodeError, TypeError, ValueError) as exc:
        raise LogFormatError(
            f"line {line_number}: {field} must be a JSON array of {size} numbers"
        ) from exc
    if array.shape != (size,):
        raise LogFormatError(
            f"line {line_number}: {field} must have {size} elements, got {array.shape}"
        )
    return array


def load_log(path: Path) -> list[LogRow]:
    path = path.resolve()
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise LogFormatError(f"{path}: missing CSV header")
        missing = [name for name in REQUIRED_COLUMNS if name not in reader.fieldnames]
        if missing:
            raise LogFormatError(f"{path}: missing required columns: {missing}")
        rows: list[LogRow] = []
        for line_number, raw in enumerate(reader, start=2):
            timestamp = _parse_optional_float(
                raw["timestamp"], field="timestamp", line_number=line_number
            )
            if not np.isfinite(timestamp):
                raise LogFormatError(f"line {line_number}: timestamp may not be blank")
            joint_name = raw["joint_name"].strip()
            if not joint_name:
                raise LogFormatError(f"line {line_number}: joint_name may not be blank")
            scalar = {
                field: _parse_optional_float(
                    raw[field], field=field, line_number=line_number
                )
                for field in SCALAR_FIELDS
            }
            rows.append(
                LogRow(
                    timestamp=timestamp,
                    joint_name=joint_name,
                    **scalar,
                    **{
                        field: _parse_vector(
                            raw[field], field=field, line_number=line_number
                        )
                        for field in VECTOR_FIELDS
                    },
                )
            )
    if not rows:
        raise LogFormatError(f"{path}: log has no data rows")
    return rows


def load_mapping(path: Path) -> tuple[dict[str, str], dict[str, tuple[float, float]]]:
    """Return hardware-name aliases and MuJoCo limits from joint_mapping.csv."""

    aliases: dict[str, str] = {}
    limits: dict[str, tuple[float, float]] = {}
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        needed = {"hardware_joint_name", "mujoco_joint_name", "mujoco_min", "mujoco_max"}
        if reader.fieldnames is None or not needed.issubset(reader.fieldnames):
            raise LogFormatError(f"{path}: invalid mapping header")
        for raw in reader:
            hardware = raw["hardware_joint_name"].strip()
            mujoco_name = raw["mujoco_joint_name"].strip()
            if hardware and mujoco_name and not mujoco_name.startswith("NOT_PRESENT"):
                aliases[hardware] = mujoco_name
                aliases[mujoco_name] = mujoco_name
                try:
                    limits[mujoco_name] = (
                        float(raw["mujoco_min"]),
                        float(raw["mujoco_max"]),
                    )
                except ValueError:
                    pass
    return aliases, limits


def canonicalize_rows(rows: Iterable[LogRow], aliases: dict[str, str]) -> list[LogRow]:
    result: list[LogRow] = []
    for row in rows:
        canonical = aliases.get(row.joint_name, row.joint_name)
        result.append(
            LogRow(
                timestamp=row.timestamp,
                joint_name=canonical,
                command_position=row.command_position,
                measured_position=row.measured_position,
                measured_velocity=row.measured_velocity,
                measured_torque=row.measured_torque,
                imu_quaternion=row.imu_quaternion,
                imu_gyro=row.imu_gyro,
                imu_accel=row.imu_accel,
            )
        )
    return result


def joint_names(rows: Iterable[LogRow]) -> list[str]:
    return sorted({row.joint_name for row in rows})


def joint_series(rows: Iterable[LogRow], name: str) -> JointSeries:
    selected = sorted(
        (row for row in rows if row.joint_name == name), key=lambda row: row.timestamp
    )
    if not selected:
        raise KeyError(name)
    timestamps = np.asarray([row.timestamp for row in selected], dtype=float)
    unique_mask = np.concatenate(([True], np.diff(timestamps) > 0))
    selected = [row for row, keep in zip(selected, unique_mask) if keep]
    return JointSeries(
        timestamp=np.asarray([row.timestamp for row in selected]),
        **{
            field: np.asarray([getattr(row, field) for row in selected], dtype=float)
            for field in SCALAR_FIELDS
        },
    )


def imu_series(rows: Iterable[LogRow]) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    """Deduplicate IMU samples repeated once per joint at a timestamp."""

    samples: dict[float, dict[str, np.ndarray]] = {}
    for row in rows:
        current = samples.setdefault(row.timestamp, {})
        for field in VECTOR_FIELDS:
            value = getattr(row, field)
            if field not in current and np.any(np.isfinite(value)):
                current[field] = value
    timestamps = sorted(
        timestamp
        for timestamp, sample in samples.items()
        if any(field in sample for field in VECTOR_FIELDS)
    )
    data = {
        field: np.asarray(
            [
                samples[timestamp].get(field, np.full(size, np.nan))
                for timestamp in timestamps
            ],
            dtype=float,
        )
        for field, size in VECTOR_FIELDS.items()
    }
    return np.asarray(timestamps, dtype=float), data


def relative_time(rows: Iterable[LogRow]) -> list[LogRow]:
    rows = list(rows)
    start = min(row.timestamp for row in rows)
    return [
        LogRow(
            timestamp=row.timestamp - start,
            joint_name=row.joint_name,
            command_position=row.command_position,
            measured_position=row.measured_position,
            measured_velocity=row.measured_velocity,
            measured_torque=row.measured_torque,
            imu_quaternion=row.imu_quaternion,
            imu_gyro=row.imu_gyro,
            imu_accel=row.imu_accel,
        )
        for row in rows
    ]


def _median_step(timestamps: np.ndarray) -> float:
    differences = np.diff(timestamps)
    differences = differences[differences > 0]
    return float(np.median(differences)) if differences.size else 0.01


def common_timeline(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    start = max(float(first[0]), float(second[0]))
    stop = min(float(first[-1]), float(second[-1]))
    if stop <= start:
        raise LogFormatError(
            f"logs have no overlapping timestamps ({first[0]}..{first[-1]} vs "
            f"{second[0]}..{second[-1]})"
        )
    step = max(_median_step(first), _median_step(second))
    count = max(2, int(np.floor((stop - start) / step)) + 1)
    return np.linspace(start, stop, count)


def interpolate(timestamps: np.ndarray, values: np.ndarray, timeline: np.ndarray) -> np.ndarray:
    if values.ndim == 1:
        finite = np.isfinite(values) & np.isfinite(timestamps)
        if np.count_nonzero(finite) < 2:
            return np.full(timeline.shape, np.nan)
        return np.interp(timeline, timestamps[finite], values[finite])
    columns = [interpolate(timestamps, values[:, index], timeline) for index in range(values.shape[1])]
    return np.column_stack(columns)


def safe_filename(name: str) -> str:
    return "".join(character if character.isalnum() or character in "-_" else "_" for character in name)

