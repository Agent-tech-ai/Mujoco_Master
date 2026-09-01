from __future__ import annotations

import csv
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np


HERE = Path(__file__).resolve().parent
MASTER_ROOT = HERE.parents[1]
ASSET_DIR = MASTER_ROOT / "assets" / "Master"
CURRENT_XML = ASSET_DIR / "ff_master_ultra.xml"


def numbers(text: str | None, count: int) -> np.ndarray:
    if not text:
        return np.zeros(count, dtype=float)
    values = np.asarray([float(value) for value in text.split()], dtype=float)
    if values.size != count:
        raise ValueError(f"expected {count} values, got {values.size}: {text}")
    return values


def fmt(value: float | None) -> str:
    return "" if value is None else f"{float(value):.12g}"


def fmt_vec(values: np.ndarray | list[float] | tuple[float, ...]) -> str:
    return " ".join(fmt(float(value)) for value in values)


def quat_to_rotation(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = [float(value) for value in quat]
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def rpy_to_rotation(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = [float(value) for value in rpy]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.asarray([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.asarray([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.asarray([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def region(name: str) -> str:
    if "hip_" in name or "knee_link" in name or "ankle_" in name:
        return "lower_limb"
    if any(token in name for token in ("shoulder", "elbow", "wrist")):
        return "arm"
    if name in {"pelvis", "waist_yaw_link", "waist_pitch_link", "torso_link"}:
        return "central"
    if name.startswith("head_"):
        return "head"
    return "other"


def side(name: str) -> str:
    if name.startswith("left_"):
        return "left"
    if name.startswith("right_"):
        return "right"
    return "center"


def full_components(matrix: np.ndarray) -> dict[str, str]:
    return {
        "ixx": fmt(matrix[0, 0]),
        "iyy": fmt(matrix[1, 1]),
        "izz": fmt(matrix[2, 2]),
        "ixy": fmt(matrix[0, 1]),
        "ixz": fmt(matrix[0, 2]),
        "iyz": fmt(matrix[1, 2]),
    }


def load_raw_mjcf_inertials(path: Path) -> dict[str, dict[str, np.ndarray | float | str]]:
    root = ET.parse(path).getroot()
    result: dict[str, dict[str, np.ndarray | float | str]] = {}
    for body in root.findall(".//body"):
        name = body.attrib.get("name", "")
        inertial = body.find("inertial")
        if not name or inertial is None:
            continue
        mass = float(inertial.attrib["mass"])
        pos = numbers(inertial.attrib.get("pos"), 3)
        quat = numbers(inertial.attrib.get("quat", "1 0 0 0"), 4)
        if "diaginertia" in inertial.attrib:
            diag = numbers(inertial.attrib["diaginertia"], 3)
            rotation = quat_to_rotation(quat)
            full = rotation @ np.diag(diag) @ rotation.T
            representation = "diaginertia+quat"
        elif "fullinertia" in inertial.attrib:
            values = numbers(inertial.attrib["fullinertia"], 6)
            full = np.asarray(
                [
                    [values[0], values[3], values[4]],
                    [values[3], values[1], values[5]],
                    [values[4], values[5], values[2]],
                ],
                dtype=float,
            )
            diag = np.linalg.eigvalsh(full)
            representation = "fullinertia"
        else:
            raise ValueError(f"unsupported inertial representation in {path}: {name}")
        result[name] = {
            "mass": mass,
            "pos": pos,
            "quat": quat,
            "diag": diag,
            "full": full,
            "representation": representation,
        }
    return result


def load_compiled_mjcf(path: Path) -> dict[str, dict[str, np.ndarray | float | str]]:
    model = mujoco.MjModel.from_xml_path(str(path))
    result: dict[str, dict[str, np.ndarray | float | str]] = {}
    for body_id in range(1, model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        quat = np.asarray(model.body_iquat[body_id], dtype=float)
        diag = np.asarray(model.body_inertia[body_id], dtype=float)
        rotation = quat_to_rotation(quat)
        full = rotation @ np.diag(diag) @ rotation.T
        result[name] = {
            "mass": float(model.body_mass[body_id]),
            "pos": np.asarray(model.body_ipos[body_id], dtype=float),
            "quat": quat,
            "diag": diag,
            "full": full,
            "representation": "compiled_principal_inertia",
        }
    return result


def load_urdf(path: Path) -> dict[str, dict[str, np.ndarray | float | str]]:
    root = ET.parse(path).getroot()
    result: dict[str, dict[str, np.ndarray | float | str]] = {}
    for link in root.findall("link"):
        name = link.attrib.get("name", "")
        inertial = link.find("inertial")
        if not name or inertial is None:
            continue
        origin = inertial.find("origin")
        mass_element = inertial.find("mass")
        inertia = inertial.find("inertia")
        if mass_element is None or inertia is None:
            continue
        pos = numbers(origin.attrib.get("xyz") if origin is not None else None, 3)
        rpy = numbers(origin.attrib.get("rpy") if origin is not None else None, 3)
        tensor = np.asarray(
            [
                [float(inertia.attrib["ixx"]), float(inertia.attrib.get("ixy", 0)), float(inertia.attrib.get("ixz", 0))],
                [float(inertia.attrib.get("ixy", 0)), float(inertia.attrib["iyy"]), float(inertia.attrib.get("iyz", 0))],
                [float(inertia.attrib.get("ixz", 0)), float(inertia.attrib.get("iyz", 0)), float(inertia.attrib["izz"])],
            ],
            dtype=float,
        )
        rotation = rpy_to_rotation(rpy)
        full = rotation @ tensor @ rotation.T
        result[name] = {
            "mass": float(mass_element.attrib["value"]),
            "pos": pos,
            "quat": np.asarray([1.0, 0.0, 0.0, 0.0]),
            "diag": np.linalg.eigvalsh(full),
            "full": full,
            "representation": "urdf_full_tensor",
        }
    return result


def aggregate(records: dict[str, dict[str, np.ndarray | float | str]]) -> dict[str, float]:
    total = sum(float(row["mass"]) for row in records.values())
    lower = sum(float(row["mass"]) for name, row in records.items() if region(name) == "lower_limb")
    upper = total - lower
    left = sum(float(row["mass"]) for name, row in records.items() if side(name) == "left")
    right = sum(float(row["mass"]) for name, row in records.items() if side(name) == "right")
    return {
        "total_mass_kg": total,
        "lower_limb_mass_kg": lower,
        "upper_body_mass_kg": upper,
        "lower_upper_mass_ratio": lower / upper,
        "left_mass_kg": left,
        "right_mass_kg": right,
        "left_right_difference_kg": left - right,
    }


def write_provenance() -> None:
    compiled = load_compiled_mjcf(CURRENT_XML)
    raw = load_raw_mjcf_inertials(CURRENT_XML)
    urdf_path = ASSET_DIR / "ff_master_ultra.urdf"
    urdf = load_urdf(urdf_path)
    rows: list[dict[str, str]] = []
    for name, current in compiled.items():
        raw_row = raw.get(name)
        urdf_row = urdf.get(name)
        if raw_row is None:
            category = "MESH_DERIVED_ESTIMATE"
            lineage = (
                "Robothon Master pelvis.STL -> ff_master_ultra.xml collision geom -> "
                "MuJoCo compile-time auto-inertia (default geom density); same-bundle URDF pelvis inertial is not used"
            )
            lineage_status = "CONFIRMED_XML_STRUCTURE_AND_COMPILED_RESULT"
            upstream = str(ASSET_DIR / "meshes" / "pelvis.STL")
            notes = "No explicit <inertial> exists on pelvis in current MJCF."
        else:
            category = "URDF_CONVERSION_NUMERIC_MATCH"
            upstream = str(urdf_path) if urdf_row is not None else "UNKNOWN"
            if urdf_row is None:
                lineage_status = "UNKNOWN_NO_URDF_ROW"
                lineage = "Robothon Master asset -> current MJCF; upstream inertial row not found"
                notes = "No same-name URDF inertial row."
            else:
                mass_delta = abs(float(current["mass"]) - float(urdf_row["mass"]))
                com_delta = float(np.max(np.abs(np.asarray(current["pos"]) - np.asarray(urdf_row["pos"]))))
                eig_delta = float(
                    np.max(
                        np.abs(
                            np.sort(np.asarray(current["diag"], dtype=float))
                            - np.sort(np.asarray(urdf_row["diag"], dtype=float))
                        )
                    )
                )
                matched = mass_delta <= 1e-4 and com_delta <= 1e-6 and eig_delta <= 2e-6
                lineage_status = "NUMERIC_MATCH_CONFIRMED" if matched else "PARTIAL_OR_NO_NUMERIC_MATCH"
                lineage = (
                    "Robothon Master same-bundle ff_master_ultra.urdf -> principal-axis MJCF representation; "
                    "conversion tool/history UNKNOWN"
                )
                notes = (
                    f"URDF comparison max deltas: mass={mass_delta:.6g} kg, "
                    f"CoM={com_delta:.6g} m, principal inertia={eig_delta:.6g} kg*m^2."
                )
        full = full_components(np.asarray(current["full"], dtype=float))
        rows.append(
            {
                "body": name,
                "region": region(name),
                "side": side(name),
                "mass_kg": fmt(float(current["mass"])),
                "inertial_pos_x_m": fmt(np.asarray(current["pos"])[0]),
                "inertial_pos_y_m": fmt(np.asarray(current["pos"])[1]),
                "inertial_pos_z_m": fmt(np.asarray(current["pos"])[2]),
                "inertial_quat_wxyz": fmt_vec(np.asarray(current["quat"])),
                "inertia_representation": str(raw_row["representation"] if raw_row is not None else current["representation"]),
                "diaginertia_kg_m2": fmt_vec(np.asarray(current["diag"])),
                **{f"full_{key}_kg_m2": value for key, value in full.items()},
                "current_source_file": str(CURRENT_XML),
                "upstream_candidate_file": upstream,
                "source_category": category,
                "source_lineage": lineage,
                "lineage_status": lineage_status,
                "hardware_evidence_level": "LEVEL_E_NOT_X2_PHYSICAL_CLOSURE",
                "x2_hardware_applicability": "UNKNOWN",
                "notes": notes,
            }
        )
    path = HERE / "phase3c_current_inertial_provenance.csv"
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def write_comparison() -> None:
    sources: list[tuple[str, Path, str, str, dict[str, dict[str, np.ndarray | float | str]]]] = [
        (
            "current_compiled_mjcf",
            CURRENT_XML,
            "MJCF_COMPILED",
            "BASELINE_CURRENT",
            load_compiled_mjcf(CURRENT_XML),
        ),
        (
            "x2_limits_compiled_mjcf",
            ASSET_DIR / "ff_master_ultra_x2_limits.xml",
            "MJCF_COMPILED",
            "DERIVED_LIMIT_VARIANT_NOT_INDEPENDENT_PHYSICAL_SOURCE",
            load_compiled_mjcf(ASSET_DIR / "ff_master_ultra_x2_limits.xml"),
        ),
    ]
    for filename in (
        "ff_master_ultra.urdf",
        "ff_master_ultra_simple_collision.urdf",
        "ff_master_fist.urdf",
        "ff_master_hand.urdf",
    ):
        path = ASSET_DIR / filename
        sources.append(
            (
                path.stem,
                path,
                "URDF",
                "SAME_ROBOTHON_MASTER_ASSET_FAMILY_NOT_INDEPENDENT_X2_PHYSICAL_SOURCE",
                load_urdf(path),
            )
        )
    baseline = sources[0][4]
    rows: list[dict[str, str]] = []
    for model_id, path, source_kind, independence, records in sources:
        for name, record in sorted(records.items()):
            base = baseline.get(name)
            mass_delta = None if base is None else float(record["mass"]) - float(base["mass"])
            com_delta = None if base is None else float(
                np.linalg.norm(np.asarray(record["pos"]) - np.asarray(base["pos"]))
            )
            inertia_delta = None if base is None else float(
                np.max(np.abs(np.asarray(record["full"]) - np.asarray(base["full"])))
            )
            full = full_components(np.asarray(record["full"], dtype=float))
            rows.append(
                {
                    "row_type": "BODY",
                    "model_id": model_id,
                    "source_file": str(path),
                    "source_kind": source_kind,
                    "independence_class": independence,
                    "body_or_metric": name,
                    "region": region(name),
                    "side": side(name),
                    "mass_or_value": fmt(float(record["mass"])),
                    "com_xyz_m": fmt_vec(np.asarray(record["pos"])),
                    "principal_inertia_kg_m2": fmt_vec(np.sort(np.asarray(record["diag"], dtype=float))),
                    "full_ixx_kg_m2": full["ixx"],
                    "full_iyy_kg_m2": full["iyy"],
                    "full_izz_kg_m2": full["izz"],
                    "full_ixy_kg_m2": full["ixy"],
                    "full_ixz_kg_m2": full["ixz"],
                    "full_iyz_kg_m2": full["iyz"],
                    "mass_delta_vs_current_kg": fmt(mass_delta),
                    "com_norm_delta_vs_current_m": fmt(com_delta),
                    "max_full_inertia_delta_vs_current_kg_m2": fmt(inertia_delta),
                    "hardware_evidence_level": "LEVEL_E_NOT_INDEPENDENT_X2_PHYSICAL_DATA",
                }
            )
        aggregates = aggregate(records)
        for metric, value in aggregates.items():
            rows.append(
                {
                    "row_type": "AGGREGATE",
                    "model_id": model_id,
                    "source_file": str(path),
                    "source_kind": source_kind,
                    "independence_class": independence,
                    "body_or_metric": metric,
                    "region": "",
                    "side": "",
                    "mass_or_value": fmt(value),
                    "com_xyz_m": "",
                    "principal_inertia_kg_m2": "",
                    "full_ixx_kg_m2": "",
                    "full_iyy_kg_m2": "",
                    "full_izz_kg_m2": "",
                    "full_ixy_kg_m2": "",
                    "full_ixz_kg_m2": "",
                    "full_iyz_kg_m2": "",
                    "mass_delta_vs_current_kg": "",
                    "com_norm_delta_vs_current_m": "",
                    "max_full_inertia_delta_vs_current_kg_m2": "",
                    "hardware_evidence_level": "LEVEL_E_NOT_INDEPENDENT_X2_PHYSICAL_DATA",
                }
            )

    # Phase 2H captured read-only excerpts from this AimDK X2 SDK simulator
    # artifact.  The complete remote XML is not copied into this workspace, so
    # only values visible in the immutable evidence capture are represented.
    # These rows establish manufacturer-SDK source lineage, not metrology.
    sdk_source = (
        "/mnt/c/Users/wesle/OneDrive/Documents/Agentech/Master Robot/"
        "AimDK_X2_SDK_v1.0.0/aimdk-aarch64-a424add7-artifacts/extra/"
        "x2_rl_deploy/x2_rl_deploy_mujoco/configuration/robot/"
        "lx2501_3_t2d5/model_info/x2.xml"
    )
    sdk_partial = {
        "left_hip_pitch_link": (1.39968, [0.00791, 0.064136, 0.000116], [0.70601, 0.705897, 0.0403038, 0.0404307], [0.00807979, 0.007409, 0.00155421]),
        "waist_pitch_link": (0.392413, [0.008967, -1e-06, 0.0], [0.5, 0.5, -0.5, 0.5], [0.00023, 0.000133, 0.000133]),
        "torso_link": (10.1441, [-0.000188, 0.000605, 0.191827], [0.999959, 0.00110321, 0.0081892, 0.00362262], [0.479863, 0.455999, 0.0666413]),
        "left_shoulder_pitch_link": (0.885896, [0.003007, 0.050014, 0.000062], [0.707145, 0.706403, 0.027325, 0.0139105], [0.00303104, 0.00292896, 0.000676999]),
        "left_shoulder_roll_link": (0.706556, [0.000621, -0.000183, -0.078842], [0.706401, 0.00587444, 0.00386922, 0.707777], [0.00549904, 0.00540092, 0.000539037]),
    }
    for name, (mass, pos_values, quat_values, diag_values) in sdk_partial.items():
        pos = np.asarray(pos_values, dtype=float)
        quat = np.asarray(quat_values, dtype=float)
        diag = np.asarray(diag_values, dtype=float)
        full_matrix = quat_to_rotation(quat) @ np.diag(diag) @ quat_to_rotation(quat).T
        full = full_components(full_matrix)
        base = baseline[name]
        rows.append(
            {
                "row_type": "BODY_PARTIAL_CAPTURE",
                "model_id": "aimdk_sdk_x2_xml",
                "source_file": sdk_source,
                "source_kind": "AIMDK_X2_SDK_MJCF_EXCERPT",
                "independence_class": "MANUFACTURER_SDK_SIMULATOR_ARTIFACT_NOT_METROLOGY",
                "body_or_metric": name,
                "region": region(name),
                "side": side(name),
                "mass_or_value": fmt(mass),
                "com_xyz_m": fmt_vec(pos),
                "principal_inertia_kg_m2": fmt_vec(np.sort(diag)),
                "full_ixx_kg_m2": full["ixx"],
                "full_iyy_kg_m2": full["iyy"],
                "full_izz_kg_m2": full["izz"],
                "full_ixy_kg_m2": full["ixy"],
                "full_ixz_kg_m2": full["ixz"],
                "full_iyz_kg_m2": full["iyz"],
                "mass_delta_vs_current_kg": fmt(mass - float(base["mass"])),
                "com_norm_delta_vs_current_m": fmt(float(np.linalg.norm(pos - np.asarray(base["pos"])))),
                "max_full_inertia_delta_vs_current_kg_m2": fmt(float(np.max(np.abs(full_matrix - np.asarray(base["full"]))))),
                "hardware_evidence_level": "LEVEL_A_SOURCE_LINEAGE_ONLY_NOT_PHYSICAL_METROLOGY",
            }
        )
    rows.append(
        {
            "row_type": "SOURCE_METADATA",
            "model_id": "aimdk_sdk_x2_xml",
            "source_file": sdk_source,
            "source_kind": "AIMDK_X2_SDK_MJCF_EXCERPT",
            "independence_class": "MANUFACTURER_SDK_SIMULATOR_ARTIFACT_NOT_METROLOGY",
            "body_or_metric": "sha256",
            "region": "",
            "side": "",
            "mass_or_value": "3ff43f05beb57412a804ba9fe05cd9adcdfce78e9ce73a95a71ac58ad20d91a3",
            "com_xyz_m": "",
            "principal_inertia_kg_m2": "",
            "full_ixx_kg_m2": "",
            "full_iyy_kg_m2": "",
            "full_izz_kg_m2": "",
            "full_ixy_kg_m2": "",
            "full_ixz_kg_m2": "",
            "full_iyz_kg_m2": "",
            "mass_delta_vs_current_kg": "",
            "com_norm_delta_vs_current_m": "",
            "max_full_inertia_delta_vs_current_kg_m2": "",
            "hardware_evidence_level": "LEVEL_A_SOURCE_LINEAGE_ONLY_NOT_PHYSICAL_METROLOGY",
        }
    )
    path = HERE / "phase3c_inertial_source_comparison.csv"
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    write_provenance()
    write_comparison()
    print(f"wrote {HERE / 'phase3c_current_inertial_provenance.csv'}")
    print(f"wrote {HERE / 'phase3c_inertial_source_comparison.csv'}")
