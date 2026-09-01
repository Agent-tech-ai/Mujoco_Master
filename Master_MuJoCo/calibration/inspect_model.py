"""Print the effective MuJoCo joints, actuators, and sensors for MJCF models.

Values are read from a compiled ``MjModel`` so MJCF defaults and includes have
already been resolved.  The command does not modify any model file.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

import mujoco


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODELS = (
    PROJECT_ROOT / "assets" / "Master" / "ff_master_ultra.xml",
    PROJECT_ROOT / "assets" / "Master" / "ff_master_ultra_x2_limits.xml",
    PROJECT_ROOT / "assets" / "Master" / "scene_x2_fixed.xml",
    PROJECT_ROOT / "assets" / "Master" / "scene_x2_free.xml",
)


def _name(model: mujoco.MjModel, obj_type: mujoco.mjtObj, obj_id: int) -> str:
    value = mujoco.mj_id2name(model, obj_type, obj_id)
    return value if value is not None else f"UNNAMED_{obj_id}"


def _enum_name(enum_type: Any, value: int) -> str:
    try:
        return enum_type(int(value)).name
    except (ValueError, TypeError):
        return f"UNKNOWN_{int(value)}"


def _dof_count(joint_type: int) -> int:
    joint_type = int(joint_type)
    if joint_type == int(mujoco.mjtJoint.mjJNT_FREE):
        return 6
    if joint_type == int(mujoco.mjtJoint.mjJNT_BALL):
        return 3
    return 1


def _float_list(values: Any) -> list[float]:
    return [float(value) for value in values]


def _range(values: Any, limited: bool) -> dict[str, Any]:
    return {"limited": bool(limited), "values": _float_list(values)}


def _sensor_target(model: mujoco.MjModel, sensor_id: int) -> dict[str, Any]:
    obj_type_value = int(model.sensor_objtype[sensor_id])
    obj_id = int(model.sensor_objid[sensor_id])
    obj_type_name = _enum_name(mujoco.mjtObj, obj_type_value)
    obj_name = "N/A"
    if obj_id >= 0:
        try:
            obj_name = _name(model, mujoco.mjtObj(obj_type_value), obj_id)
        except ValueError:
            obj_name = f"ID_{obj_id}"
    return {"object_type": obj_type_name, "object_name": obj_name}


def inspect(path: Path) -> dict[str, Any]:
    path = path.resolve()
    model = mujoco.MjModel.from_xml_path(str(path))

    sensors: list[dict[str, Any]] = []
    joint_sensors: dict[int, list[str]] = {}
    actuator_sensors: dict[int, list[str]] = {}
    for sensor_id in range(model.nsensor):
        sensor_name = _name(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_id)
        sensor_type = _enum_name(mujoco.mjtSensor, model.sensor_type[sensor_id])
        target = _sensor_target(model, sensor_id)
        sensor = {
            "id": sensor_id,
            "name": sensor_name,
            "type": sensor_type,
            "dimension": int(model.sensor_dim[sensor_id]),
            "target": target,
            "noise": float(model.sensor_noise[sensor_id]),
            "cutoff": float(model.sensor_cutoff[sensor_id]),
        }
        sensors.append(sensor)
        obj_type = int(model.sensor_objtype[sensor_id])
        obj_id = int(model.sensor_objid[sensor_id])
        label = f"{sensor_type}:{sensor_name}"
        if obj_type == int(mujoco.mjtObj.mjOBJ_JOINT) and obj_id >= 0:
            joint_sensors.setdefault(obj_id, []).append(label)
        elif obj_type == int(mujoco.mjtObj.mjOBJ_ACTUATOR) and obj_id >= 0:
            actuator_sensors.setdefault(obj_id, []).append(label)

    actuators_by_joint: dict[int, list[dict[str, Any]]] = {}
    actuators: list[dict[str, Any]] = []
    joint_transmissions = {
        int(mujoco.mjtTrn.mjTRN_JOINT),
        int(mujoco.mjtTrn.mjTRN_JOINTINPARENT),
    }
    for actuator_id in range(model.nu):
        transmission_type = int(model.actuator_trntype[actuator_id])
        target_id = int(model.actuator_trnid[actuator_id, 0])
        actuator = {
            "id": actuator_id,
            "name": _name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id),
            "transmission_type": _enum_name(mujoco.mjtTrn, transmission_type),
            "target_joint": (
                _name(model, mujoco.mjtObj.mjOBJ_JOINT, target_id)
                if transmission_type in joint_transmissions and target_id >= 0
                else "N/A"
            ),
            "ctrlrange": _range(
                model.actuator_ctrlrange[actuator_id],
                bool(model.actuator_ctrllimited[actuator_id]),
            ),
            "forcerange": _range(
                model.actuator_forcerange[actuator_id],
                bool(model.actuator_forcelimited[actuator_id]),
            ),
            "sensors": actuator_sensors.get(actuator_id, []),
        }
        actuators.append(actuator)
        if transmission_type in joint_transmissions and target_id >= 0:
            actuators_by_joint.setdefault(target_id, []).append(actuator)

    joints: list[dict[str, Any]] = []
    for joint_id in range(model.njnt):
        joint_type = int(model.jnt_type[joint_id])
        dof_start = int(model.jnt_dofadr[joint_id])
        dof_end = dof_start + _dof_count(joint_type)
        joint_actuators = actuators_by_joint.get(joint_id, [])
        joint_sensor_labels = list(joint_sensors.get(joint_id, []))
        for actuator in joint_actuators:
            joint_sensor_labels.extend(actuator["sensors"])
        is_axis_joint = joint_type in {
            int(mujoco.mjtJoint.mjJNT_HINGE),
            int(mujoco.mjtJoint.mjJNT_SLIDE),
        }
        joints.append(
            {
                "id": joint_id,
                "name": _name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id),
                "type": _enum_name(mujoco.mjtJoint, joint_type),
                "body_link": _name(
                    model,
                    mujoco.mjtObj.mjOBJ_BODY,
                    int(model.jnt_bodyid[joint_id]),
                ),
                "axis": _float_list(model.jnt_axis[joint_id]) if is_axis_joint else None,
                "range": _range(
                    model.jnt_range[joint_id], bool(model.jnt_limited[joint_id])
                ),
                "damping": _float_list(model.dof_damping[dof_start:dof_end]),
                "armature": _float_list(model.dof_armature[dof_start:dof_end]),
                "frictionloss": _float_list(model.dof_frictionloss[dof_start:dof_end]),
                "joint_actuator_force_range": _range(
                    model.jnt_actfrcrange[joint_id],
                    bool(model.jnt_actfrclimited[joint_id]),
                ),
                "actuators": joint_actuators,
                "sensors": sorted(set(joint_sensor_labels)),
            }
        )

    return {
        "file": str(path),
        "model_name": path.stem,
        "counts": {
            "joints": model.njnt,
            "qpos": model.nq,
            "dofs": model.nv,
            "actuators": model.nu,
            "sensors": model.nsensor,
            "bodies": model.nbody,
        },
        "joints": joints,
        "actuators": actuators,
        "sensors": sensors,
    }


def _format_values(values: list[float] | None) -> str:
    if values is None:
        return "N/A"
    return "[" + ", ".join(f"{value:.9g}" for value in values) + "]"


def _format_range(value: dict[str, Any]) -> str:
    prefix = "limited" if value["limited"] else "unlimited"
    return f"{prefix} {_format_values(value['values'])}"


def format_text(reports: list[dict[str, Any]]) -> str:
    lines: list[str] = []
    for report_index, report in enumerate(reports):
        if report_index:
            lines.append("")
        counts = report["counts"]
        lines.extend(
            [
                "=" * 96,
                f"MODEL: {report['file']}",
                (
                    "COUNTS: "
                    f"joints={counts['joints']} qpos={counts['qpos']} "
                    f"dofs={counts['dofs']} actuators={counts['actuators']} "
                    f"sensors={counts['sensors']} bodies={counts['bodies']}"
                ),
                "-" * 96,
                "JOINTS (compiled/effective values; angular quantities use radians)",
            ]
        )
        for joint in report["joints"]:
            lines.append(
                f"[{joint['id']:02d}] {joint['name']} | body/link={joint['body_link']} "
                f"| type={joint['type']}"
            )
            lines.append(
                f"     axis={_format_values(joint['axis'])} "
                f"| range={_format_range(joint['range'])}"
            )
            lines.append(
                f"     damping={_format_values(joint['damping'])} "
                f"| armature={_format_values(joint['armature'])} "
                f"| frictionloss={_format_values(joint['frictionloss'])}"
            )
            lines.append(
                "     joint actuatorfrcrange="
                f"{_format_range(joint['joint_actuator_force_range'])}"
            )
            if joint["actuators"]:
                for actuator in joint["actuators"]:
                    lines.append(
                        f"     actuator={actuator['name']} "
                        f"| ctrlrange={_format_range(actuator['ctrlrange'])} "
                        f"| actuator forcerange={_format_range(actuator['forcerange'])}"
                    )
            else:
                lines.append("     actuator=N/A | ctrlrange=N/A | actuator forcerange=N/A")
            lines.append(
                "     sensors=" + (", ".join(joint["sensors"]) or "NONE")
            )

        lines.extend(["-" * 96, "ALL SENSORS"])
        for sensor in report["sensors"]:
            target = sensor["target"]
            lines.append(
                f"[{sensor['id']:02d}] {sensor['name']} | type={sensor['type']} "
                f"| dim={sensor['dimension']} | target={target['object_type']}:"
                f"{target['object_name']} | noise={sensor['noise']:.9g} "
                f"| cutoff={sensor['cutoff']:.9g}"
            )
    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model",
        action="append",
        type=Path,
        help="MJCF path; repeat for multiple files. Defaults to the four audited models.",
    )
    parser.add_argument("--format", choices=("text", "json"), default="text")
    parser.add_argument("--output", type=Path, help="Also write the report to this path.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    paths = args.model or list(DEFAULT_MODELS)
    try:
        reports = [inspect(path) for path in paths]
    except (OSError, ValueError, mujoco.FatalError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    output = (
        json.dumps(reports, ensure_ascii=False, indent=2) + "\n"
        if args.format == "json"
        else format_text(reports)
    )
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output, encoding="utf-8")
    sys.stdout.write(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
