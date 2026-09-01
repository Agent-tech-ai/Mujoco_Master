#!/usr/bin/env python3
"""Build a local, simulation-only pelvis/hip distance sensitivity table."""

from __future__ import annotations

import math

import mujoco
import numpy as np
import pandas as pd

from phase3ax_core import HERE, P3AR, datasets


def main() -> int:
    dataset = datasets()["wave"]
    reference_frame, _real = P3AR.load_frames(dataset)
    t0 = max(float(reference_frame.t.min()), -5.0)
    reference = P3AR.P3A.Reference(reference_frame, "linear", 50.0)
    model = P3AR.load_model(free_base=True)
    data = mujoco.MjData(model)
    base_controller = P3AR.RobustController(model, P3AR.Design("probe", "PROBE", "probe", "n/a", "n/a"))
    for joint in base_controller.joints:
        if joint.name in reference.data:
            data.qpos[joint.qpos_adr] = reference.at(joint.name, t0, "position")
    mujoco.mj_forward(model, data)
    data.qpos[2] -= P3AR.P3A.foot_surface_minimum(model, data)
    mujoco.mj_forward(model, data)
    geoms = P3AR.geom_info(model)
    nominal = data.qpos.copy()

    cases: list[tuple[str, str, float]] = [("nominal", "none", 0.0)]
    for name in (
        "left_hip_roll_joint", "left_hip_pitch_joint",
        "right_hip_roll_joint", "right_hip_pitch_joint",
        "waist_roll_joint", "waist_pitch_joint",
    ):
        cases.extend(((f"{name}_plus_0p25deg", name, 0.25), (f"{name}_minus_0p25deg", name, -0.25)))
    cases.extend((
        ("pelvis_roll_plus_0p25deg", "pelvis_roll", 0.25),
        ("pelvis_roll_minus_0p25deg", "pelvis_roll", -0.25),
        ("pelvis_pitch_plus_0p25deg", "pelvis_pitch", 0.25),
        ("pelvis_pitch_minus_0p25deg", "pelvis_pitch", -0.25),
        ("safe_standing_left_hip_roll_plus_0p025rad", "left_hip_roll_joint", math.degrees(0.025)),
    ))
    by_name = {joint.name: joint for joint in base_controller.joints}
    rows = []
    for case, variable, delta_deg in cases:
        data.qpos[:] = nominal
        if variable in by_name:
            data.qpos[by_name[variable].qpos_adr] += math.radians(delta_deg)
        elif variable.startswith("pelvis_"):
            roll = math.radians(delta_deg) if variable == "pelvis_roll" else 0.0
            pitch = math.radians(delta_deg) if variable == "pelvis_pitch" else 0.0
            quat = np.zeros(4, dtype=np.float64)
            mujoco.mju_euler2Quat(quat, np.array([roll, pitch, 0.0], dtype=np.float64), "xyz")
            data.qpos[3:7] = quat
        mujoco.mj_forward(model, data)
        left, _ = P3AR.pair_distance(model, data, geoms["pelvis"], geoms["left_hip"])
        right, _ = P3AR.pair_distance(model, data, geoms["pelvis"], geoms["right_hip"])
        rows.append({
            "case": case, "variable": variable, "delta_deg": delta_deg,
            "left_distance_m": left, "right_distance_m": right,
        })
    frame = pd.DataFrame(rows)
    base = frame.iloc[0]
    frame["left_delta_from_nominal_m"] = frame.left_distance_m - float(base.left_distance_m)
    frame["right_delta_from_nominal_m"] = frame.right_distance_m - float(base.right_distance_m)
    frame["warning_zone_m"] = 0.003
    frame["hard_zone_m"] = 0.00075
    frame["classification"] = "SIMULATION_LOCAL_SENSITIVITY; NOT ROBOT GEOMETRY CALIBRATION"
    frame.to_csv(HERE / "phase3ax_contact_margin_sensitivity.csv", index=False)
    print(frame.to_string(index=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
