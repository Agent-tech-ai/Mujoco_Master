#!/usr/bin/env python3
"""Run frozen/current contact and whole-body diagnostic scenarios."""

from __future__ import annotations

import json

import mujoco
import pandas as pd

from phase3ar_core import (
    Design,
    RUNS,
    datasets,
    geom_info,
    load_frames,
    load_model,
    pair_distance,
    run_replay,
    run_standing,
    standing_offsets,
)


def static_geometry_table() -> pd.DataFrame:
    model = load_model(free_base=True)
    joint_ids = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index): index
        for index in range(model.njnt)
    }
    geoms = geom_info(model)
    design = Design("current_07", "CURRENT", "none", "none", "none")
    offsets = standing_offsets(design)
    rows = []
    for dataset_name, dataset in datasets().items():
        reference, _ = load_frames(dataset)
        initial = {}
        for name, group in reference.groupby("joint_name"):
            group = group.sort_values("t")
            initial[name] = float(group.iloc[int((group.t + 3.0).abs().argmin())].position)
        for scale in (0.0, 0.25, 0.5, 0.75, 1.0):
            data = mujoco.MjData(model)
            for name, value in initial.items():
                joint_id = joint_ids.get(name)
                if joint_id is None:
                    continue
                qadr = int(model.jnt_qposadr[joint_id])
                data.qpos[qadr] = value + scale * offsets.get(name, 0.0)
            mujoco.mj_forward(model, data)
            left, _ = pair_distance(model, data, geoms["pelvis"], geoms["left_hip"])
            right, _ = pair_distance(model, data, geoms["pelvis"], geoms["right_hip"])
            rows.append(
                {
                    "dataset": dataset_name,
                    "state": "raw_initial" if scale == 0.0 else f"static_offset_scale_{scale:.2f}",
                    "standing_offset_scale": scale,
                    "left_signed_distance_m": left,
                    "right_signed_distance_m": right,
                    "left_minus_right_m": left - right,
                }
            )
    return pd.DataFrame(rows)


def main() -> int:
    RUNS.mkdir(parents=True, exist_ok=True)
    source = datasets()
    current = Design("current_07", "CURRENT_PHASE3A", "global balance scale", "1.0", "0.7")
    legacy = Design(
        "legacy_phase3av",
        "LEGACY_PHASE3AV",
        "controller baseline",
        "current candidate",
        "legacy",
        standing_reference_scale=0.0,
        pitch_kp=200.0,
        pitch_kd=30.0,
        roll_kp=100.0,
        roll_kd=20.0,
        shoulder_gain_scale=1.0,
        wrist_gain_scale=1.0,
    )
    scenarios = []
    for label, callback in (
        ("current heart standing", lambda: run_standing(current, source["heart"], save_detail=True)),
        ("current wave standing", lambda: run_standing(current, source["wave"], save_detail=True)),
        ("current heart arm-only", lambda: run_replay(current, source["heart"], "arm_only", save_detail=True)),
        ("current wave arm-only", lambda: run_replay(current, source["wave"], "arm_only", save_detail=True)),
        ("current wave whole-body", lambda: run_replay(current, source["wave"], "whole_body", save_detail=True)),
        ("legacy wave arm-only", lambda: run_replay(legacy, source["wave"], "arm_only", save_detail=True)),
    ):
        print(f"RUN {label}", flush=True)
        scenarios.append(callback())
    static_geometry_table().to_csv(RUNS / "static_pelvis_hip_geometry.csv", index=False)
    (RUNS / "diagnostic_scenarios.json").write_text(json.dumps(scenarios, indent=2), encoding="utf-8")
    print(json.dumps({"diagnostic_scenarios": len(scenarios)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
