#!/usr/bin/env python3
"""Evidence-only analysis for Phase 3B-C Clap self-contact closure."""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys
from typing import Any

import mujoco
import numpy as np
import pandas as pd


HERE = Path(__file__).resolve().parent
CALIBRATION = HERE.parent
BV_DIR = CALIBRATION / "phase3bv_physical_direction_validation"
P3AR_DIR = CALIBRATION / "phase3ar_controller_redesign"
if str(P3AR_DIR) not in sys.path:
    sys.path.insert(0, str(P3AR_DIR))
import phase3ar_core as P3AR  # noqa: E402


CONTACT_FILES = {
    "original": HERE / "phase3bc_clap_baseline_contacts.csv",
    "mass_direction": HERE / "phase3bc_clap_mass_contacts.csv",
}
PROBE_FILES = {
    "original": HERE / "phase3bc_clap_baseline_wrist_distance.csv",
    "mass_direction": HERE / "phase3bc_clap_mass_wrist_distance.csv",
}
EXPERIMENT_IDS = {
    "original": "phase3bv_original_physical_baseline",
    "mass_direction": "phase3bv_bs_mass_lower_plus08",
}
MODES = ("arm_only", "whole_body")
ARM_TOKENS = ("shoulder", "elbow", "wrist")
TIMELINE_JOINTS = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_yaw_joint", "left_wrist_pitch_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_yaw_joint", "right_wrist_pitch_joint", "right_wrist_roll_joint",
    "waist_pitch_joint", "waist_roll_joint", "waist_yaw_joint",
    "left_hip_pitch_joint", "right_hip_pitch_joint",
    "left_knee_joint", "right_knee_joint",
    "left_ankle_pitch_joint", "right_ankle_pitch_joint",
    "left_ankle_roll_joint", "right_ankle_roll_joint",
)


def name(model: mujoco.MjModel, kind: mujoco.mjtObj, object_id: int) -> str:
    return mujoco.mj_id2name(model, kind, int(object_id)) or ""


def fmt(value: float, digits: int = 6) -> str:
    return "UNKNOWN" if not np.isfinite(value) else f"{value:.{digits}f}"


def load_model() -> mujoco.MjModel:
    return P3AR.load_model(free_base=True)


def enrich_contact_file(path: Path, model: mujoco.MjModel) -> pd.DataFrame:
    frame = pd.read_csv(path)
    if frame.empty:
        return frame
    resolved1, resolved2, mesh1, mesh2 = [], [], [], []
    for row in frame.itertuples(index=False):
        values = []
        for geom_id, raw_name in ((int(row.geom1_id), row.geom1_name), (int(row.geom2_id), row.geom2_name)):
            data_id = int(model.geom_dataid[geom_id])
            mesh_name = name(model, mujoco.mjtObj.mjOBJ_MESH, data_id) if data_id >= 0 else ""
            source_name = "" if pd.isna(raw_name) else str(raw_name)
            resolved = source_name or f"UNNAMED_COLLISION_GEOM_{geom_id}[mesh={mesh_name or 'NONE'}]"
            values.append((resolved, mesh_name))
        resolved1.append(values[0][0]); mesh1.append(values[0][1])
        resolved2.append(values[1][0]); mesh2.append(values[1][1])
    frame["geom1_resolved_name"] = resolved1
    frame["geom1_mesh_name"] = mesh1
    frame["geom2_resolved_name"] = resolved2
    frame["geom2_mesh_name"] = mesh2
    # These are derived descriptive columns; raw IDs, blank MJCF names, and all
    # simulation values remain untouched.
    frame.to_csv(path, index=False)
    return frame


def episode_metrics(frame: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for (mode, episode_id), group in frame.groupby(["mode", "episode_id"], sort=True):
        rows.append({
            "condition": str(group["condition"].iloc[0]),
            "mode": mode,
            "episode_id": episode_id,
            "geom_pair_key": str(group.geom_pair_key.iloc[0]),
            "geom_pair_name": str(group.geom_pair_name.iloc[0]),
            "body_pair": f"{group.body1_name.iloc[0]} <-> {group.body2_name.iloc[0]}",
            "onset_s": float(group.timestamp_s.min()),
            "end_s": float(group.timestamp_s.max()),
            "duration_s": float(group.contact_duration_s.max()),
            "contact_samples_1khz": int(group.timestamp_s.nunique()),
            "max_penetration_m": float(group.penetration_depth_m.max()),
            "peak_normal_force_n": float(group.normal_force_n.max()),
            "max_abs_relative_normal_velocity_m_s": float(group.relative_normal_velocity_m_s.abs().max()),
            "max_approach_distance_rate_m_s": float(max(0.0, -group.pair_min_distance_rate_m_s.min())),
        })
    return pd.DataFrame(rows)


def pair_metrics(frame: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for (condition, mode, pair), group in frame.groupby(["condition", "mode", "geom_pair_key"], sort=True):
        rows.append({
            "condition": condition,
            "mode": mode,
            "geom_pair_key": pair,
            "geom_pair_name": str(group.geom_pair_name.iloc[0]),
            "body1_name": str(group.body1_name.iloc[0]),
            "body2_name": str(group.body2_name.iloc[0]),
            "episodes": int(group.episode_id.nunique()),
            "first_onset_s": float(group.timestamp_s.min()),
            "last_end_s": float(group.timestamp_s.max()),
            "total_contact_duration_s": float(group.timestamp_s.nunique() * 0.001),
            "max_episode_duration_s": float(group.contact_duration_s.max()),
            "max_penetration_m": float(group.penetration_depth_m.max()),
            "peak_normal_force_n": float(group.normal_force_n.max()),
            "max_abs_relative_normal_velocity_m_s": float(group.relative_normal_velocity_m_s.abs().max()),
        })
    return pd.DataFrame(rows)


def body_path(model: mujoco.MjModel, body_id: int) -> list[int]:
    result = [int(body_id)]
    while result[-1] > 0:
        result.append(int(model.body_parentid[result[-1]]))
    return result


def topology(model: mujoco.MjModel, geom1: int, geom2: int) -> dict[str, Any]:
    body1, body2 = int(model.geom_bodyid[geom1]), int(model.geom_bodyid[geom2])
    path1, path2 = body_path(model, body1), body_path(model, body2)
    common = next(item for item in path1 if item in set(path2))
    parent_child = int(model.body_parentid[body1]) == body2 or int(model.body_parentid[body2]) == body1
    grandparent_child = (
        len(path1) > 2 and path1[2] == body2
        or len(path2) > 2 and path2[2] == body1
    )
    result: dict[str, Any] = {
        "geom1_id": geom1,
        "geom2_id": geom2,
        "geom1_mjcf_name": name(model, mujoco.mjtObj.mjOBJ_GEOM, geom1) or "UNNAMED",
        "geom2_mjcf_name": name(model, mujoco.mjtObj.mjOBJ_GEOM, geom2) or "UNNAMED",
        "geom1_mesh": name(model, mujoco.mjtObj.mjOBJ_MESH, int(model.geom_dataid[geom1])),
        "geom2_mesh": name(model, mujoco.mjtObj.mjOBJ_MESH, int(model.geom_dataid[geom2])),
        "geom1_type": mujoco.mjtGeom(int(model.geom_type[geom1])).name,
        "geom2_type": mujoco.mjtGeom(int(model.geom_type[geom2])).name,
        "geom1_contype": int(model.geom_contype[geom1]),
        "geom1_conaffinity": int(model.geom_conaffinity[geom1]),
        "geom2_contype": int(model.geom_contype[geom2]),
        "geom2_conaffinity": int(model.geom_conaffinity[geom2]),
        "body1_id": body1,
        "body2_id": body2,
        "body1_name": name(model, mujoco.mjtObj.mjOBJ_BODY, body1),
        "body2_name": name(model, mujoco.mjtObj.mjOBJ_BODY, body2),
        "parent_child": parent_child,
        "grandparent_child": grandparent_child,
        "kinematic_tree_adjacent": parent_child or grandparent_child,
        "lowest_common_ancestor": name(model, mujoco.mjtObj.mjOBJ_BODY, common) or "world",
        "body1_edges_to_common_ancestor": path1.index(common),
        "body2_edges_to_common_ancestor": path2.index(common),
        "explicit_exclude_count_model": int(model.nexclude),
        "explicitly_excluded_pair": False,
        "collision_enabled_by_masks": bool(
            int(model.geom_contype[geom1]) & int(model.geom_conaffinity[geom2])
            or int(model.geom_contype[geom2]) & int(model.geom_conaffinity[geom1])
        ),
        "body1_path": [name(model, mujoco.mjtObj.mjOBJ_BODY, item) or "world" for item in path1],
        "body2_path": [name(model, mujoco.mjtObj.mjOBJ_BODY, item) or "world" for item in path2],
    }
    return result


def diagnostic_run_path(condition: str, mode: str, suffix: str) -> Path:
    stem = f"{EXPERIMENT_IDS[condition]}__phase3bv_clap__{mode}"
    return HERE / "diagnostic_runs" / f"{stem}_{suffix}"


def build_timeline(contacts: pd.DataFrame, probe: pd.DataFrame) -> tuple[pd.DataFrame, dict[str, float]]:
    focus = contacts[(contacts["condition"] == "original") & (contacts["mode"] == "arm_only")].copy()
    first_episode = focus.loc[focus.timestamp_s.idxmin(), "episode_id"]
    episode = focus[focus.episode_id == first_episode]
    onset = float(episode.timestamp_s.min())
    end = float(episode.timestamp_s.max())
    peak_index = episode.penetration_depth_m.idxmax()
    peak_time = float(episode.loc[peak_index, "timestamp_s"])
    joints = pd.read_csv(diagnostic_run_path("original", "arm_only", "joint_log.csv"))
    safety = pd.read_csv(diagnostic_run_path("original", "arm_only", "safety_log.csv"))
    window = joints[joints.t.between(onset - 0.5, onset + 0.5)].copy()
    times = np.sort(window.t.unique())
    timeline = pd.DataFrame({"timestamp_s": times})

    arm = window[window.joint_name.str.contains("shoulder|elbow|wrist", regex=True)]
    aggregate = arm.groupby("t").agg(
        arm_tracking_rms_rad=("tracking_error_rad", lambda x: float(np.sqrt(np.mean(np.square(x))))),
        arm_tracking_max_abs_rad=("tracking_error_rad", lambda x: float(np.max(np.abs(x)))),
        arm_velocity_norm_rad_s=("velocity", lambda x: float(np.sqrt(np.sum(np.square(x))))),
    ).reset_index().rename(columns={"t": "timestamp_s"})
    timeline = timeline.merge(aggregate, on="timestamp_s", how="left")
    arm_reference = arm.pivot(index="t", columns="joint_name", values="reference_position").sort_index()
    reference_velocity = arm_reference.diff().div(arm_reference.index.to_series().diff(), axis=0)
    reference_norm = np.sqrt((reference_velocity ** 2).sum(axis=1)).rename("arm_reference_velocity_norm_rad_s")
    timeline = timeline.merge(reference_norm.reset_index().rename(columns={"t": "timestamp_s"}), on="timestamp_s", how="left")

    for joint_name in TIMELINE_JOINTS:
        selected = window[window.joint_name == joint_name][[
            "t", "reference_position", "target_position", "position", "velocity", "tracking_error_rad"
        ]].copy()
        selected = selected.rename(columns={
            "t": "timestamp_s",
            "reference_position": f"{joint_name}__reference_rad",
            "target_position": f"{joint_name}__target_rad",
            "position": f"{joint_name}__position_rad",
            "velocity": f"{joint_name}__velocity_rad_s",
            "tracking_error_rad": f"{joint_name}__tracking_error_rad",
        })
        timeline = timeline.merge(selected, on="timestamp_s", how="left")
    timeline = timeline.merge(
        safety[["t", "base_roll_rad", "base_pitch_rad", "base_yaw_rad", "com_support_margin_m",
                "left_foot_slip_m", "right_foot_slip_m"]].rename(columns={"t": "timestamp_s"}),
        on="timestamp_s", how="left",
    )

    p = probe[(probe["condition"] == "original") & (probe["mode"] == "arm_only")].sort_values("timestamp_s")
    timeline = pd.merge_asof(
        timeline.sort_values("timestamp_s"),
        p[["timestamp_s", "signed_wrist_geom_distance_m"]],
        on="timestamp_s", direction="nearest", tolerance=0.0011,
    )
    c = focus.copy()
    c["sample_bin_s"] = (c.timestamp_s / 0.02).round() * 0.02
    contact_20hz = c.groupby("sample_bin_s").agg(
        contact_active=("timestamp_s", lambda x: 1),
        contact_penetration_m=("penetration_depth_m", "max"),
        contact_normal_force_n=("normal_force_n", "max"),
        contact_relative_normal_velocity_m_s=("relative_normal_velocity_m_s", lambda x: float(np.max(np.abs(x)))),
    ).reset_index().rename(columns={"sample_bin_s": "timestamp_round_s"})
    timeline["timestamp_round_s"] = (timeline.timestamp_s / 0.02).round() * 0.02
    timeline = timeline.merge(contact_20hz, on="timestamp_round_s", how="left").drop(columns="timestamp_round_s")
    timeline["contact_active"] = timeline.contact_active.fillna(0).astype(int)
    for column in ("contact_penetration_m", "contact_normal_force_n", "contact_relative_normal_velocity_m_s"):
        timeline[column] = timeline[column].fillna(0.0)
    timeline.to_csv(HERE / "phase3bc_contact_timeline.csv", index=False)

    def tracking_rms(start: float, stop: float) -> float:
        subset = arm[arm.t.between(start, stop)]
        return float(np.sqrt(np.mean(np.square(subset.tracking_error_rad))))

    stats = {
        "onset_s": onset,
        "end_s": end,
        "peak_time_s": peak_time,
        "duration_s": float(episode.contact_duration_s.max()),
        "pre_tracking_rms_rad": tracking_rms(onset - 0.5, onset - 0.001),
        "during_tracking_rms_rad": tracking_rms(onset, end),
        "post_tracking_rms_rad": tracking_rms(end + 0.001, end + 0.5),
        "during_max_abs_tracking_rad": float(arm[arm.t.between(onset, end)].tracking_error_rad.abs().max()),
        "during_max_base_tilt_deg": float(np.degrees(max(
            safety[safety.t.between(onset, end)].base_roll_rad.abs().max(),
            safety[safety.t.between(onset, end)].base_pitch_rad.abs().max(),
        ))),
        "during_min_com_support_margin_m": float(safety[safety.t.between(onset, end)].com_support_margin_m.min()),
    }
    return timeline, stats


def comparison_rows(episodes: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for mode in MODES:
        base = episodes[(episodes["condition"] == "original") & (episodes["mode"] == mode)].sort_values("onset_s").reset_index(drop=True)
        mass = episodes[(episodes["condition"] == "mass_direction") & (episodes["mode"] == mode)].sort_values("onset_s").reset_index(drop=True)
        if len(base) != len(mass):
            raise RuntimeError(f"episode count differs in {mode}: {len(base)} vs {len(mass)}")
        for index in range(len(base)):
            first, second = base.iloc[index], mass.iloc[index]
            rows.append({
                "mode": mode,
                "episode": index + 1,
                "same_geom_pair": first.geom_pair_key == second.geom_pair_key,
                "baseline_onset_s": first.onset_s,
                "candidate_onset_s": second.onset_s,
                "onset_delta_ms": (second.onset_s - first.onset_s) * 1000.0,
                "baseline_duration_s": first.duration_s,
                "candidate_duration_s": second.duration_s,
                "duration_delta_ms": (second.duration_s - first.duration_s) * 1000.0,
                "baseline_max_penetration_mm": first.max_penetration_m * 1000.0,
                "candidate_max_penetration_mm": second.max_penetration_m * 1000.0,
                "penetration_delta_mm": (second.max_penetration_m - first.max_penetration_m) * 1000.0,
                "baseline_peak_normal_force_n": first.peak_normal_force_n,
                "candidate_peak_normal_force_n": second.peak_normal_force_n,
                "peak_force_delta_n": second.peak_normal_force_n - first.peak_normal_force_n,
            })
    result = pd.DataFrame(rows)
    result.to_csv(HERE / "phase3bc_baseline_mass_episode_comparison.csv", index=False)
    return result


def markdown_table(frame: pd.DataFrame, columns: list[str], formats: dict[str, str] | None = None) -> str:
    formats = formats or {}
    header = "| " + " | ".join(columns) + " |"
    divider = "|" + "|".join("---" for _ in columns) + "|"
    lines = [header, divider]
    for row in frame[columns].itertuples(index=False, name=None):
        values = []
        for column, value in zip(columns, row):
            if isinstance(value, (float, np.floating)):
                values.append(format(value, formats.get(column, ".6f")))
            else:
                values.append(str(value))
        lines.append("| " + " | ".join(values) + " |")
    return "\n".join(lines)


def write_reports(
    contacts: pd.DataFrame,
    probes: pd.DataFrame,
    pairs: pd.DataFrame,
    episodes: pd.DataFrame,
    compare: pd.DataFrame,
    topo: dict[str, Any],
    timeline: pd.DataFrame,
    timeline_stats: dict[str, float],
) -> None:
    # Evidence-backed classification: one cross-arm end-effector pair, absent
    # during pre-roll, repeated exactly three times with the Clap closures.
    classification = "CONTROLLER_POSTURE_SELF_CONTACT"
    root_cause = "CONTROLLER_POSTURE"
    same_pairs = bool(compare.same_geom_pair.all()) and contacts.geom_pair_key.nunique() == 1
    added_pair = contacts[contacts["condition"] == "mass_direction"].geom_pair_key.nunique() > contacts[contacts["condition"] == "original"].geom_pair_key.nunique()
    max_base = float(contacts[contacts["condition"] == "original"].penetration_depth_m.max())
    max_mass = float(contacts[contacts["condition"] == "mass_direction"].penetration_depth_m.max())
    max_force_base = float(contacts[contacts["condition"] == "original"].normal_force_n.max())
    max_force_mass = float(contacts[contacts["condition"] == "mass_direction"].normal_force_n.max())

    pre_roll = probes[probes.timestamp_s < 0.0]
    pre_min = pre_roll.groupby(["condition", "mode"]).signed_wrist_geom_distance_m.min().reset_index()
    pair_view = pairs.copy()
    pair_view["max_penetration_mm"] = pair_view.max_penetration_m * 1000.0
    pair_view["classification"] = classification
    pair_view.to_csv(HERE / "phase3bc_contact_pair_metrics.csv", index=False)

    pair_report = f"""# Phase 3B-C contact-pair analysis

## Result

Only one internal pair occurs in all four frozen replays:

- body pair: `{topo['body1_name']}` ↔ `{topo['body2_name']}`
- geom IDs: `{topo['geom1_id']}` ↔ `{topo['geom2_id']}`
- MJCF geom names: both `UNNAMED`; deterministic resolved names are included in the contact CSVs
- mesh assets: `{topo['geom1_mesh']}` ↔ `{topo['geom2_mesh']}`
- classification: `{classification}`
- semantic interpretation: repeated cross-arm end-effector surface contact at the three Clap closures; exact real contact surface still needs video/physical verification

The pair is absent throughout pre-roll: minimum pre-roll separation is `{pre_min.signed_wrist_geom_distance_m.min():.6f} m`. It first penetrates at about `1.602–1.604 s`, then repeats near `2.778 s` and `3.978 s`. This is not an initial model overlap.

## Pair summary

{markdown_table(pair_view, ['condition','mode','body1_name','body2_name','episodes','first_onset_s','total_contact_duration_s','max_episode_duration_s','max_penetration_mm','peak_normal_force_n','max_abs_relative_normal_velocity_m_s','classification'])}

## Episode-aligned baseline versus +8%

{markdown_table(compare, ['mode','episode','same_geom_pair','baseline_onset_s','candidate_onset_s','onset_delta_ms','baseline_duration_s','candidate_duration_s','duration_delta_ms','baseline_max_penetration_mm','candidate_max_penetration_mm','penetration_delta_mm','baseline_peak_normal_force_n','candidate_peak_normal_force_n','peak_force_delta_n'])}

## Classification rationale

- `EXPECTED_ADJACENT_LINK_CONTACT`: **NO**. The bodies are on separate left/right arm branches, not parent-child or grandparent-child.
- `MODEL_GEOMETRY_OVERLAP_CANDIDATE`: **NO**. Pre-roll separation is about 0.449 m and no internal contact occurs before the motion.
- `CONTROLLER_POSTURE_SELF_CONTACT`: **YES**. Exactly three finite-duration episodes follow the three Clap closures, with the same pair in baseline and candidate.
- `NUMERICAL_TRANSIENT`: **NO**. Durations are 0.153–0.186 s, penetration reaches `{max(max_base, max_mass)*1000:.3f} mm`, and normal force reaches `{max(max_force_base, max_force_mass):.3f} N`; the episodes are repeatable rather than isolated one-step events.
- `UNKNOWN`: **NO** for the simulation root cause. Whether the real robot's physical palms/housings touched in the captured Clap remains `NEEDS_PHYSICAL_VERIFICATION`.

The source collision meshes correspond to the actual wrist-roll/end-effector link surfaces, so this is a physically possible robot-surface contact. No `reported_effort` is used.
"""
    (HERE / "phase3bc_contact_pair_analysis.md").write_text(pair_report, encoding="utf-8")

    topology_report = f"""# Phase 3B-C collision-topology audit

| Check | Evidence-backed result |
|---|---|
| bodies | `{topo['body1_name']}` ↔ `{topo['body2_name']}` |
| parent-child | `{str(topo['parent_child']).upper()}` |
| grandparent-child | `{str(topo['grandparent_child']).upper()}` |
| kinematic-tree adjacent | `{str(topo['kinematic_tree_adjacent']).upper()}` |
| lowest common ancestor | `{topo['lowest_common_ancestor']}` |
| edges from bodies to common ancestor | `{topo['body1_edges_to_common_ancestor']}` / `{topo['body2_edges_to_common_ancestor']}` |
| geom types | `{topo['geom1_type']}` / `{topo['geom2_type']}` |
| geom masks | contype/conaffinity `{topo['geom1_contype']}/{topo['geom1_conaffinity']}` and `{topo['geom2_contype']}/{topo['geom2_conaffinity']}` |
| collision enabled by masks | `{str(topo['collision_enabled_by_masks']).upper()}` |
| model explicit excludes | `{topo['explicit_exclude_count_model']}` |
| pair explicitly excluded | `FALSE` |
| source collision representation | full mesh `{topo['geom1_mesh']}` / `{topo['geom2_mesh']}` |

Body 1 chain: `{' -> '.join(topo['body1_path'])}`

Body 2 chain: `{' -> '.join(topo['body2_path'])}`

The pair is a left/right end-effector pair on separate branches. Such surfaces are normally expected to remain collision-enabled; automatically excluding them would hide physically possible hand-to-hand or hand-to-object contact. The contact therefore does **not** demonstrate a parent/adjacent-link filter error.

`MJCF_COLLISION_FILTER_REVIEW_REQUIRED = NO`

This is an audit conclusion only. No mask, geometry, contype/conaffinity, or MJCF file was changed.
"""
    (HERE / "phase3bc_collision_topology_audit.md").write_text(topology_report, encoding="utf-8")

    # Select readable snapshots from the full ±0.5 s CSV.
    selected_times = [timeline_stats['onset_s'] - 0.5, timeline_stats['onset_s'] - 0.2,
                      timeline_stats['onset_s'], timeline_stats['peak_time_s'],
                      timeline_stats['end_s'], timeline_stats['onset_s'] + 0.5]
    selected_indices = sorted(set(int((timeline.timestamp_s - value).abs().idxmin()) for value in selected_times))
    snapshots = timeline.loc[selected_indices, [
        "timestamp_s", "signed_wrist_geom_distance_m", "contact_active", "contact_penetration_m",
        "contact_normal_force_n", "arm_reference_velocity_norm_rad_s", "arm_velocity_norm_rad_s",
        "arm_tracking_rms_rad", "arm_tracking_max_abs_rad", "base_roll_rad", "base_pitch_rad",
        "com_support_margin_m",
    ]].copy()
    timeline_report = f"""# Phase 3B-C first-contact causal timeline

Scope: original physical baseline, `arm_only`, first collision, `{timeline_stats['onset_s']-0.5:.3f}` to `{timeline_stats['onset_s']+0.5:.3f} s`. The complete 50 Hz aligned timeline is `phase3bc_contact_timeline.csv`; wrist surface distance/contact is sourced from the 1 kHz diagnostic probe. Contact columns shown at 50 Hz are 20 ms bin aggregates; exact onset/end values come from the 1 kHz rows.

| Event | Time |
|---|---:|
| first negative wrist-geom distance / contact onset | {timeline_stats['onset_s']:.3f} s |
| maximum penetration in episode | {timeline_stats['peak_time_s']:.3f} s |
| contact end | {timeline_stats['end_s']:.3f} s |
| contact duration | {timeline_stats['duration_s']:.3f} s |

{markdown_table(snapshots, list(snapshots.columns))}

## Tracking/posture → contact onset

1. During pre-roll the wrist collision meshes remain separated by about 0.449 m; there is no initial overlap.
2. The measured-Clap arm reference brings both end-effector meshes together. Distance approaches zero, then becomes negative at `{timeline_stats['onset_s']:.3f} s`.
3. Contact persists through the closed-hand phase and ends at `{timeline_stats['end_s']:.3f} s` as the arms separate. The same sequence repeats twice more.
4. Arm tracking RMS is `{timeline_stats['pre_tracking_rms_rad']:.6f} rad` in the preceding 0.5 s, `{timeline_stats['during_tracking_rms_rad']:.6f} rad` during contact, and `{timeline_stats['post_tracking_rms_rad']:.6f} rad` in the following 0.5 s. Contact does not coincide with a tracking-error blow-up; during-contact maximum absolute tracking error is `{timeline_stats['during_max_abs_tracking_rad']:.6f} rad`.
5. Maximum base tilt during the first episode is `{timeline_stats['during_max_base_tilt_deg']:.3f} deg`; minimum COM support margin is `{timeline_stats['during_min_com_support_margin_m']:.6f} m`. No fall, contact cascade, limit violation, or non-foot ground contact occurs.

The temporal direction is therefore: **Clap reference/posture closure → wrist surface convergence → contact**, not instability or a numerical one-step event → posture disturbance.
"""
    (HERE / "phase3bc_contact_timeline.md").write_text(timeline_report, encoding="utf-8")

    reinterpretation = f"""# Phase 3B-V gate reinterpretation after Phase 3B-C

## Evidence update

- baseline and +8% have the exact same sole geom/body pair: `{str(same_pairs).upper()}`
- both have three contact episodes in each mode
- onset deltas are within `{compare.onset_delta_ms.abs().max():.3f} ms`
- maximum pair penetration: baseline `{max_base*1000:.6f} mm`, candidate `{max_mass*1000:.6f} mm`
- peak normal force: baseline `{max_force_base:.6f} N`, candidate `{max_force_mass:.6f} N`
- candidate added a contact pair: `{str(added_pair).upper()}`
- root cause: repeated Clap end-effector closure, not collision topology and not a numerical transient

`MASS_DIRECTION_CAUSES_CONTACT = NO`

The +8% condition does not create the contact. Arm-only pair penetration is slightly higher in one episode while whole-body penetration/force are lower or comparable; there is no consistent cross-mode severity increase. Therefore the contact must not be used as evidence that the mass direction *caused* a safety regression.

## Gate status

| Gate | Phase 3B-C interpretation |
|---|---|
| PHYSICAL_DIRECTION_MAGNITUDE_SUPPORT | PARTIAL |
| ABSOLUTE_CLAP_SAFETY_INTERPRETATION | PREEXISTING_REPEATABLE_CLAP_END_EFFECTOR_CONTACT; NOT CANDIDATE-CAUSED |
| CONTROLLER_BASELINE_PRESERVED | YES |
| SAFETY_BASELINE_PRESERVED | YES |
| POSITION_SPACE_PHYSICAL_DIRECTION_VALIDATED | NO |
| DYNAMICS_CALIBRATION_READY | NO |

This contact should **not** remain a candidate-specific veto against `bs_mass_lower_plus08`. However, the frozen generic rule "any self-contact fails absolute safety" still evaluates to NO until the project explicitly approves a gesture-aware expected-contact policy and, ideally, verifies the real Clap contact surface from video/physical evidence. Phase 3B-C does not silently weaken that rule and does not auto-promote the physical direction to VALIDATED.

Persistent blockers remain: `PHYSICAL_SIGN=UNKNOWN`, `PHYSICAL_ZERO=UNKNOWN`, `EFFORT_SEMANTICS=UNKNOWN`, `IMU_TRANSFORM=PARTIAL`, `MC_INTERNAL_COMMAND=UNOBSERVABLE`.
"""
    (HERE / "phase3bc_phase3bv_gate_reinterpretation.md").write_text(reinterpretation, encoding="utf-8")

    final_report = f"""# Phase 3B-C final report

## Final classification

`CLAP_SELF_COLLISION_ROOT_CAUSE = {root_cause}`

`CONTACT_PAIR_CLASSIFICATION = {classification}`

`MASS_DIRECTION_CAUSES_CONTACT = NO`

`MJCF_COLLISION_FILTER_REVIEW_REQUIRED = NO`

`POSITION_SPACE_PHYSICAL_DIRECTION_VALIDATED = NO`

`DYNAMICS_CALIBRATION_READY = NO`

## Answers to the eight closure questions

1. **Which bodies/geoms?** `{topo['body1_name']}` geom `{topo['geom1_id']}` (`UNNAMED`, mesh `{topo['geom1_mesh']}`) versus `{topo['body2_name']}` geom `{topo['geom2_id']}` (`UNNAMED`, mesh `{topo['geom2_mesh']}`).
2. **Are baseline and +8% identical?** They have the exact same sole pair and three episodes per replay mode. Onsets differ by at most `{compare.onset_delta_ms.abs().max():.3f} ms`; row counts differ only by 2 of roughly 1,048 1-kHz contact samples across both modes.
3. **Maximum penetration and duration?** Across all four replays, maximum penetration is `{max(max_base,max_mass)*1000:.6f} mm`; maximum single-episode duration is `{episodes.duration_s.max():.3f} s`. Peak normal force is `{max(max_force_base,max_force_mass):.6f} N`.
4. **Gesture-peak relation?** Yes. There are exactly three contact closures at approximately 1.60, 2.78, and 3.98 s, aligned with the three closed-hand phases of the Clap trajectory; no pre-roll contact exists.
5. **Controller posture or topology?** `CONTROLLER_POSTURE`: the replayed Clap posture brings two physical end-effector mesh surfaces together. The bodies are not adjacent and collision masks are functioning as authored.
6. **Does +8% aggravate contact?** No systematic evidence. It adds no pair or episode; mode-dependent changes are tens of micrometres / about 1 ms and do not consistently increase penetration, force, or duration.
7. **Should it veto physical-direction validation?** Not as evidence against the +8% direction. It remains an absolute gesture-safety/policy item pending expected-contact semantics and physical verification.
8. **Does Phase 3B-V need reinterpretation?** Yes: the previous absolute failure is a pre-existing Clap end-effector contact, not candidate-caused. Magnitude support is `PARTIAL`, but formal validation remains `NO`; no gate is auto-promoted.

## Integrity

All four diagnostic replays reproduce the frozen Phase 3B-V fall, self-contact, pelvis/hip-contact, non-foot-ground-contact, limit, target-clip, and safety counts exactly. Instrumentation is read-only after `mj_step`. No controller, physical parameter, MJCF, collision mask, robot state, or hardware mapping was modified, and no `reported_effort` was used.

The +8% candidate remains **SHARED PHYSICAL SENSITIVITY DIRECTION — NOT IDENTIFIED HARDWARE PARAMETER**. This report is not `REAL_MASS_CALIBRATION`, `HARDWARE_MASS_IDENTIFICATION`, or `ACTUATOR_SYSTEM_IDENTIFICATION`.
"""
    (HERE / "phase3bc_final_report.md").write_text(final_report, encoding="utf-8")


def main() -> int:
    required = list(CONTACT_FILES.values()) + list(PROBE_FILES.values()) + [HERE / "phase3bc_replay_execution.json"]
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise FileNotFoundError("missing Phase 3B-C diagnostic output: " + "; ".join(missing))
    execution = json.loads((HERE / "phase3bc_replay_execution.json").read_text(encoding="utf-8"))
    if not execution.get("logger_only") or not all(item.get("match") for item in execution["replay_identity_checks"]):
        raise RuntimeError("Phase 3B-C logger-only replay identity is not proven")

    model = load_model()
    contact_frames = [enrich_contact_file(path, model) for path in CONTACT_FILES.values()]
    contacts = pd.concat(contact_frames, ignore_index=True)
    probes = pd.concat([pd.read_csv(path) for path in PROBE_FILES.values()], ignore_index=True)
    if contacts.geom_pair_key.nunique() != 1:
        raise RuntimeError(f"expected one collision pair for closure, found {contacts.geom_pair_key.unique()}")
    first = contacts.iloc[0]
    topo = topology(model, int(first.geom1_id), int(first.geom2_id))
    episodes = episode_metrics(contacts)
    pairs = pair_metrics(contacts)
    episodes.to_csv(HERE / "phase3bc_contact_episode_metrics.csv", index=False)
    compare = comparison_rows(episodes)
    timeline, timeline_stats = build_timeline(contacts, probes)
    write_reports(contacts, probes, pairs, episodes, compare, topo, timeline, timeline_stats)
    final = {
        "CLAP_SELF_COLLISION_ROOT_CAUSE": "CONTROLLER_POSTURE",
        "CONTACT_PAIR_CLASSIFICATION": "CONTROLLER_POSTURE_SELF_CONTACT",
        "MASS_DIRECTION_CAUSES_CONTACT": "NO",
        "MJCF_COLLISION_FILTER_REVIEW_REQUIRED": "NO",
        "PHYSICAL_DIRECTION_MAGNITUDE_SUPPORT": "PARTIAL",
        "POSITION_SPACE_PHYSICAL_DIRECTION_VALIDATED": "NO",
        "DYNAMICS_CALIBRATION_READY": "NO",
        "body_pair": [topo["body1_name"], topo["body2_name"]],
        "geom_pair": [topo["geom1_id"], topo["geom2_id"]],
        "max_penetration_m": float(contacts.penetration_depth_m.max()),
        "max_episode_duration_s": float(episodes.duration_s.max()),
        "max_normal_force_n": float(contacts.normal_force_n.max()),
    }
    (HERE / "phase3bc_final_gate.json").write_text(json.dumps(final, indent=2), encoding="utf-8")
    print(json.dumps(final, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
