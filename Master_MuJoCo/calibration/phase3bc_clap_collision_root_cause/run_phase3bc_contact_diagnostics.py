#!/usr/bin/env python3
"""Logger-only Phase 3B-C contact diagnostics for the frozen Clap replays.

This script deliberately reuses the exact Phase 3B-V dataset, controller,
runtime physical experiments, integrator, control rate, and replay modes.  The
only runtime substitution wraps ``mujoco.mj_step``: it first calls the original
function and then reads the resulting contact state.  It never writes an MJCF,
connects to a robot, loads reported effort, or changes a controller command.
"""

from __future__ import annotations

from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
from typing import Any

import mujoco
import numpy as np
import pandas as pd


HERE = Path(__file__).resolve().parent
CALIBRATION = HERE.parent
PROJECT = CALIBRATION.parent
WORKSPACE = PROJECT.parent
BV_DIR = CALIBRATION / "phase3bv_physical_direction_validation"
if str(BV_DIR) not in sys.path:
    sys.path.insert(0, str(BV_DIR))
import run_phase3bv_replays as BV  # noqa: E402


P3AR = BV.P3AR
AX = BV.AX
Y = BV.Y
RUNS = HERE / "diagnostic_runs"
DT = 0.001


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_json_hash(payload: Any) -> str:
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def relative(path: Path) -> str:
    return str(path.resolve().relative_to(WORKSPACE.resolve()))


def write_source_lock() -> None:
    """Freeze every replay input before diagnostics are executed."""
    key_paths = [
        BV_DIR / "capture" / "phase3bv_clap_001" / "raw_serialized_evidence.txt",
        BV_DIR / "phase3bv_measured_reference.csv",
        BV_DIR / "phase3bv_aligned_joint_data.csv",
        BV_DIR / "phase3bv_capture_metadata.json",
        BV_DIR / "phase3bv_independence.json",
        PROJECT / "assets" / "Master" / "ff_master_ultra.xml",
        PROJECT / "assets" / "Master" / "scene_x2_free.xml",
        CALIBRATION / "phase3ar_controller_redesign" / "phase3ar_core.py",
        CALIBRATION / "phase3ax_constraint_balance" / "phase3ax_core.py",
        CALIBRATION / "phase3ay_motion_conditioned_balance" / "phase3ay_core.py",
        CALIBRATION / "phase3ay_motion_conditioned_balance" / "simulation_motion_conditioned_balance_candidate.json",
        CALIBRATION / "phase3bs_physical_sensitivity" / "phase3bs_core.py",
        BV_DIR / "run_phase3bv_replays.py",
        BV_DIR / "phase3bv_source_manifest.csv",
        Path(__file__).resolve(),
    ]
    missing = [str(path) for path in key_paths if not path.exists()]
    if missing:
        raise FileNotFoundError("Phase 3B-C source-lock input missing: " + "; ".join(missing))
    rows = [{"path": relative(path), "bytes": path.stat().st_size, "sha256": sha256(path)} for path in key_paths]
    pd.DataFrame(rows).to_csv(HERE / "phase3bc_source_manifest.csv", index=False)

    baseline_design = asdict(BV.ay_candidate(BV.BASELINE.experiment_id))
    mass_design = asdict(BV.ay_candidate(BV.MASS_DIRECTION.experiment_id))
    baseline_design_no_id = dict(baseline_design)
    mass_design_no_id = dict(mass_design)
    baseline_design_no_id.pop("experiment_id", None)
    mass_design_no_id.pop("experiment_id", None)
    if baseline_design_no_id != mass_design_no_id:
        raise RuntimeError("Frozen baseline and +8% replays do not have identical controller configuration")

    lock = {
        "capture": relative(key_paths[0]),
        "processed_replay_input": relative(key_paths[1]),
        "source_mjcf": relative(key_paths[5]),
        "baseline_physical_experiment": asdict(BV.BASELINE),
        "mass_direction_physical_experiment": asdict(BV.MASS_DIRECTION),
        "controller_config_without_experiment_id": baseline_design_no_id,
        "controller_config_sha256": canonical_json_hash(baseline_design_no_id),
        "baseline_runtime_spec_sha256": canonical_json_hash(asdict(BV.BASELINE)),
        "mass_direction_runtime_spec_sha256": canonical_json_hash(asdict(BV.MASS_DIRECTION)),
        "replay_modes": ["arm_only", "whole_body"],
        "pre_roll_s": 5.0,
        "post_roll_s": 5.0,
        "simulation_timestep_s": DT,
        "instrumentation": "POST_MJ_STEP_READ_ONLY_CONTACT_LOGGER",
        "robot_connected": False,
        "reported_effort_loaded": False,
        "controller_modified": False,
        "source_mjcf_modified": False,
        "collision_topology_modified": False,
    }
    (HERE / "phase3bc_runtime_lock.json").write_text(json.dumps(lock, indent=2), encoding="utf-8")

    manifest = pd.DataFrame(rows)
    table = "\n".join(
        f"| `{row.path}` | {row.bytes} | `{row.sha256}` |" for row in manifest.itertuples(index=False)
    )
    text = f"""# Phase 3B-C source lock

All inputs below were hashed **before** the diagnostic replay. Phase 3B-V's full source manifest was also verified before execution.

| Item | Locked value |
|---|---|
| Real motion | `CLAP` |
| Capture | `{lock['capture']}` |
| Processed replay input | `{lock['processed_replay_input']}` |
| Original physical condition | `phase3bv_original_physical_baseline` |
| Candidate physical condition | `bs_mass_lower_plus08` / total-mass-preserving runtime lower-limb scale `1.08` |
| Candidate meaning | `SHARED_PHYSICAL_SENSITIVITY_DIRECTION — NOT IDENTIFIED HARDWARE PARAMETER` |
| Controller | frozen Phase 3A-Y; configuration hash `{lock['controller_config_sha256']}` |
| Controller equality | `IDENTICAL` between physical conditions (experiment ID excluded) |
| MJCF | `{lock['source_mjcf']}` |
| Replay modes | `arm_only`, `whole_body` |
| Timestep / control update | `0.001 s / 0.001 s` |
| Instrumentation | post-`mj_step` read-only contact sampling |

The logger does not change simulation state. It creates no robot connection, publisher, client, or motion command. It does not load `reported_effort`; tune controller, mass, inertia, friction, damping, armature, gear, or limits; or edit MJCF/collision masks.

## Runtime-spec hashes

- baseline: `{lock['baseline_runtime_spec_sha256']}`
- lower-limb +8% candidate: `{lock['mass_direction_runtime_spec_sha256']}`

## File manifest

| path | bytes | SHA-256 |
|---|---:|---|
{table}

`DYNAMICS_CALIBRATION_READY = NO`
"""
    (HERE / "phase3bc_source_lock.md").write_text(text, encoding="utf-8")


def object_name(model: mujoco.MjModel, kind: mujoco.mjtObj, index: int) -> str:
    return mujoco.mj_id2name(model, kind, int(index)) or ""


class ContactLogger:
    """Read-only 1 kHz logger for every non-floor, inter-body contact point."""

    def __init__(self, condition: str, mode: str, t_start: float):
        self.condition = condition
        self.mode = mode
        self.t_start = t_start
        self.rows: list[dict[str, Any]] = []
        self.probe_rows: list[dict[str, Any]] = []
        self.previous_pair_distance: dict[str, tuple[float, float]] = {}
        self.model: mujoco.MjModel | None = None
        self.floor_geom_id = -1
        self.joints: list[tuple[str, int]] = []
        self.pelvis_body_id = -1
        self.probe_geom_ids: tuple[int, int] | None = None

    def bind(self, model: mujoco.MjModel) -> None:
        self.model = model
        self.floor_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self.pelvis_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        probe_ids = []
        for body_name in ("left_wrist_roll_link", "right_wrist_roll_link"):
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            candidates = [
                geom_id for geom_id in range(model.ngeom)
                if int(model.geom_bodyid[geom_id]) == body_id
                and int(model.geom_contype[geom_id]) != 0
                and int(model.geom_conaffinity[geom_id]) != 0
            ]
            if len(candidates) != 1:
                raise RuntimeError(f"expected one active collision geom on {body_name}, found {candidates}")
            probe_ids.append(candidates[0])
        self.probe_geom_ids = (probe_ids[0], probe_ids[1])
        self.joints = []
        for joint_id in range(model.njnt):
            name = object_name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            joint_type = int(model.jnt_type[joint_id])
            if name and joint_type in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
                self.joints.append((name, int(model.jnt_qposadr[joint_id])))

    @staticmethod
    def point_velocity(model: mujoco.MjModel, data: mujoco.MjData, body_id: int, point: np.ndarray) -> np.ndarray:
        spatial = np.zeros(6, dtype=np.float64)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, body_id, spatial, 0)
        angular = spatial[:3]
        linear_at_body = spatial[3:]
        return linear_at_body + np.cross(angular, point - np.asarray(data.xpos[body_id], dtype=float))

    def record(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        if self.model is None:
            self.bind(model)
        timestamp = self.t_start + float(data.time)
        assert self.probe_geom_ids is not None
        fromto = np.zeros(6, dtype=np.float64)
        wrist_distance = float(mujoco.mj_geomDistance(
            model, data, self.probe_geom_ids[0], self.probe_geom_ids[1], 0.50, fromto
        ))
        self.probe_rows.append({
            "condition": self.condition,
            "mode": self.mode,
            "timestamp_s": timestamp,
            "sim_time_s": float(data.time),
            "geom1_id": self.probe_geom_ids[0],
            "geom2_id": self.probe_geom_ids[1],
            "signed_wrist_geom_distance_m": wrist_distance,
            "nearest_point_geom1_x_m": float(fromto[0]),
            "nearest_point_geom1_y_m": float(fromto[1]),
            "nearest_point_geom1_z_m": float(fromto[2]),
            "nearest_point_geom2_x_m": float(fromto[3]),
            "nearest_point_geom2_y_m": float(fromto[4]),
            "nearest_point_geom2_z_m": float(fromto[5]),
        })
        eligible: list[tuple[int, Any, int, int, int, int, str]] = []
        pair_minimum: dict[str, float] = {}
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            body1, body2 = int(model.geom_bodyid[geom1]), int(model.geom_bodyid[geom2])
            if geom1 == self.floor_geom_id or geom2 == self.floor_geom_id or body1 == body2:
                continue
            low, high = sorted((geom1, geom2))
            pair_key = f"{low}:{high}"
            eligible.append((contact_index, contact, geom1, geom2, body1, body2, pair_key))
            pair_minimum[pair_key] = min(pair_minimum.get(pair_key, float("inf")), float(contact.dist))
        if not eligible:
            self.previous_pair_distance = {}
            return

        joint_configuration = json.dumps(
            {name: float(data.qpos[qpos_adr]) for name, qpos_adr in self.joints},
            separators=(",", ":"),
            sort_keys=True,
        )
        base_position = [float(value) for value in data.xpos[self.pelvis_body_id]] if self.pelvis_body_id >= 0 else [None] * 3
        if self.pelvis_body_id >= 0:
            base_roll, base_pitch, base_yaw = P3AR.P3A.rpy(data.xmat[self.pelvis_body_id])
        else:
            base_roll = base_pitch = base_yaw = float("nan")
        active_pairs = set(pair_minimum)
        rates: dict[str, float] = {}
        for pair_key, distance in pair_minimum.items():
            previous = self.previous_pair_distance.get(pair_key)
            rates[pair_key] = float("nan") if previous is None else (distance - previous[1]) / max(timestamp - previous[0], 1e-12)
        self.previous_pair_distance = {
            key: (timestamp, pair_minimum[key]) for key in active_pairs
        }

        pair_point_index: dict[str, int] = {}
        for contact_index, contact, geom1, geom2, body1, body2, pair_key in eligible:
            point_index = pair_point_index.get(pair_key, 0)
            pair_point_index[pair_key] = point_index + 1
            force = np.zeros(6, dtype=np.float64)
            mujoco.mj_contactForce(model, data, contact_index, force)
            position = np.asarray(contact.pos, dtype=float)
            frame = np.asarray(contact.frame, dtype=float).reshape(3, 3)
            normal = frame[0]
            v1 = self.point_velocity(model, data, body1, position)
            v2 = self.point_velocity(model, data, body2, position)
            relative_normal_velocity = float(np.dot(v2 - v1, normal))
            geom1_name = object_name(model, mujoco.mjtObj.mjOBJ_GEOM, geom1)
            geom2_name = object_name(model, mujoco.mjtObj.mjOBJ_GEOM, geom2)
            body1_name = object_name(model, mujoco.mjtObj.mjOBJ_BODY, body1)
            body2_name = object_name(model, mujoco.mjtObj.mjOBJ_BODY, body2)
            canonical_names = sorted((geom1_name or f"geom_{geom1}", geom2_name or f"geom_{geom2}"))
            self.rows.append({
                "condition": self.condition,
                "mode": self.mode,
                "timestamp_s": timestamp,
                "sim_time_s": float(data.time),
                "contact_index": contact_index,
                "pair_contact_point_index": point_index,
                "geom_pair_key": pair_key,
                "geom_pair_name": " <-> ".join(canonical_names),
                "geom1_id": geom1,
                "geom1_name": geom1_name,
                "geom2_id": geom2,
                "geom2_name": geom2_name,
                "body1_id": body1,
                "body1_name": body1_name,
                "body2_id": body2,
                "body2_name": body2_name,
                "contact_x_m": float(position[0]),
                "contact_y_m": float(position[1]),
                "contact_z_m": float(position[2]),
                "normal_x": float(normal[0]),
                "normal_y": float(normal[1]),
                "normal_z": float(normal[2]),
                "contact_distance_m": float(contact.dist),
                "penetration_depth_m": max(0.0, -float(contact.dist)),
                "normal_force_n": float(force[0]),
                "tangent_force_1_n": float(force[1]),
                "tangent_force_2_n": float(force[2]),
                "relative_normal_velocity_m_s": relative_normal_velocity,
                "pair_min_distance_rate_m_s": rates[pair_key],
                "base_x_m": base_position[0],
                "base_y_m": base_position[1],
                "base_z_m": base_position[2],
                "base_roll_rad": base_roll,
                "base_pitch_rad": base_pitch,
                "base_yaw_rad": base_yaw,
                "joint_configuration_json": joint_configuration,
            })

    def frame_with_episodes(self) -> pd.DataFrame:
        frame = pd.DataFrame(self.rows)
        if frame.empty:
            return frame
        frame = frame.sort_values(["mode", "geom_pair_key", "timestamp_s", "pair_contact_point_index"]).reset_index(drop=True)
        episode_labels = pd.Series(index=frame.index, dtype="object")
        for (mode, pair), group in frame.groupby(["mode", "geom_pair_key"], sort=False):
            unique_times = np.sort(group.timestamp_s.unique())
            episode_by_time: dict[float, int] = {}
            episode = 0
            previous: float | None = None
            for time_value in unique_times:
                if previous is None or time_value - previous > 1.5 * DT:
                    episode += 1
                episode_by_time[float(time_value)] = episode
                previous = float(time_value)
            for index in group.index:
                local = episode_by_time[float(frame.at[index, "timestamp_s"])]
                episode_labels.at[index] = f"{self.condition}__{mode}__{pair}__episode_{local:03d}"
        frame["episode_id"] = episode_labels
        episode_stats = frame.groupby("episode_id").agg(
            episode_onset_s=("timestamp_s", "min"),
            episode_end_s=("timestamp_s", "max"),
            episode_sample_count=("timestamp_s", "nunique"),
            episode_contact_row_count=("timestamp_s", "size"),
        )
        episode_stats["contact_duration_s"] = episode_stats.episode_end_s - episode_stats.episode_onset_s + DT
        return frame.join(episode_stats, on="episode_id").sort_values(["mode", "timestamp_s", "contact_index"]).reset_index(drop=True)

    def probe_frame(self) -> pd.DataFrame:
        return pd.DataFrame(self.probe_rows)


def replay_time_start(dataset) -> float:
    reference_frame, _ = P3AR.load_frames(dataset)
    return max(float(reference_frame.t.min()), -5.0)


def run_one(experiment, dataset, mode: str) -> tuple[dict[str, Any], pd.DataFrame, pd.DataFrame]:
    condition = "original" if experiment.family == "BASELINE" else "mass_direction"
    logger = ContactLogger(condition, mode, replay_time_start(dataset))
    design = BV.ay_candidate(experiment.experiment_id)
    original_load = P3AR.load_model
    original_runs = Y.RUNS
    original_step = AX.mujoco.mj_step
    override_audit: dict[str, Any] = {}

    def loader(*, free_base: bool):
        model = original_load(free_base=free_base)
        override_audit.update(BV.apply_runtime_override(model, experiment))
        logger.bind(model)
        return model

    def instrumented_step(model: mujoco.MjModel, data: mujoco.MjData) -> None:
        original_step(model, data)
        logger.record(model, data)

    P3AR.load_model = loader
    Y.RUNS = RUNS
    AX.mujoco.mj_step = instrumented_step
    try:
        summary = Y.run_replay(design, dataset, mode, pre_s=5.0, post_s=5.0, save_detail=True)
    finally:
        AX.mujoco.mj_step = original_step
        P3AR.load_model = original_load
        Y.RUNS = original_runs
    summary.update({
        "phase3bc_instrumentation": "POST_MJ_STEP_READ_ONLY_CONTACT_LOGGER",
        "contact_logging_rate_hz": 1000.0,
        "logged_contact_scope": "ALL_NON_FLOOR_INTER_BODY_CONTACT_POINTS",
        "physical_experiment": asdict(experiment),
        "runtime_override_audit": override_audit,
        "source_mjcf_modified": False,
        "controller_modified": False,
        "collision_topology_modified": False,
        "reported_effort_loaded": False,
        "robot_connected": False,
    })
    stem = f"{experiment.experiment_id}__{dataset.name}__{mode}"
    (RUNS / f"{stem}_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary, logger.frame_with_episodes(), logger.probe_frame()


def verify_replay_identity(summaries: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Confirm instrumentation did not change the frozen replay safety summaries."""
    checks = []
    for summary in summaries:
        stem = f"{summary['experiment_id']}__{summary['dataset']}__{summary['mode']}_summary.json"
        frozen_path = BV.RUNS / stem
        if not frozen_path.exists():
            raise FileNotFoundError(f"missing frozen Phase 3B-V replay summary: {frozen_path}")
        frozen = json.loads(frozen_path.read_text(encoding="utf-8"))
        fields = (
            "stable_no_fall", "self_collision_samples", "pelvis_hip_contact_samples",
            "other_self_collision_samples", "nonfoot_ground_contact_samples",
            "limit_violation_samples", "target_clip_samples",
        )
        differences = {field: [frozen.get(field), summary.get(field)] for field in fields if frozen.get(field) != summary.get(field)}
        if differences:
            raise RuntimeError(f"logger-only replay identity failed for {stem}: {differences}")
        checks.append({"experiment_id": summary["experiment_id"], "mode": summary["mode"], "fields": list(fields), "match": True})
    return checks


def main() -> int:
    BV.verify_manifest()
    HERE.mkdir(parents=True, exist_ok=True)
    RUNS.mkdir(parents=True, exist_ok=True)
    write_source_lock()
    dataset = BV.dataset_from_capture()
    summaries: list[dict[str, Any]] = []
    frames: dict[str, list[pd.DataFrame]] = {"original": [], "mass_direction": []}
    probes: dict[str, list[pd.DataFrame]] = {"original": [], "mass_direction": []}
    for experiment in (BV.BASELINE, BV.MASS_DIRECTION):
        condition = "original" if experiment.family == "BASELINE" else "mass_direction"
        for mode in ("arm_only", "whole_body"):
            print(f"PHASE3BC LOGGER-ONLY REPLAY {condition} {mode}", flush=True)
            summary, frame, probe = run_one(experiment, dataset, mode)
            summaries.append(summary)
            frames[condition].append(frame)
            probes[condition].append(probe)
    identity = verify_replay_identity(summaries)
    output_names = {
        "original": "phase3bc_clap_baseline_contacts.csv",
        "mass_direction": "phase3bc_clap_mass_contacts.csv",
    }
    probe_names = {
        "original": "phase3bc_clap_baseline_wrist_distance.csv",
        "mass_direction": "phase3bc_clap_mass_wrist_distance.csv",
    }
    contact_counts: dict[str, int] = {}
    for condition, pieces in frames.items():
        nonempty = [piece for piece in pieces if not piece.empty]
        merged = pd.concat(nonempty, ignore_index=True) if nonempty else pd.DataFrame()
        merged.to_csv(HERE / output_names[condition], index=False)
        contact_counts[condition] = len(merged)
        pd.concat(probes[condition], ignore_index=True).to_csv(HERE / probe_names[condition], index=False)
    execution = {
        "phase": "Phase 3B-C",
        "replay_identity_checks": identity,
        "contact_rows": contact_counts,
        "source_manifest_verified_before_run": True,
        "logger_only": True,
        "controller_modified": False,
        "source_mjcf_modified": False,
        "collision_topology_modified": False,
        "robot_connected": False,
        "reported_effort_loaded": False,
        "summaries": [{
            "experiment_id": item["experiment_id"],
            "mode": item["mode"],
            "self_collision_samples": item["self_collision_samples"],
            "other_self_collision_samples": item["other_self_collision_samples"],
            "stable_no_fall": item["stable_no_fall"],
        } for item in summaries],
    }
    (HERE / "phase3bc_replay_execution.json").write_text(json.dumps(execution, indent=2), encoding="utf-8")
    print(json.dumps(execution, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
