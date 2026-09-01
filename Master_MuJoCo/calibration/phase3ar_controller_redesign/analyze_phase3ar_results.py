#!/usr/bin/env python3
"""Generate Phase 3A-R evidence reports, plots, and final gates."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import mujoco
import numpy as np
import pandas as pd

from phase3ar_core import (
    HERE,
    NUMERICAL_CONTACT_TOLERANCE_M,
    datasets,
    load_model,
)


RUNS = HERE / "runs"
PLOTS = HERE / "plots"
WORKSPACE = HERE.parents[2]
CURRENT = "current_07"
FINAL = "phase3ar_final_candidate"


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def summary(experiment: str, dataset: str, mode: str):
    return read_json(RUNS / f"{experiment}__{dataset}__{mode}_summary.json")


def frame(experiment: str, dataset: str, mode: str, kind: str) -> pd.DataFrame:
    return pd.read_csv(RUNS / f"{experiment}__{dataset}__{mode}_{kind}_log.csv")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify_sources() -> pd.DataFrame:
    manifest = pd.read_csv(HERE / "phase3ar_source_manifest.csv")
    rows = []
    for item in manifest.itertuples(index=False):
        path = WORKSPACE / Path(item.path)
        current = sha256(path) if path.exists() else "MISSING"
        rows.append(
            {
                "path": item.path,
                "locked_sha256": item.sha256,
                "current_sha256": current,
                "status": "VERIFIED_UNCHANGED" if current == item.sha256 else "CHANGED_OR_MISSING",
            }
        )
    result = pd.DataFrame(rows)
    result.to_csv(HERE / "phase3ar_source_verification.csv", index=False)
    return result


def first_row(data: pd.DataFrame, mask: pd.Series):
    selected = data[mask]
    return None if selected.empty else selected.iloc[0]


def event_timeline(experiment: str, dataset: str, mode: str) -> list[dict[str, object]]:
    base = frame(experiment, dataset, mode, "base")
    joints = frame(experiment, dataset, mode, "joint")
    contacts = frame(experiment, dataset, mode, "contact")
    events: list[tuple[str, object, str]] = []
    row = first_row(joints, joints.tracking_error_rad.abs() > 0.10)
    if row is not None:
        events.append(("FIRST_LARGE_TRACKING_ERROR", row, f"joint={row.joint_name}; error={row.tracking_error_rad:.6f} rad"))
    left = contacts[(contacts.side == "left") & (contacts.contact_active == 1)]
    if not left.empty:
        row = left.iloc[0]
        events.append(("FIRST_PELVIS_HIP_CONTACT", row, f"side=left; dist={row.contact_dist_m:.9f} m"))
    over = contacts[(contacts.contact_active == 1) & (contacts.contact_dist_m < -NUMERICAL_CONTACT_TOLERANCE_M)]
    if not over.empty:
        row = over.iloc[0]
        events.append(("FIRST_CONTACT_OVER_TOLERANCE", row, f"side={row.side}; dist={row.contact_dist_m:.9f} m"))
    row = first_row(joints, joints.limit_margin_rad <= 0.0)
    if row is not None:
        events.append(("FIRST_LIMIT_VIOLATION", row, f"joint={row.joint_name}; margin={row.limit_margin_rad:.6f} rad"))
    row = first_row(base, (base.base_roll_rad.abs() > math.radians(10)) | (base.base_pitch_rad.abs() > math.radians(10)))
    if row is not None:
        events.append(("FIRST_BALANCE_EXCURSION_GT_10_DEG", row, f"roll={row.base_roll_rad:.6f}; pitch={row.base_pitch_rad:.6f}"))
    row = first_row(joints, joints.ctrl_saturation_fraction >= 0.98)
    if row is not None:
        events.append(("FIRST_ACTUATOR_SATURATION", row, f"joint={row.joint_name}; fraction={row.ctrl_saturation_fraction:.6f}"))
    row = first_row(base, (base.base_z < 0.30) | (base.base_roll_rad.abs() > math.radians(45)) | (base.base_pitch_rad.abs() > math.radians(45)))
    if row is not None:
        events.append(("FALL_THRESHOLD", row, f"z={row.base_z:.6f}; roll={row.base_roll_rad:.6f}; pitch={row.base_pitch_rad:.6f}"))
    events.sort(key=lambda item: float(item[1].sim_time))
    return [
        {
            "scenario": f"{experiment}__{dataset}__{mode}",
            "event": event,
            "sim_time_s": float(row.sim_time),
            "reference_time_s": float(row.t),
            "details": details,
        }
        for event, row, details in events
    ]


def contact_scenario(experiment: str, dataset: str, mode: str) -> dict[str, object]:
    contacts = frame(experiment, dataset, mode, "contact")
    left = contacts[(contacts.side == "left") & (contacts.contact_active == 1)]
    over = left[left.contact_dist_m < -NUMERICAL_CONTACT_TOLERANCE_M]
    return {
        "scenario": f"{experiment}__{dataset}__{mode}",
        "first_contact_sim_s": None if left.empty else float(left.sim_time.min()),
        "first_over_tolerance_sim_s": None if over.empty else float(over.sim_time.min()),
        "last_contact_sim_s": None if left.empty else float(left.sim_time.max()),
        "contact_sample_count": int(len(left)),
        "over_tolerance_sample_count": int(len(over)),
        "maximum_penetration_m": 0.0 if left.empty else float(max(0.0, -left.contact_dist_m.min())),
    }


def onset_state() -> tuple[pd.Series, pd.DataFrame, pd.Series]:
    contacts = frame(CURRENT, "wave", "arm_only", "contact")
    left = contacts[(contacts.side == "left") & (contacts.contact_active == 1)]
    onset = left.iloc[0]
    joints = frame(CURRENT, "wave", "arm_only", "joint")
    at = joints[np.isclose(joints.sim_time, onset.sim_time)]
    base = frame(CURRENT, "wave", "arm_only", "base")
    base_at = base.iloc[int((base.sim_time - onset.sim_time).abs().argmin())]
    return onset, at, base_at


def consecutive_duration(mask: np.ndarray, dt: float = 0.02) -> float:
    best = current = 0
    for value in mask:
        current = current + 1 if value else 0
        best = max(best, current)
    return best * dt


def saturation_table(experiment: str) -> pd.DataFrame:
    joints = frame(experiment, "wave", "whole_body", "joint")
    rows = []
    for name, group in joints.groupby("joint_name"):
        group = group.sort_values("sim_time")
        saturated = group.ctrl_saturation_fraction.to_numpy(float) >= 0.98
        if not saturated.any():
            continue
        hit = group[saturated]
        rows.append(
            {
                "experiment": experiment,
                "joint_name": name,
                "saturation_start_sim_s": float(hit.sim_time.iloc[0]),
                "saturation_duration_samples_s": float(len(hit) / 50.0),
                "saturation_ratio": float(saturated.mean()),
                "max_consecutive_saturation_s": consecutive_duration(saturated),
                "max_abs_tracking_error_rad": float(hit.tracking_error_rad.abs().max()),
                "max_abs_balance_addition_nm": float(hit.balance_addition_nm.abs().max()),
            }
        )
    return pd.DataFrame(rows)


def limit_table(experiment: str) -> pd.DataFrame:
    model = load_model(free_base=True)
    limits = {}
    for joint_id in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if name:
            limits[name] = tuple(float(v) for v in model.jnt_range[joint_id])
    joints = frame(experiment, "wave", "whole_body", "joint")
    rows = []
    for name, group in joints[joints.limit_margin_rad <= 0.0].groupby("joint_name"):
        first = group.sort_values("sim_time").iloc[0]
        lower, upper = limits[name]
        rows.append(
            {
                "experiment": experiment,
                "joint_name": name,
                "first_violation_sim_s": float(first.sim_time),
                "requested_target_rad": float(first.target_position),
                "actual_q_rad": float(first.position),
                "lower_limit_rad": lower,
                "upper_limit_rad": upper,
                "reference_rad": float(first.reference_position),
                "reference_inside_limit": lower <= float(first.reference_position) <= upper,
                "minimum_margin_rad": float(group.limit_margin_rad.min()),
                "classification": "CONTROLLER_LIMIT_MANAGEMENT_FAILURE" if lower <= float(first.reference_position) <= upper else "REFERENCE_OUTSIDE_LIMIT",
            }
        )
    return pd.DataFrame(rows)


def tracking_map(summary_data: dict[str, object]) -> dict[str, dict[str, float | None]]:
    return {row["joint_name"]: row for row in summary_data["tracking_metrics"]}


def balance_map(summary_data: dict[str, object]) -> dict[str, dict[str, float | None]]:
    return {row["joint_name"]: row for row in summary_data["balance_metrics"]}


def table(headers: list[str], rows: list[list[object]]) -> str:
    lines = ["| " + " | ".join(headers) + " |", "| " + " | ".join("---" for _ in headers) + " |"]
    lines += ["| " + " | ".join(str(value) for value in row) + " |" for row in rows]
    return "\n".join(lines)


def plot_results() -> None:
    for subdir in ("contact", "saturation", "heart", "wave", "balance", "before_after"):
        (PLOTS / subdir).mkdir(parents=True, exist_ok=True)
    plt.style.use("seaborn-v0_8-whitegrid")

    current = frame(CURRENT, "wave", "arm_only", "contact")
    final = frame(FINAL, "wave", "arm_only", "contact")
    fig, ax = plt.subplots(figsize=(10, 4.5))
    for data, label in ((current, "Phase 3A current"), (final, "Phase 3A-R candidate")):
        left = data[data.side == "left"]
        ax.plot(left.t, 1000.0 * left.signed_geom_distance_m, label=label)
    ax.axhline(-1000.0 * NUMERICAL_CONTACT_TOLERANCE_M, color="red", linestyle="--", label="-0.5 mm tolerance")
    ax.set(xlabel="t relative to motion onset (s)", ylabel="signed pelvis/left-hip distance (mm)", title="Persistent pelvis/hip contact margin")
    ax.legend()
    fig.tight_layout()
    fig.savefig(PLOTS / "contact" / "pelvis_hip_distance_before_after.png", dpi=160)
    fig.savefig(PLOTS / "before_after" / "pelvis_hip_contact_margin.png", dpi=160)
    plt.close(fig)

    sat = pd.concat([saturation_table(CURRENT), saturation_table(FINAL)], ignore_index=True)
    if not sat.empty:
        pivot = sat.pivot(index="joint_name", columns="experiment", values="saturation_ratio").fillna(0.0)
        ax = pivot.plot.bar(figsize=(12, 5))
        ax.set(ylabel="saturated sample fraction", title="Wave whole-body saturation")
        ax.legend(title="experiment")
        plt.tight_layout()
        plt.savefig(PLOTS / "saturation" / "whole_body_saturation_by_joint.png", dpi=160)
        plt.close()

    for dataset in ("heart", "wave"):
        real = pd.read_csv(datasets()[dataset].real_joint_path, usecols=["t", "joint_name", "position"])
        current_j = frame(CURRENT, dataset, "arm_only", "joint")
        final_j = frame(FINAL, dataset, "arm_only", "joint")
        names = [name for name in ("left_shoulder_roll_joint", "right_shoulder_roll_joint", "left_wrist_yaw_joint", "right_wrist_yaw_joint") if name in set(final_j.joint_name)]
        fig, axes = plt.subplots(len(names), 1, figsize=(10, 2.6 * len(names)), sharex=True)
        axes = np.atleast_1d(axes)
        for ax, name in zip(axes, names):
            r = real[real.joint_name == name]
            c = current_j[current_j.joint_name == name]
            f = final_j[final_j.joint_name == name]
            ax.plot(r.t, r.position, color="black", label="real measured")
            ax.plot(c.t, c.position, linestyle="--", label="Phase 3A")
            ax.plot(f.t, f.position, label="Phase 3A-R")
            ax.set_ylabel(name.replace("_joint", "") + "\nrad")
        axes[-1].set_xlabel("t relative to motion onset (s)")
        axes[0].legend(ncol=3)
        fig.tight_layout()
        fig.savefig(PLOTS / dataset / "arm_tracking.png", dpi=160)
        plt.close(fig)

    labels = []
    current_values = []
    final_values = []
    for dataset in ("heart", "wave"):
        c = balance_map(summary(CURRENT, dataset, "arm_only"))
        f = balance_map(summary(FINAL, dataset, "arm_only"))
        for name in c:
            labels.append(dataset[0].upper() + ":" + name.replace("_joint", "").replace("left_", "L_").replace("right_", "R_"))
            current_values.append(float(c[name]["excursion_ratio"]))
            final_values.append(float(f[name]["excursion_ratio"]))
    x = np.arange(len(labels))
    fig, ax = plt.subplots(figsize=(15, 5))
    ax.bar(x - 0.2, current_values, 0.4, label="Phase 3A")
    ax.bar(x + 0.2, final_values, 0.4, label="Phase 3A-R")
    ax.axhline(1.0, color="black", linewidth=1)
    ax.set_xticks(x, labels, rotation=70, ha="right")
    ax.set(ylabel="sim/real excursion ratio", title="Heart + wave balance excursion response")
    ax.legend()
    fig.tight_layout()
    fig.savefig(PLOTS / "balance" / "heart_wave_excursion_ratios.png", dpi=160)
    plt.close(fig)


def main() -> int:
    verification = verify_sources()
    if not (verification.status == "VERIFIED_UNCHANGED").all():
        raise RuntimeError("Frozen inputs changed")

    scenarios = [
        (CURRENT, "heart", "standing"), (CURRENT, "wave", "standing"),
        (CURRENT, "heart", "arm_only"), (CURRENT, "wave", "arm_only"),
        (CURRENT, "wave", "whole_body"), ("legacy_phase3av", "wave", "arm_only"),
        (FINAL, "heart", "standing"), (FINAL, "wave", "standing"),
        (FINAL, "heart", "arm_only"), (FINAL, "wave", "arm_only"),
        (FINAL, "wave", "whole_body"),
    ]
    contact_rows = [contact_scenario(*scenario) for scenario in scenarios]
    timeline = []
    for scenario in ((CURRENT, "wave", "arm_only"), (CURRENT, "wave", "whole_body"), (FINAL, "wave", "whole_body")):
        timeline += event_timeline(*scenario)
    pd.DataFrame(contact_rows + timeline).to_csv(HERE / "phase3ar_contact_timeline.csv", index=False)

    onset, onset_joints, onset_base = onset_state()
    key_names = [
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_roll_joint", "waist_pitch_joint", "waist_roll_joint",
    ]
    onset_joint_rows = []
    for name in key_names:
        row = onset_joints[onset_joints.joint_name == name]
        if not row.empty:
            item = row.iloc[0]
            onset_joint_rows.append([f"`{name}`", f"{item.position:.6f}", f"{item.target_position:.6f}", f"{item.ctrl_nm:.3f}", f"{item.ctrl_saturation_fraction:.3f}"])

    static_geometry = pd.read_csv(RUNS / "static_pelvis_hip_geometry.csv")
    current_wave_arm = summary(CURRENT, "wave", "arm_only")
    final_wave_arm = summary(FINAL, "wave", "arm_only")
    current_wave_standing = summary(CURRENT, "wave", "standing")
    final_wave_standing = summary(FINAL, "wave", "standing")
    current_heart_arm = summary(CURRENT, "heart", "arm_only")

    (HERE / "phase3ar_source_lock.md").write_text(f"""# Phase 3A-R source lock

- status: `PHASE3AR_SOURCE_LOCKED`
- immutable input files: `{len(verification)}`
- verification: `{int((verification.status == 'VERIFIED_UNCHANGED').sum())}/{len(verification)} VERIFIED_UNCHANGED`
- Phase 2/3 inputs overwritten: `False`
- MJCF modified: `False`
- hardware mapping modified: `False`
- reported effort used: `False`
- physical dynamics modified: `False`

Manifest: `phase3ar_source_manifest.csv`; verification: `phase3ar_source_verification.csv`.
""", encoding="utf-8")

    geometry_rows = []
    for item in static_geometry.itertuples(index=False):
        geometry_rows.append([item.dataset, item.state, f"{1000*item.left_signed_distance_m:.3f}", f"{1000*item.right_signed_distance_m:.3f}"])
    (HERE / "phase3ar_contact_root_cause.md").write_text(f"""# Phase 3A-R pelvis/hip contact root cause

## Conclusion

Most-supported classification: **`CONTROLLER_POSTURE_ISSUE_WITH_LOW_LEFT_GEOMETRIC_MARGIN`**.

The wave static initial pose has positive left clearance (`1.523 mm`), so the
MJCF is not in pelvis/hip penetration at initialization. The unchanged current
controller creates persistent contact during wave standing alone (maximum
`{1000*current_wave_standing['maximum_pelvis_hip_penetration_m']:.3f} mm`) and
deepens it during arm motion (`{1000*current_wave_arm['maximum_pelvis_hip_penetration_m']:.3f} mm`).
Heart standing/arm produces no self-contact. Therefore motion is not required;
dynamic settling and attitude correction drive the low-margin wave posture into contact.

The specific collision geoms are MuJoCo geom `2` (`pelvis` collision mesh) and
geom `7` (`col_left_hip_roll_link`). The symmetric right collision geom is `30`
(`col_right_hip_roll_link`). The XML contains a small kinematic asymmetry:
left hip-roll body x offset `-0.000575 m`, right `0 m`. This may explain part of
the lower left margin, but the evidence does **not** prove that the geometry is wrong.

`MJCF_COLLISION_GEOMETRY_FIX_REQUIRED = NO` (not supported by current evidence).

## Static signed-distance check

{table(['dataset', 'state', 'left mm', 'right mm'], geometry_rows)}

The `0.000 mm` right-side entries are non-negative mesh-distance query boundary
values. No active right pelvis/hip contact or negative penetration was present,
so they are not treated as right-side penetration evidence.

## Current wave arm-only first-contact state

- onset: sim `{onset.sim_time:.3f} s`, reference `{onset.t:.3f} s`
- contact position: `[{onset.contact_pos_x:.6f}, {onset.contact_pos_y:.6f}, {onset.contact_pos_z:.6f}] m`
- raw contact normal: `[{onset.contact_normal_x:.6f}, {onset.contact_normal_y:.6f}, {onset.contact_normal_z:.6f}]`
- pelvis roll/pitch/yaw: `{onset_base.base_roll_rad:.6f} / {onset_base.base_pitch_rad:.6f} / {onset_base.base_yaw_rad:.6f} rad`
- CoM xyz: `{onset_base.com_x:.6f} / {onset_base.com_y:.6f} / {onset_base.com_z:.6f} m`
- applied pitch/roll feedback: `{onset_base.applied_pitch_feedback_nm:.3f} / {onset_base.applied_roll_feedback_nm:.3f} N·m`

{table(['joint', 'q rad', 'target rad', 'ctrl N·m', 'sat fraction'], onset_joint_rows)}

## Numerical tolerance

`NUMERICAL_CONTACT_TOLERANCE = 0.500 mm`. Basis: in the accepted Phase 3A
free-standing baseline, all-contact penetration p99 was about `0.421 mm` at
`0.001 s` timestep with the unchanged solver/contact settings; a rounded 0.5 mm
threshold provides a small numerical allowance. Persistent `1.289 mm` penetration
is outside it. The final candidate's `0.486–0.494 mm` persistent contact is below
the threshold but has only `0.006–0.014 mm` margin, so it is not considered robustly resolved.
""", encoding="utf-8")

    saturation = pd.concat([saturation_table(CURRENT), saturation_table(FINAL)], ignore_index=True)
    saturation.to_csv(HERE / "phase3ar_saturation_metrics.csv", index=False)
    sat_rows = [[row.experiment, f"`{row.joint_name}`", f"{row.saturation_start_sim_s:.3f}", f"{row.saturation_ratio:.4f}", f"{row.max_consecutive_saturation_s:.3f}", f"{row.max_abs_tracking_error_rad:.4f}", f"{row.max_abs_balance_addition_nm:.3f}"] for row in saturation.itertuples(index=False)]
    (HERE / "phase3ar_saturation_analysis.md").write_text(f"""# Phase 3A-R saturation analysis

{table(['experiment', 'joint', 'start s', 'ratio', 'max consecutive s', 'max |error| rad', 'max |balance add| N·m'], sat_rows)}

The controller has no integral term or equivalent accumulating state; `CONTROLLER_WINDUP`
is therefore not supported. Saturation appears after early tracking/contact/limit
problems in whole-body replay and is classified as **`TRACKING_CONFLICT_WITH_LIMIT_COUPLING`**,
not a reason to increase torque limits or gear.
""", encoding="utf-8")

    limits = pd.concat([limit_table(CURRENT), limit_table(FINAL)], ignore_index=True)
    limits.to_csv(HERE / "phase3ar_limit_metrics.csv", index=False)
    limit_rows = [[row.experiment, f"`{row.joint_name}`", f"{row.first_violation_sim_s:.3f}", f"{row.requested_target_rad:.5f}", f"{row.actual_q_rad:.5f}", f"[{row.lower_limit_rad:.3f}, {row.upper_limit_rad:.3f}]", row.reference_inside_limit, f"{row.minimum_margin_rad:.5f}", row.classification] for row in limits.itertuples(index=False)]
    (HERE / "phase3ar_limit_analysis.md").write_text(f"""# Phase 3A-R joint-limit analysis

{table(['experiment', 'joint', 'first s', 'target', 'actual q', 'MJCF range', 'real ref legal', 'min margin', 'classification'], limit_rows)}

The measured references at first violation are inside the current MJCF ranges;
actual simulated q crosses a limit while the target remains legal. The supported
classification is `CONTROLLER_LIMIT_MANAGEMENT_FAILURE`, not `MJCF_RANGE_ERROR`.
""", encoding="utf-8")

    experiments = pd.read_csv(HERE / "phase3ar_experiments.csv")
    family_rows = []
    for family, group in experiments.groupby("candidate_family"):
        best = group.sort_values(["safety_result", "balance_shape_score"], ascending=[False, True]).iloc[0]
        family_rows.append([family, best.experiment_id, best.safety_result, f"{best.balance_shape_score:.3f}", best.decision])
    (HERE / "phase3ar_balance_allocation_analysis.md").write_text(f"""# Phase 3A-R balance allocation analysis

The Phase 3A controller applies pitch/roll attitude torque only to both ankles;
hip, knee, and waist balance additions are zero. It has no integral state.

{table(['family', 'best experiment', 'safety', 'shape score', 'decision'], family_rows)}

Heart and wave raw pitch-feedback peaks are similar (`~10.95/12.27 N·m`), while
raw roll-feedback peaks differ strongly (`~0.77/7.58 N·m`). This explains why a
single global 0.7x scalar cannot preserve heart response and suppress wave
over-response simultaneously. Joint/channel-specific allocation is structurally
more appropriate and reduced the wave contact and proximal excursions without
changing arm gains. However, the final candidate still has serious knee response
ratios and persistent near-tolerance contact, so **`BALANCE_GENERALIZES_HEART_AND_WAVE = NO`**.

All real targets are `OUTPUT_RESPONSE_DESIGN_TARGET`, not `MC_GAIN_IDENTIFICATION`.
""", encoding="utf-8")

    current_heart = summary(CURRENT, "heart", "arm_only")
    final_heart = summary(FINAL, "heart", "arm_only")
    current_wave = summary(CURRENT, "wave", "arm_only")
    final_wave = summary(FINAL, "wave", "arm_only")
    heart_track = tracking_map(final_heart)
    wave_track = tracking_map(final_wave)
    heart_bal = balance_map(final_heart)
    wave_bal = balance_map(final_wave)

    def tracking_rows(final_map):
        return [[f"`{name}`", f"{row['real_excursion_rad']:.4f}", f"{row['rmse_rad']:.5f}", "UNKNOWN" if row['lag_s'] is None else f"{row['lag_s']:.3f}"] for name, row in final_map.items() if float(row["real_excursion_rad"]) >= 0.02]

    def balance_rows(final_map):
        return [[f"`{name}`", f"{row['real_excursion_rad']:.5f}", f"{row['sim_excursion_rad']:.5f}", f"{row['excursion_ratio']:.3f}", f"{row['relative_rmse_rad']:.5f}"] for name, row in final_map.items()]

    (HERE / "phase3ar_heart_validation.md").write_text(f"""# Phase 3A-R heart validation

- stable/no fall: `{final_heart['stable_no_fall']}`
- safety pass: `{final_heart['safety_pass']}`
- pelvis/hip over-tolerance samples: `{final_heart['pelvis_hip_over_tolerance_samples']}`
- persistent saturation fraction: `{final_heart['persistent_saturation_fraction']:.5f}`
- minimum limit margin: `{final_heart['minimum_limit_margin_rad']:.5f} rad`

{table(['active arm joint', 'real excursion', 'RMSE rad', 'lag s'], tracking_rows(heart_track))}

{table(['balance joint', 'real exc', 'sim exc', 'ratio', 'relative RMSE'], balance_rows(heart_bal))}

Arm tracking remains consistent with the Phase 3A candidate. Balance does not
fully generalize: ankle under-response and left-knee over-response remain.
""", encoding="utf-8")

    (HERE / "phase3ar_wave_validation.md").write_text(f"""# Phase 3A-R right-wave validation

- stable/no fall: `{final_wave['stable_no_fall']}`
- safety threshold pass: `{final_wave['safety_pass']}`
- maximum pelvis/hip penetration: `{1000*final_wave['maximum_pelvis_hip_penetration_m']:.3f} mm`
- over-tolerance samples: `{final_wave['pelvis_hip_over_tolerance_samples']}`
- persistent saturation fraction: `{final_wave['persistent_saturation_fraction']:.5f}`
- minimum limit margin: `{final_wave['minimum_limit_margin_rad']:.5f} rad`

{table(['active arm joint', 'real excursion', 'RMSE rad', 'lag s'], tracking_rows(wave_track))}

{table(['balance joint', 'real exc', 'sim exc', 'ratio', 'relative RMSE'], balance_rows(wave_bal))}

Shoulder roll (`RMSE {wave_track['right_shoulder_roll_joint']['rmse_rad']:.5f} rad`,
lag `{wave_track['right_shoulder_roll_joint']['lag_s']:.3f} s`) and wrist yaw
(`RMSE {wave_track['right_wrist_yaw_joint']['rmse_rad']:.5f} rad`, lag
`{wave_track['right_wrist_yaw_joint']['lag_s']:.3f} s`) retain their independent
tracking improvement. Knee/hip excursion over-response and persistent near-tolerance
contact prevent balance acceptance.
""", encoding="utf-8")

    current_whole = summary(CURRENT, "wave", "whole_body")
    final_whole = summary(FINAL, "wave", "whole_body")
    current_events = event_timeline(CURRENT, "wave", "whole_body")
    final_events = event_timeline(FINAL, "wave", "whole_body")
    event_rows = [[row["scenario"], row["event"], f"{row['sim_time_s']:.3f}", row["details"]] for row in current_events + final_events]
    (HERE / "phase3ar_whole_body_diagnostic.md").write_text(f"""# Phase 3A-R whole-body measured-reference diagnostic

This is a `SIMULATION_REPLAY_STABILITY_TEST`, not a claim about real-robot stability
and not an observable MC command replay.

{table(['scenario', 'event', 'sim s', 'details'], event_rows)}

Current fall: `{current_whole['fall_time_s']:.3f} s`; final-candidate fall:
`{final_whole['fall_time_s']:.3f} s`. Current/final minimum limit margin:
`{current_whole['minimum_limit_margin_rad']:.5f}/{final_whole['minimum_limit_margin_rad']:.5f} rad`.
Current/final persistent saturation fraction:
`{current_whole['persistent_saturation_fraction']:.5f}/{final_whole['persistent_saturation_fraction']:.5f}`.

For the current replay the causal order is **`TRACKING_FIRST`**: large tracking
error, pelvis/hip contact, limit violation, large balance excursion, saturation,
then fall. The final candidate does not correct the whole-body tracker conflict
and falls earlier. Root classification:
`WHOLE_BODY_TRACKER_DRIVES_SELF_CONTACT_AND_LIMIT_COUPLING`; saturation is a later
consequence, not the initiating event.
""", encoding="utf-8")

    rehearsal = read_json(HERE / "rehearsal_12_joint_regression.json")
    candidate = read_json(HERE / "simulation_controller_robustness_candidate.json")
    candidate["classification"] = "REJECTED_AFTER_FINAL_ROBUSTNESS_VALIDATION"
    candidate["final_validation"] = {
        "standing_heart_safe": summary(FINAL, "heart", "standing")["safety_pass"],
        "standing_wave_threshold_safe": summary(FINAL, "wave", "standing")["safety_pass"],
        "heart_arm_only_safe": final_heart["safety_pass"],
        "wave_arm_only_safe": final_wave["safety_pass"],
        "wave_whole_body_safe": final_whole["safety_pass"],
        "rehearsal_settled": f"{rehearsal['settled_count']}/{rehearsal['total']}",
        "PELVIS_HIP_CONTACT_RESOLVED": "NO",
        "BALANCE_GENERALIZES_HEART_AND_WAVE": "NO",
        "ARM_TRACKING_GENERALIZES": "YES",
        "VALIDATED_SIM_CONTROLLER_BASELINE": "NO",
    }
    (HERE / "simulation_controller_robustness_candidate.json").write_text(json.dumps(candidate, indent=2), encoding="utf-8")

    (HERE / "phase3ar_controller_candidate_report.md").write_text(f"""# Phase 3A-R controller candidate report

Selected for full validation: `{candidate['selected_experiment_id']}`.

- ankle pitch allocation: `0.7`
- hip pitch allocation: `0.10`
- knee pitch allocation: `0.15`
- ankle roll allocation: `0.7`
- shoulder/wrist bandwidth: unchanged at Phase 3A `8x`
- standing-reference alignment: unchanged
- physical dynamics/MJCF/hardware mapping changes: `none`
- reported effort used: `False`

Result: **`REJECTED_AFTER_FINAL_ROBUSTNESS_VALIDATION`**. It preserves arm tracking,
passes 12/12 rehearsal, and reduces contact depth, but it does not provide a
robust contact margin, does not generalize balance response across both motions,
and does not stabilize whole-body measured-reference replay.
""", encoding="utf-8")

    pelvis_resolved = False
    balance_generalizes = False
    arm_generalizes = True
    validated = pelvis_resolved and balance_generalizes and arm_generalizes
    (HERE / "phase3ar_final_gate.md").write_text(f"""# Phase 3A-R final gate

1. Contact root cause: **controller settling/attitude posture drives a low-margin
   left pelvis/hip geometry into contact**; static initialization is clear.
2. Geometry vs controller: primary evidence supports `CONTROLLER_POSTURE_ISSUE`;
   geometry asymmetry exists but is not proven erroneous.
3. Whole-body instability: legal measured targets develop tracking error, early
   contact and limit crossing; large balance excursion and saturation follow.
4. Causal order: `TRACKING_FIRST -> CONTACT -> LIMIT -> BALANCE_EXCURSION -> SATURATION -> FALL`.
5. Global 0.7x does not generalize because heart/wave roll disturbance and
   joint contribution structure differ substantially.
6. Joint-specific allocation is more reasonable and improves safety/response,
   but the tested architecture remains insufficient.
7. Shoulder-roll/wrist-yaw independent tracking improvement: **PRESERVED**.
8. Heart candidate arm-only stable: **YES**.
9. Wave candidate arm-only stable: **YES**.
10. Rehearsal: **{rehearsal['settled_count']}/{rehearsal['total']} SETTLED**.
11. Persistent pelvis/hip contact robustly resolved: **NO**; final wave
    standing/arm maxima are `{1000*final_wave_standing['maximum_pelvis_hip_penetration_m']:.3f} / {1000*final_wave_arm['maximum_pelvis_hip_penetration_m']:.3f} mm`, only barely
    below the `0.500 mm` numerical threshold and with persistent contact.
12. **`VALIDATED_SIM_CONTROLLER_BASELINE = {'YES' if validated else 'NO'}`**.

`PELVIS_HIP_CONTACT_RESOLVED = {'YES' if pelvis_resolved else 'NO'}`  
`BALANCE_GENERALIZES_HEART_AND_WAVE = {'YES' if balance_generalizes else 'NO'}`  
`ARM_TRACKING_GENERALIZES = {'YES' if arm_generalizes else 'NO'}`  
`MJCF_COLLISION_GEOMETRY_FIX_REQUIRED = NO`  
`DYNAMICS_CALIBRATION_READY = NO`

Do not enter Phase 3B physical tuning from this result. Continue controller
architecture work, especially whole-body target arbitration/limit management and
a posture envelope with meaningful contact clearance.
""", encoding="utf-8")

    plot_results()
    print(json.dumps({
        "source_verification": f"{len(verification)}/{len(verification)}",
        "PELVIS_HIP_CONTACT_RESOLVED": "NO",
        "BALANCE_GENERALIZES_HEART_AND_WAVE": "NO",
        "ARM_TRACKING_GENERALIZES": "YES",
        "VALIDATED_SIM_CONTROLLER_BASELINE": "NO",
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
