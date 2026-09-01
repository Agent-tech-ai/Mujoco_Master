# Phase 3C Final Gate

## Final classification

`CROSS_MOTION_SENSITIVITY_DIRECTION_CONFIRMED BUT REAL_PARAMETER_MAGNITUDE_UNIDENTIFIED`

Across Heart, Wave and independent Clap motion, increasing relative lower-limb mass distribution in this simulation produced a repeatable position-space response direction that generally reduced observed leg-response mismatch while preserving controller tracking and the interpreted safety baseline. This remains a simulation sensitivity result.

`bs_mass_lower_plus08 = CROSS_MOTION_VALIDATED_PHYSICAL_SENSITIVITY_DIRECTION`

`bs_mass_lower_plus08 != identified real X2 mass`

## Physical evidence verdict

`CURRENT_MJCF_LOWER_LIMB_MASS_UNDERESTIMATED = INSUFFICIENT_EVIDENCE`

The AimDK X2 SDK simulator artifact confirms baseline source lineage for sampled bodies, but supplies no independent real-hardware metrology. The same-family URDF/MJCF comparison exposes a pelvis auto-inertia conversion discrepancy, not proof that the true lower limbs are under-massed. No source supports the exact `+8%` magnitude.

## Gates

| Gate | Result | Reason |
|---|---|---|
| `MASS_PARAMETER_EVIDENCE_READY` | `NO` | No independent A/B/C X2 mass measurement or documented physical per-link data. |
| `INERTIA_PARAMETER_EVIDENCE_READY` | `NO` | No independent X2 inertia tensors or uncertainty/provenance. |
| `PHYSICAL_SIGN_READY` | `NO` | Names/indices are confirmed; physical positive directions for fitted joints are not. |
| `PHYSICAL_ZERO_READY` | `NO` | No physical datum → measured encoder position evidence chain. |
| `IMU_TRANSFORM_READY` | `NO` | Message `frame_id` and simulator sites are known, but deployed raw-axis/output transform remains partial. |
| `EFFORT_SEMANTICS_READY` | `NO` | N·m label is known; measured/estimated/current-derived/commanded source remains unknown. |
| `MC_COMMAND_OBSERVABLE` | `NO` | No verified post-arbitration per-joint MC reference stream. |
| `EVIDENCE_BACKED_PHYSICAL_MODEL_UPDATE_READY` | `NO` | Required physical parameter evidence is not closed. |

## Preserved results and restrictions

- `POSITION_SPACE_PHYSICAL_DIRECTION_VALIDATED = YES` remains unchanged from Phase 3B-C2.
- The clap wrist-to-wrist contact exception remains limited to the confirmed preset-3017 clap closure windows and pair. No other self-collision is ignored.
- No `ff_master_ultra_calibrated.xml` was created.
- No mass, inertia, CoM, damping, friction, armature, gear, actuator or controller parameter was modified.
- `joint_mapping.csv` was not modified.
- Reported effort was not used.
- No robot was accessed or controlled.

## What can proceed

Position-only/output-response experiments may continue under their existing labels and limits, but they must not be called hardware mass calibration, actuator identification or full dynamics calibration. The pelvis inertial discrepancy may be investigated as a separate model-conversion/provenance issue once independent physical evidence is available.

`DYNAMICS_CALIBRATION_READY = NO`

