# Phase 3D Master Status — Calibration Evidence Consolidation

Freeze date: 2026-08-31

## Executive state

Phase 1 through Phase 3C is frozen as an auditable position-space calibration evidence package. No calibrated hardware model exists.

- `POSITION_ONLY_CALIBRATION_READY = YES`
- `POSITION_RESPONSE_BASELINE_READY = YES` for the restricted Phase 3A position/controller scope
- `ARM_TRACKING_GENERALIZES = YES`
- `CONTACT_SAFETY_ROBUST = YES` for the frozen Phase 3A-X/Y simulation safety architecture and tested conditions
- `POSITION_SPACE_PHYSICAL_DIRECTION_VALIDATED = YES`
- `VALIDATED_SIM_CONTROLLER_BASELINE = NO` for cross-motion balance amplitude/timing
- `EVIDENCE_BACKED_PHYSICAL_MODEL_UPDATE_READY = NO`
- `DYNAMICS_CALIBRATION_READY = NO`
- `CALIBRATED_MJCF_CREATION_ALLOWED = NO`

## 1. Current model baseline

The immutable physical-model baseline is `assets/Master/ff_master_ultra.xml`, SHA-256 `89619295...4353db`. It compiles to 33 bodies including world, 32 joints (one free plus 31 hinges), 31 actuators and 67 sensors. Its resolved inertial mass is 43.474796659 kg across 32 non-world inertial bodies.

The X2-limit derivative `ff_master_ultra_x2_limits.xml`, SHA-256 `6d594049...b9399dd`, compiles to 31 joints (one free plus 30 hinges), 30 actuators and 65 sensors. `scene_x2_fixed.xml` and `scene_x2_free.xml` remain the fixed/free evaluation scenes. None is calibrated against physical X2 dynamics.

`joint_mapping.csv` is frozen at SHA-256 `3975d90f...fb5ff04`. Live name/index mapping is confirmed, but physical sign, zero and encoder offset remain unknown.

## 2. Completed controller improvements

All items below are simulation-only engineering results.

| Phase | Completed result | Preserved limitation |
|---|---|---|
| 3A position/controller alignment | Heart shoulder-roll lag `0.240 → 0.100 s`; wrist-yaw lag `0.380 → 0.160 s`; bilateral knee equilibrium absolute offset `0.12030 → 0.00843 rad`; free-base standing 10 s stable; rehearsal 12/12 settled | Standing offsets are `SIMULATION_REFERENCE_ALIGNMENT`, not hardware zero; gains are not real Kp/Kd |
| 3A-V independent Wave validation | Excited right shoulder-roll RMSE `0.10414 → 0.02656 rad`, lag `0.840 → 0.080 s`; right wrist-yaw RMSE `0.38912 → 0.14992 rad`, lag `0.420 → 0.160 s`; arm tracking generalizes | Wave exposed persistent pelvis/hip contact and excessive balance response in the earlier controller |
| 3A-X constraint-aware safety | Contact-aware retreat, direction-aware limit envelope, slew/authority scaling and arbitration removed tested Wave contact/fall/limit/saturation chain; perturbations 8/8 pass; rehearsal 12/12 settled | Heart/Wave balance-response similarity did not both pass |
| 3A-Y motion-conditioned response | Continuous activity/asymmetry scheduling preserved arm tracking and hard safety without using motion ID | Balance amplitude and timing still do not generalize across Heart/Wave; candidate is not a validated response baseline |

The frozen analysis controller stack is Phase 3A arm tracking/standing alignment + Phase 3A-X safety shell + Phase 3A-Y motion-conditioned response. It is a reproducible simulation experiment baseline, not hardware calibration and not a fully validated balance-response baseline.

## 3. Real-motion evidence

All captures are read-only robot-state recordings. Preset execution was performed externally by the operator through the existing MC-compatible path; the recorders sent no command.

| Motion | Real source | Quality / independence | Main evidence |
|---|---|---|---|
| Heart | native MC preset 1007, area 3; raw SHA-256 `a6757b13...2c539d9` | 5.659 s motion; 13.78 s pre-roll; 11.60 s post-roll; six required streams ~46–47 Hz; replay ready | Bilateral mirrored J2/J7 measured trajectories; initial tracking lags; knee equilibrium mismatch; ankle/hip/knee/waist response baseline |
| Wave (right) | native MC preset 1002, area 2; raw SHA-256 `185433f6...472436` | 4.349 s motion; 13.067/8.017 s pre/post; sufficiently independent from Heart | Independent arm-bandwidth generalization; unilateral/asymmetric disturbance; revealed earlier contact and balance over-response |
| Clap | native MC preset 3017, area 11; raw SHA-256 `37a56c53...9f513` | 5.444 s motion; 9.849/6.048 s pre/post; sufficiently independent from Heart/Wave | Blind third-motion validation; real hand-to-hand contact confirmed; exact simulated wrist pair/time windows classified as expected task contact |

The measured references are immutable and separately hashed in the final freeze manifest. Reported effort is excluded from fitting and validation. IMU is used only for relative/auxiliary comparisons because its complete transform is not known.

## 4. Cross-motion physical sensitivity direction

`bs_mass_lower_plus08 = CROSS_MOTION_VALIDATED_PHYSICAL_SENSITIVITY_DIRECTION`

Across frozen Heart, Wave and independent Clap evidence, increasing relative lower-limb mass distribution in this simulation generally moves leg-response error in an improving direction while preserving arm tracking and comparative safety.

- Heart aggregate absolute-error improvement: 0.373% across the 12 reported channels, with mixed local changes.
- Wave aggregate absolute-error improvement: 9.189% across 12 channels.
- Clap aggregate absolute-error improvement: 9.366% across 12 channels; valid sagittal aggregate improvement 9.591%, with 6/7 valid sagittal channels improved.
- Clap candidate/baseline arm position and velocity RMSE ratios: 0.999523 and 0.999125.
- No candidate-caused fall, limit violation, persistent saturation, slip regression or non-expected contact was introduced.

This validates a response direction only. It does not identify mass as the unique cause and does not validate the `+8%` magnitude.

## 5. Pelvis inertial provenance discrepancy

Thirty-one explicit MJCF inertials numerically match the same-bundle Ultra URDF within conversion precision. The pelvis is the exception:

- current MJCF has no explicit pelvis `<inertial>`;
- MuJoCo derives compiled pelvis mass `5.031810659 kg` from the collision mesh/default density;
- same-bundle URDF pelvis mass is `3.523487 kg`;
- difference is `+1.508323659 kg`, or `+42.8077%` relative to the URDF value;
- this inflates non-lower mass and changes lower/non-lower ratio from URDF `0.657462` to compiled MJCF `0.620499`.

AimDK X2 SDK simulator excerpts reproduce the same model lineage, including the mesh-derived pelvis structure and sampled inertials. This confirms provenance, not physical metrology. Neither pelvis value is accepted as true X2 hardware mass.

## 6. Open hardware-evidence blockers

- physical joint sign for fitted joints;
- physical zero and encoder offset;
- independent per-link mass, CoM and inertia for the exact robot/payload configuration;
- complete chest/torso IMU raw-axis, driver-output and base-frame transform;
- `JointState.effort` source semantics, conversion, sign and filtering;
- observable post-arbitration MC internal joint reference;
- physical contact/foot-force/slip baseline;
- actuator/gear/torque/current/velocity/acceleration and compliance/backlash/friction evidence;
- clock synchronization and end-to-end command/state latency needed for identification.

Details and closure tests are in `phase3d_open_blockers.md`.

## 7. Safe simulation-only terminology

The following labels are allowed:

- `ACCEPTED_SIM_CONTROLLER_ALIGNMENT`
- `SIMULATION_REFERENCE_ALIGNMENT`
- `SAFETY_ARCHITECTURE_CANDIDATE_NOT_VALIDATED_RESPONSE_BASELINE`
- `SIMULATION_MOTION_CONDITIONED_BALANCE_CANDIDATE_NOT_VALIDATED_RESPONSE_BASELINE`
- `PHYSICAL_SENSITIVITY_EXPERIMENT`
- `CROSS_MOTION_VALIDATED_PHYSICAL_SENSITIVITY_DIRECTION`
- `POSITION_SPACE_PHYSICAL_DIRECTION_VALIDATED`
- `OUTPUT_RESPONSE_COMPARISON`
- `MODEL_CONVERSION_PROVENANCE_DISCREPANCY`

## 8. Forbidden hardware-calibration claims

The project must not claim:

- `CALIBRATED_MJCF` or `HARDWARE_CALIBRATED`;
- real X2 lower-limb mass is `+8%`;
- identified real mass, CoM, inertia, damping, friction or armature;
- `REAL_KP`, `REAL_KD`, real gear ratio or real torque limit;
- measured/estimated torque semantics for `effort`;
- actuator or full system identification;
- confirmed absolute IMU attitude alignment;
- confirmed physical sign/zero/encoder offset;
- validated MC internal command tracking.

## 9. Evidence required before continuing

The minimum restart evidence is: exact-configuration manufacturer/CAD mass properties; documented or physically verified joint axes and zero datums; deployed IMU driver/TF transform chain; deployed HAL effort assignment chain; and, for system identification, a read-only time-indexed MC internal joint reference. A synchronized physical contact/force and external pose reference is strongly recommended before fitting contact or balance dynamics.

## 10. Calibrated MJCF decision

`CALIBRATED_MJCF_CREATION_ALLOWED = NO`

A future derived MJCF may be created only after the affected parameters have traceable A/B/C-level physical evidence, uncertainty and configuration provenance, and after the corresponding gate is explicitly reopened and passed. The original `ff_master_ultra.xml` must remain immutable.

## Final required conclusion

`POSITION_SPACE_PHYSICAL_DIRECTION_VALIDATED = YES`

`EVIDENCE_BACKED_PHYSICAL_MODEL_UPDATE_READY = NO`

`DYNAMICS_CALIBRATION_READY = NO`

