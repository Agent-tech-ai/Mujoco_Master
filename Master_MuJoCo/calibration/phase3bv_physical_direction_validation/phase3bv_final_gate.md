# Phase 3B-V blind-validation final gate — Phase 3B-C2 reinterpretation

`PENDING_THIRD_MOTION_CAPTURE` → `CLAP_CAPTURE_VALID` → `BLIND_DUAL_REPLAY_COMPLETE` → `REAL_CLAP_CONTACT_CONFIRMED`

Reinterpretation date: 2026-08-28. No replay, retuning, or parameter change was performed.

## Final gate

| Gate | Status |
|---|---|
| CLAP_CAPTURE_VALID | YES |
| CLAP_LEG_BALANCE_EXCITATION_SUFFICIENT | YES |
| CLAP_KNEE_VALIDATION_STRENGTH | WEAK |
| REAL_CLAP_HAND_CONTACT | CONFIRMED |
| CLAP_WRIST_PAIR_CLASSIFICATION | EXPECTED_TASK_CONTACT — THREE CLOSURE WINDOWS ONLY |
| MASS_DIRECTION_CAUSES_CONTACT | NO |
| ABSOLUTE_CLAP_SAFETY_PASS | YES — GESTURE_AWARE EXPECTED-CONTACT EXCLUSION |
| PHYSICAL_DIRECTION_GENERALIZES | YES |
| CONTROLLER_BASELINE_PRESERVED | YES |
| SAFETY_BASELINE_PRESERVED | YES |
| POSITION_SPACE_PHYSICAL_DIRECTION_VALIDATED | YES |
| EXACT_PHYSICAL_PARAMETER_IDENTIFIED | NO |
| DYNAMICS_CALIBRATION_READY | NO |

## Validated engineering interpretation

`bs_mass_lower_plus08 = CROSS_MOTION_VALIDATED_PHYSICAL_SENSITIVITY_DIRECTION`

This is a magnitude-direction result in position space. Across frozen Heart, Wave, and independent Clap evidence,
increasing relative lower-limb mass distribution in the simulation produces a repeatable response direction that
generally reduces observed leg-response mismatch while preserving arm tracking and safety.

For independent Clap:

- sagittal aggregate absolute-error improvement: `9.591%`;
- valid sagittal channels improved / degraded: `6 / 1`;
- candidate/baseline arm position RMSE ratio: `0.999523`;
- candidate/baseline arm velocity RMSE ratio: `0.999125`;
- no new fall, limit violation, persistent saturation, slip regression, or non-expected collision;
- the sole wrist-to-wrist contact is expected real task contact and is not candidate-caused.

The expected-contact exception is strictly limited to `left_wrist_roll_link <-> right_wrist_roll_link` during the
three preset-3017 Clap closure windows. Every other self-contact remains a safety failure. MJCF collision filters are
unchanged.

## Identification boundary

`bs_mass_lower_plus08 != IDENTIFIED_REAL_X2_MASS`

The result does not support `REAL_X2_LOWER_LIMB_MASS=+8%`, `HARDWARE_MASS_IDENTIFIED`, `CALIBRATED_MJCF`,
`REAL_MASS_CALIBRATION`, or `ACTUATOR_SYSTEM_IDENTIFICATION`.

Persistent blockers: `PHYSICAL_SIGN=UNKNOWN`, `PHYSICAL_ZERO=UNKNOWN`, `EFFORT_SEMANTICS=UNKNOWN`,
`IMU_TRANSFORM=PARTIAL`, `MC_INTERNAL_COMMAND=UNOBSERVABLE`. Clap knee evidence remains weak and Clap lateral
excitation remains insufficient.

Therefore `DYNAMICS_CALIBRATION_READY = NO` even though
`POSITION_SPACE_PHYSICAL_DIRECTION_VALIDATED = YES`.

