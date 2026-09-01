# Phase 3B-C2 — Phase 3B-V final gate reinterpretation

Date: 2026-08-28

## New evidence and decision rule

`REAL_CLAP_HAND_CONTACT = CONFIRMED`

Soft-engineer field confirmation establishes that X2 MC preset Clap `area=11`, `motion=3017` normally makes physical
hand-to-hand contact. The sole frozen simulation pair, `left_wrist_roll_link <-> right_wrist_roll_link`, occurs only in
the three expected Clap closure episodes. It is therefore classified as `EXPECTED_TASK_CONTACT` for those windows.

The gesture-aware exception removes this specific pair from the absolute Clap safety veto. It does not change the
MJCF, weaken the global self-collision rule, or exclude any other pair/time window.

## Frozen blind-validation evidence

No Phase 3B-V replay or metric was recomputed.

### Independent Clap response

| Metric | Frozen result |
|---|---:|
| Sagittal aggregate absolute-error improvement | `9.591%` |
| Valid sagittal channels improved / degraded | `6 / 1` |
| Left/right ankle-pitch absolute-error improvement | `8.852% / 5.969%` |
| Left/right hip-pitch absolute-error improvement | `12.649% / 4.673%` |
| Left/right knee absolute-error improvement | `13.418% / 12.757%` |
| Waist-roll absolute-error improvement | `43.614%` — small real excursion |
| Waist-pitch absolute-error change | `-6.507%` — degraded |
| Lateral aggregate | `INSUFFICIENT_AGGREGATE_EXCITATION` |
| Knee validation strength | `WEAK` — small real excursion and pre-existing sign/shape conflict |
| Candidate/baseline arm position RMSE ratio | `0.999523` |
| Candidate/baseline arm velocity RMSE ratio | `0.999125` |

### Safety and contact

- no new fall;
- no new limit violation;
- no persistent actuator saturation;
- no foot-slip regression;
- no new contact pair or episode;
- expected-contact onset difference between baseline and candidate is no more than `1.000 ms`;
- `MASS_DIRECTION_CAUSES_CONTACT = NO`;
- candidate does not systematically worsen expected-contact penetration, force, or duration.

### Cross-motion interpretation

Heart and Wave results remain frozen. Together with independent Clap, they show a repeatable position-space response
direction: increasing relative lower-limb mass distribution in this simulation generally reduces observed leg-response
mismatch while preserving controller tracking and comparative safety. The evidence supports the direction of
sensitivity across motions, not the exact magnitude or the physical identity of the perturbed parameter.

The prior Wave right-knee context remains: real excursion `0.0101536374 rad`, baseline simulation excursion
`0.0820920169 rad`, ratio `8.084986x`, and absolute error `0.0719383795 rad`. Absolute error, not the small-denominator
ratio, remains the decision metric.

## Final gate

| Gate | Status | Basis |
|---|---|---|
| REAL_CLAP_HAND_CONTACT | CONFIRMED | Explicit soft-engineer field confirmation for preset 3017/area 11. |
| SPECIFIC_WRIST_CONTACT_EXPECTED | YES | Exact pair aligns with all three expected Clap closures. |
| SPECIFIC_EXPECTED_CONTACT_EXCLUSION | YES — CLAP WINDOWS ONLY | Pair/time/motion-bounded exception; no global collision exemption. |
| MASS_DIRECTION_CAUSES_CONTACT | NO | Identical pair/episodes and onset within 1 ms in both frozen conditions. |
| ABSOLUTE_CLAP_SAFETY_PASS | YES — GESTURE_AWARE | Expected pair excluded only in approved windows; all remaining frozen safety checks pass. |
| PHYSICAL_DIRECTION_MAGNITUDE_SUPPORT | VALIDATED_ACROSS_MOTIONS | Heart, Wave, and independent Clap provide consistent aggregate directional support with recorded local exceptions. |
| EXACT_PARAMETER_IDENTIFIED | NO | A sensitivity direction and one tested perturbation do not identify real X2 mass. |
| PHYSICAL_DIRECTION_GENERALIZES | YES | Independent Clap confirms the direction without arm or safety regression. |
| CONTROLLER_BASELINE_PRESERVED | YES | Arm position/velocity ratios remain approximately one and no controller retuning occurred. |
| SAFETY_BASELINE_PRESERVED | YES | Candidate creates no new non-expected safety failure. |
| POSITION_SPACE_PHYSICAL_DIRECTION_VALIDATED | YES | Magnitude-direction validation only. |
| DYNAMICS_CALIBRATION_READY | NO | Physical sign/zero, effort, IMU, and command-observability blockers remain. |

## Engineering classification

`bs_mass_lower_plus08 = CROSS_MOTION_VALIDATED_PHYSICAL_SENSITIVITY_DIRECTION`

`bs_mass_lower_plus08 != IDENTIFIED_REAL_X2_MASS`

Validated statement:

> Across Heart, Wave, and independent Clap motion, increasing relative lower-limb mass distribution in this
> simulation produces a repeatable position-space response direction that generally reduces observed leg-response
> mismatch while preserving controller tracking and safety.

This does not establish that the real X2 lower-limb mass is `+8%`, nor that mass alone causes the residual mismatch.

## Preserved limitations

- Clap knee real excursion is small; knee-specific evidence remains `WEAK`.
- Clap lateral excitation remains insufficient.
- `PHYSICAL_SIGN = UNKNOWN`.
- `PHYSICAL_ZERO = UNKNOWN`.
- `EFFORT_SEMANTICS = UNKNOWN`.
- `IMU_TRANSFORM = PARTIAL`.
- `MC_INTERNAL_COMMAND = UNOBSERVABLE`.

Forbidden classifications remain: `REAL_X2_LOWER_LIMB_MASS=+8%`, `HARDWARE_MASS_IDENTIFIED`, `CALIBRATED_MJCF`,
`REAL_MASS_CALIBRATION`, `ACTUATOR_SYSTEM_IDENTIFICATION`, and `DYNAMICS_CALIBRATION_READY=YES`.

