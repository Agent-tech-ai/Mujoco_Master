# Phase 3A-V balance and standing validation

## Standing equilibrium

| joint | legacy settled-real rad | candidate settled-real rad |
| --- | --- | --- |
| left_ankle_pitch_joint | 0.28394 | 0.01080 |
| right_ankle_pitch_joint | 0.44338 | 0.07329 |
| left_hip_pitch_joint | 0.01579 | -0.02940 |
| right_hip_pitch_joint | 0.07744 | 0.03218 |
| left_knee_joint | 0.16008 | 0.09371 |
| right_knee_joint | 0.00646 | -0.02274 |
| waist_pitch_joint | -0.09149 | -0.00011 |
| waist_roll_joint | 0.05352 | -0.00053 |

- bilateral knee mean absolute mismatch: `0.08327 -> 0.05823 rad`
- standing-reference decision: **GENERALIZES**

## Arm-only autonomous balance response

| joint | real excursion | legacy ratio | candidate ratio | legacy RMSE | candidate RMSE | candidate lag s | candidate recovery s |
| --- | --- | --- | --- | --- | --- | --- | --- |
| left_ankle_pitch_joint | 0.03010 | 0.000 | 2.599 | 0.44459 | 0.02739 | -0.260 | UNKNOWN |
| right_ankle_pitch_joint | 0.02799 | 0.001 | 1.666 | 0.33176 | 0.01832 | -0.460 | 1.971 |
| left_hip_pitch_joint | 0.01400 | 2.847 | 3.933 | 0.13781 | 0.02458 | -0.840 | UNKNOWN |
| right_hip_pitch_joint | 0.01707 | 7.052 | 3.399 | 0.04682 | 0.01935 | -0.840 | UNKNOWN |
| left_knee_joint | 0.00729 | 0.053 | 20.455 | 0.16109 | 0.04700 | 0.040 | UNKNOWN |
| right_knee_joint | 0.01015 | 0.211 | 8.296 | 0.00998 | 0.04067 | 1.000 | 1.211 |
| waist_pitch_joint | 0.02804 | 5.998 | 0.882 | 0.06379 | 0.01169 | -0.340 | 1.051 |
| waist_roll_joint | 0.03863 | 4.404 | 0.563 | 0.05139 | 0.02062 | 0.100 | 1.331 |

- 0.7x balance response aggregate RMSE gate: **GENERALIZES_OR_PARTIAL**
- ankle under-response check: **NO_OVER_CORRECTION_DETECTED**
- free-base safety gate: **FAIL**

Evidence adjudication: **PARTIAL_GENERALIZATION** for the `0.7x` balance-gain
candidate. The candidate removes the legacy fall and reduces aggregate relative
RMSE, and it does not repeat the heart left-ankle under-response. However, the
wave validation shows excursion over-response: left/right ankle pitch ratios
`2.599/1.666` and left/right knee ratios `20.455/8.296`. It also has the
persistent `pelvis <-> left_hip_roll_link` contact documented in
`phase3av_contact_diagnostic.md`. Therefore the balance candidate is not fully
independently validated.

IMU results in `phase3av_relative_imu_metrics.csv` use only relative roll/pitch and gyro shape because `IMU_TRANSFORM = PARTIAL`. Reported effort is not used.
