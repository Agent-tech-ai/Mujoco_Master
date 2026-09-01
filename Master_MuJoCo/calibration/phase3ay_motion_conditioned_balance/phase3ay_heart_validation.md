# Phase 3A-Y Heart Validation

Safety pass: **True**. Arm tracking retained: **True**.

## Key response metrics

| joint_name | real_excursion_rad | sim_excursion_rad | excursion_ratio | sign_match | onset_delta_s | peak_timing_delta_s | xcorr_lag_s | amplitude_band | timing_band | overall_band |
|---|---|---|---|---|---|---|---|---|---|---|
| left_ankle_pitch_joint | 0.0461992 | 0.03494 | 0.75629 | 1 | -0.4 | 2.44 | -1 | GOOD | POOR | POOR |
| right_knee_joint | 0.0237765 | 0.054343 | 2.28557 | 1 | -0.52 | 0.76 | 0.18 | POOR | POOR | POOR |
| waist_roll_joint | 0.01018 | 0.00812792 | 0.798424 | 0 | -0.14 | 0.44 | 0.3 | GOOD | ACCEPTABLE | POOR |

## All balance channels

| plane | joint_name | excursion_ratio | sign_match | xcorr_lag_s | velocity_shape_correlation | amplitude_band | timing_band | overall_band |
|---|---|---|---|---|---|---|---|---|
| pitch | left_ankle_pitch_joint | 0.75629 | 1 | -1 | -0.234232 | GOOD | POOR | POOR |
| pitch | right_ankle_pitch_joint | 1.2268 | 1 | -0.98 | -0.259065 | GOOD | POOR | POOR |
| pitch | left_knee_joint | 7.49148 | 0 | 0.38 | -0.000574253 | POOR | POOR | POOR |
| pitch | right_knee_joint | 2.28557 | 1 | 0.18 | 0.350752 | POOR | POOR | POOR |
| pitch | left_hip_pitch_joint | 0.838608 | 1 | 1 | -0.362699 | GOOD | POOR | POOR |
| pitch | right_hip_pitch_joint | 0.766909 | 1 | 1 | -0.445774 | GOOD | POOR | POOR |
| pitch | waist_pitch_joint | 0.638061 | 0 | 1 | -0.0770332 | ACCEPTABLE | POOR | POOR |
| roll | left_ankle_roll_joint | 0.99935 | 0 | 1 | 0.0296827 | GOOD | POOR | POOR |
| roll | right_ankle_roll_joint | 3.18944 | 0 | -1 | -0.0639784 | POOR | POOR | POOR |
| roll | left_hip_roll_joint | 0.656998 | 0 | -1 | 0.0720656 | ACCEPTABLE | POOR | POOR |
| roll | right_hip_roll_joint | 1.98712 | 1 | 1 | 0.158503 | ACCEPTABLE | POOR | POOR |
| roll | waist_roll_joint | 0.798424 | 0 | 0.3 | 0.134785 | GOOD | ACCEPTABLE | POOR |

Reported effort was not used. Relative IMU remains auxiliary only.
