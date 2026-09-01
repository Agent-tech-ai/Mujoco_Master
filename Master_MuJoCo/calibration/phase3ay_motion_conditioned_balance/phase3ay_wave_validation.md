# Phase 3A-Y Wave Validation

Safety pass: **True**. Arm tracking retained: **True**.

## Key response metrics

| joint_name | real_excursion_rad | sim_excursion_rad | excursion_ratio | sign_match | onset_delta_s | peak_timing_delta_s | xcorr_lag_s | amplitude_band | timing_band | overall_band |
|---|---|---|---|---|---|---|---|---|---|---|
| left_ankle_pitch_joint | 0.0301042 | 0.0241447 | 0.802038 | 1 | -0.22 | -1.2 | 0.48 | GOOD | POOR | POOR |
| right_knee_joint | 0.0101536 | 0.082092 | 8.08499 | 0 | 0.46 | 0.28 | -1 | POOR | POOR | POOR |
| waist_roll_joint | 0.0386269 | 0.0218709 | 0.566209 | 1 | 0.78 | 0.14 | 0.78 | ACCEPTABLE | POOR | POOR |

## All balance channels

| plane | joint_name | excursion_ratio | sign_match | xcorr_lag_s | velocity_shape_correlation | amplitude_band | timing_band | overall_band |
|---|---|---|---|---|---|---|---|---|
| pitch | left_ankle_pitch_joint | 0.802038 | 1 | 0.48 | -0.274015 | GOOD | POOR | POOR |
| pitch | right_ankle_pitch_joint | 0.563919 | 1 | -0.42 | -0.126507 | ACCEPTABLE | POOR | POOR |
| pitch | left_knee_joint | 9.93316 | 0 | 0.74 | 0.228703 | POOR | POOR | POOR |
| pitch | right_knee_joint | 8.08499 | 0 | -1 | -0.0396693 | POOR | POOR | POOR |
| pitch | left_hip_pitch_joint | 3.4949 | 1 | -0.82 | -0.0361 | POOR | POOR | POOR |
| pitch | right_hip_pitch_joint | 2.52558 | 1 | -0.7 | 0.0237972 | POOR | POOR | POOR |
| pitch | waist_pitch_joint | 1.0651 | 0 | 0.8 | 0.0921118 | GOOD | POOR | POOR |
| roll | left_ankle_roll_joint | 2.67395 | 0 | 0.9 | 0.257665 | POOR | POOR | POOR |
| roll | right_ankle_roll_joint | 2.613 | 0 | 1 | 0.239175 | POOR | POOR | POOR |
| roll | left_hip_roll_joint | 1.4622 | 1 | -0.56 | 0.183725 | GOOD | POOR | POOR |
| roll | right_hip_roll_joint | 1.736 | 1 | -0.56 | 0.152089 | ACCEPTABLE | POOR | POOR |
| roll | waist_roll_joint | 0.566209 | 1 | 0.78 | 0.379641 | ACCEPTABLE | POOR | POOR |

Reported effort was not used. Relative IMU remains auxiliary only.
