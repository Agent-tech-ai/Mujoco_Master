# Phase 3B-V Clap original physical baseline

Main mismatch: sagittal over-response in both ankle-pitch, hip-pitch, and knee channels. Knee ratios are
large with only `0.006519`–`0.008054 rad` real excursion.

| plane | joint_name | real_excursion_rad | baseline_sim_excursion_rad | baseline_abs_error_rad | baseline_rmse_rad | baseline_velocity_rmse_rad_s | excitation_flag |
|---|---|---|---|---|---|---|---|
| SAGITTAL | left_ankle_pitch_joint | 0.0358562 | 0.0684663 | 0.03261 | 0.0269373 | 0.0191722 | SUFFICIENT |
| SAGITTAL | left_hip_pitch_joint | 0.0226259 | 0.0498687 | 0.0272428 | 0.0236887 | 0.0209328 | SUFFICIENT |
| SAGITTAL | left_knee_joint | 0.00651932 | 0.0460205 | 0.0395012 | 0.0281895 | 0.0318511 | SMALL_REAL_EXCURSION |
| SAGITTAL | right_ankle_pitch_joint | 0.0333638 | 0.0716727 | 0.0383089 | 0.0384188 | 0.024017 | SUFFICIENT |
| SAGITTAL | right_hip_pitch_joint | 0.021667 | 0.0530528 | 0.0313858 | 0.0301684 | 0.0208042 | SUFFICIENT |
| SAGITTAL | right_knee_joint | 0.00805378 | 0.047824 | 0.0397702 | 0.0315988 | 0.0353019 | SMALL_REAL_EXCURSION |
| SAGITTAL | waist_pitch_joint | 0.00736861 | 0.00469274 | 0.00267587 | 0.00349266 | 0.00767719 | SMALL_REAL_EXCURSION |
| LATERAL | waist_roll_joint | 0.00701887 | 0.00744944 | 0.00043057 | 0.00178963 | 0.00888134 | SMALL_REAL_EXCURSION |
