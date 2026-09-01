# Phase 3B-V Clap arm tracking blind validation

| joint_name | real_excursion_rad | baseline_sim_excursion_rad | mass_sim_excursion_rad | baseline_rmse_rad | mass_rmse_rad | baseline_velocity_rmse_rad_s | mass_velocity_rmse_rad_s | baseline_onset_delta_s | mass_onset_delta_s | baseline_peak_delta_s | mass_peak_delta_s | baseline_recovery_delta_s | mass_recovery_delta_s | baseline_sign_agreement | mass_sign_agreement | timing_judgement |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| left_elbow_joint | 0.665526 | 0.533209 | 0.533166 | 0.154353 | 0.154285 | 0.803619 | 0.803117 | 0.12 | 0.12 | 0.12 | 0.12 | -1.35003e-13 | -1.35003e-13 | YES | YES | VALID |
| left_shoulder_pitch_joint | 0.730225 | 0.707473 | 0.707612 | 0.103305 | 0.103222 | 0.301001 | 0.300502 | 0.12 | 0.12 | -0.52 | -0.52 | 0.04 | 0.04 | YES | YES | VALID |
| left_shoulder_roll_joint | 0.059232 | 0.0547524 | 0.054749 | 0.0098311 | 0.00981992 | 0.1018 | 0.101568 | 0.06 | 0.06 | 0.06 | 0.06 | -1.35003e-13 | -1.35003e-13 | YES | YES | VALID |
| left_shoulder_yaw_joint | 0.659411 | 0.658555 | 0.658565 | 0.038051 | 0.0380503 | 0.123532 | 0.123509 | 0.06 | 0.06 | 0.5 | 0.5 | -1.35003e-13 | -1.35003e-13 | YES | YES | VALID |
| left_wrist_pitch_joint | 0.00267029 | 0.00771506 | 0.00765943 | 0.00231932 | 0.00232087 | 0.0173355 | 0.0173779 | 0.22 | 0.22 | 0.28 | 0.28 | -1.35003e-13 | -1.35003e-13 | YES | YES | INSUFFICIENT_EXCITATION |
| left_wrist_roll_joint | 0.0309 | 0.0295434 | 0.0295284 | 0.00752227 | 0.00752005 | 0.072377 | 0.0723532 | 0.06 | 0.06 | 1.4 | 1.4 | -1.35003e-13 | -1.35003e-13 | YES | YES | VALID |
| left_wrist_yaw_joint | 1.55282 | 1.55593 | 1.55593 | 0.150539 | 0.150539 | 0.385241 | 0.385242 | 0.08 | 0.08 | 0.5 | 0.5 | -1.35003e-13 | -1.35003e-13 | YES | YES | VALID |
| right_elbow_joint | 0.941909 | 0.767838 | 0.767648 | 0.197443 | 0.19732 | 0.958913 | 0.958193 | 0.12 | 0.12 | 0.18 | 0.18 | 0.1 | 0.1 | YES | YES | VALID |
| right_shoulder_pitch_joint | 0.918424 | 0.887945 | 0.888015 | 0.127833 | 0.127677 | 0.343727 | 0.342759 | 0.14 | 0.12 | 0.16 | 0.16 | 0.02 | 0.02 | YES | YES | VALID |
| right_shoulder_roll_joint | 0.0434571 | 0.0335066 | 0.0335011 | 0.00621964 | 0.00621214 | 0.0713138 | 0.0711516 | 0.06 | 0.06 | -0.2 | -0.2 | 0.04 | 0.04 | NO | NO | VALID |
| right_shoulder_yaw_joint | 0.600993 | 0.600678 | 0.600664 | 0.0347753 | 0.0347742 | 0.11685 | 0.116827 | 0.06 | 0.06 | -1.84 | -1.84 | -1.35003e-13 | -1.35003e-13 | YES | YES | VALID |
| right_wrist_pitch_joint | 0.0183105 | 0.0169956 | 0.0169652 | 0.00370598 | 0.0037014 | 0.0355276 | 0.0354424 | -0.7 | 0.08 | 1.4 | 1.4 | 0.18 | 0.2 | YES | YES | INSUFFICIENT_EXCITATION |
| right_wrist_roll_joint | 0.0255585 | 0.0224003 | 0.0223966 | 0.00588716 | 0.00588713 | 0.0517061 | 0.0516944 | 0.06 | 0.06 | 1.36 | 1.36 | -1.35003e-13 | -1.35003e-13 | YES | YES | VALID |
| right_wrist_yaw_joint | 1.14349 | 1.14288 | 1.14288 | 0.110711 | 0.110711 | 0.296493 | 0.29648 | 0.08 | 0.08 | 0.64 | 0.64 | -1.35003e-13 | -1.35003e-13 | YES | YES | VALID |

- sufficiently excited joints: `12`
- candidate/baseline mean position RMSE ratio: `0.999523`
- candidate/baseline mean velocity RMSE ratio: `0.999125`
- new >5% per-joint RMSE regression: `NO`
- candidate introduced a new response-sign disagreement versus baseline: `NO`
- absolute candidate/real dominant-response sign agreement: `11/12`; pre-existing disagreements are retained as diagnostics

`CONTROLLER_BASELINE_PRESERVED = YES`
