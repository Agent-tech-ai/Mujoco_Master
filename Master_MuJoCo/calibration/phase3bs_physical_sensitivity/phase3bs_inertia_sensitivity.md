# Phase 3B-S — Rotational inertia sensitivity

Classification: **PHYSICAL_SENSITIVITY_EXPERIMENT — NOT HARDWARE CALIBRATION**.

Only `selected_link_rotational_inertia_scale` changed. Controller, source MJCF, gear, torque/force/ctrl limits, and all other physical families remained frozen.

## Heart/Wave runs

| experiment_id | dataset | parameter_value | heart_left_ankle_excursion_rad | heart_knee_excursion_rad | heart_waist_roll_excursion_rad | wave_right_knee_excursion_rad | wave_ankle_excursion_rad | base_pitch_excursion_rad | base_roll_excursion_rad | foot_slip_max_m | contact_penetration_max_m | arm_tracking_rmse_rad | safety_pass |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| bs_inertia_minus10 | heart | 0.9 | 0.0348026 | 0.0552024 | 0.00812939 |  |  | 0.0637247 | 0.00382751 | 0.007244 | 0.000446473 | 0.110971 | True |
| bs_inertia_minus10 | wave | 0.9 |  |  |  | 0.0813671 | 0.0196761 | 0.113285 | 0.0116799 | 0.00979373 | 0.000554927 | 0.0778702 | True |
| bs_inertia_plus10 | heart | 1.1 | 0.0349158 | 0.055129 | 0.00812317 |  |  | 0.0635715 | 0.0038409 | 0.00721371 | 0.000446727 | 0.110975 | True |
| bs_inertia_plus10 | wave | 1.1 |  |  |  | 0.0838855 | 0.02232 | 0.112094 | 0.0106727 | 0.00975915 | 0.000568656 | 0.0778683 | True |

## Central normalized local sensitivity

Definition: `(y_high - y_low) / |y_baseline| / (normalized_p_high - normalized_p_low)`. Sign is directional; magnitude ranks local sensitivity.

| physical_family | parameter | low_experiment | high_experiment | low_normalized_change | high_normalized_change | safety_effect | heart_left_ankle_excursion_normalized_sensitivity | heart_left_ankle_excursion_sweep_percent_change | heart_knee_excursion_normalized_sensitivity | heart_knee_excursion_sweep_percent_change | heart_waist_roll_excursion_normalized_sensitivity | heart_waist_roll_excursion_sweep_percent_change | wave_right_knee_excursion_normalized_sensitivity | wave_right_knee_excursion_sweep_percent_change | wave_ankle_excursion_normalized_sensitivity | wave_ankle_excursion_sweep_percent_change | base_pitch_excursion_normalized_sensitivity | base_pitch_excursion_sweep_percent_change | base_roll_excursion_normalized_sensitivity | base_roll_excursion_sweep_percent_change | foot_slipax_normalized_sensitivity | foot_slipax_sweep_percent_change | contact_penetrationax_normalized_sensitivity | contact_penetrationax_sweep_percent_change | arm_tracking_rmse_normalized_sensitivity | arm_tracking_rmse_sweep_percent_change |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ROTATIONAL_INERTIA | selected_link_rotational_inertia_scale | bs_inertia_minus10 | bs_inertia_plus10 | -0.1 | 0.1 | PASS_ALL_HEART_WAVE | 0.0162096 | 0.324192 | -0.00664975 | -0.132995 | -0.00382654 | -0.0765308 | 0.153387 | 3.06774 | 0.662108 | 13.2422 | -0.038332 | -0.766639 | -0.315887 | -6.31774 | -0.0190257 | -0.380513 | 0.0700444 | 1.40089 | 5.29194e-05 | 0.00105839 |

## Cross-motion assessment

| experiment_id | family | level | wave_knee_absolute_error_improvement_rad | heart_ankle_absolute_error_regression_rad | heart_waist_roll_absolute_error_regression_rad | mean_arm_tracking_rmse_ratio_to_baseline | heart_wave_safety_pass | shared_direction_criteria_pass | classification |
|---|---|---|---|---|---|---|---|---|---|
| bs_inertia_minus10 | ROTATIONAL_INERTIA | minus | 0.00072491 | 0.000137447 | -1.46766e-06 | 1.00005 | True | False | PHYSICAL_SENSITIVITY_EXPERIMENT |
| bs_inertia_plus10 | ROTATIONAL_INERTIA | plus | -0.00179346 | 2.41747e-05 | 4.75271e-06 | 1.00006 | True | False | PHYSICAL_SENSITIVITY_EXPERIMENT |

No value above is claimed to be a real X2 parameter.
