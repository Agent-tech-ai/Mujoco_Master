# Phase 3B-S — Armature sensitivity

Classification: **PHYSICAL_SENSITIVITY_EXPERIMENT — NOT HARDWARE CALIBRATION**.

Only `pitch_chain_armature_scale` changed. Controller, source MJCF, gear, torque/force/ctrl limits, and all other physical families remained frozen.

## Heart/Wave runs

| experiment_id | dataset | parameter_value | heart_left_ankle_excursion_rad | heart_knee_excursion_rad | heart_waist_roll_excursion_rad | wave_right_knee_excursion_rad | wave_ankle_excursion_rad | base_pitch_excursion_rad | base_roll_excursion_rad | foot_slip_max_m | contact_penetration_max_m | arm_tracking_rmse_rad | safety_pass |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| bs_armature_minus20 | heart | 0.8 | 0.0350848 | 0.0551603 | 0.00813116 |  |  | 0.0636684 | 0.0038231 | 0.00725015 | 0.000447475 | 0.110973 | True |
| bs_armature_minus20 | wave | 0.8 |  |  |  | 0.0814474 | 0.0211783 | 0.108507 | 0.0105185 | 0.00981437 | 0.000552941 | 0.0778719 | True |
| bs_armature_plus20 | heart | 1.2 | 0.0346838 | 0.0552098 | 0.00812004 |  |  | 0.0637003 | 0.00383417 | 0.00720602 | 0.000446052 | 0.110973 | True |
| bs_armature_plus20 | wave | 1.2 |  |  |  | 0.0829363 | 0.0233677 | 0.10905 | 0.0119145 | 0.00969097 | 0.000558846 | 0.0778604 | True |

## Central normalized local sensitivity

Definition: `(y_high - y_low) / |y_baseline| / (normalized_p_high - normalized_p_low)`. Sign is directional; magnitude ranks local sensitivity.

| physical_family | parameter | low_experiment | high_experiment | low_normalized_change | high_normalized_change | safety_effect | heart_left_ankle_excursion_normalized_sensitivity | heart_left_ankle_excursion_sweep_percent_change | heart_knee_excursion_normalized_sensitivity | heart_knee_excursion_sweep_percent_change | heart_waist_roll_excursion_normalized_sensitivity | heart_waist_roll_excursion_sweep_percent_change | wave_right_knee_excursion_normalized_sensitivity | wave_right_knee_excursion_sweep_percent_change | wave_ankle_excursion_normalized_sensitivity | wave_ankle_excursion_sweep_percent_change | base_pitch_excursion_normalized_sensitivity | base_pitch_excursion_sweep_percent_change | base_roll_excursion_normalized_sensitivity | base_roll_excursion_sweep_percent_change | foot_slipax_normalized_sensitivity | foot_slipax_sweep_percent_change | contact_penetrationax_normalized_sensitivity | contact_penetrationax_sweep_percent_change | arm_tracking_rmse_normalized_sensitivity | arm_tracking_rmse_sweep_percent_change |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ARMATURE | pitch_chain_armature_scale | bs_armature_minus20 | bs_armature_plus20 | -0.2 | 0.2 | PASS_ALL_HEART_WAVE | -0.0286911 | -1.14764 | 0.00224092 | 0.089637 | -0.00341831 | -0.136732 | 0.0453441 | 1.81376 | 0.274145 | 10.9658 | 0.0081919 | 0.327676 | 0.223626 | 8.94506 | -0.0245665 | -0.982662 | 0.0112267 | 0.449068 | -0.000152059 | -0.00608237 |

## Cross-motion assessment

| experiment_id | family | level | wave_knee_absolute_error_improvement_rad | heart_ankle_absolute_error_regression_rad | heart_waist_roll_absolute_error_regression_rad | mean_arm_tracking_rmse_ratio_to_baseline | heart_wave_safety_pass | shared_direction_criteria_pass | classification |
|---|---|---|---|---|---|---|---|---|---|
| bs_armature_minus20 | ARMATURE | minus | 0.000644648 | -0.000144783 | -3.23449e-06 | 1.00007 | True | False | PHYSICAL_SENSITIVITY_EXPERIMENT |
| bs_armature_plus20 | ARMATURE | plus | -0.000844308 | 0.000256203 | 7.87902e-06 | 1.00001 | True | False | PHYSICAL_SENSITIVITY_EXPERIMENT |

No value above is claimed to be a real X2 parameter.
