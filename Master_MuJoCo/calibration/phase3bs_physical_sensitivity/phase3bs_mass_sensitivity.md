# Phase 3B-S — Mass distribution sensitivity

Classification: **PHYSICAL_SENSITIVITY_EXPERIMENT — NOT HARDWARE CALIBRATION**.

Only `lower_limb_mass_scale_total_mass_preserved` changed. Controller, source MJCF, gear, torque/force/ctrl limits, and all other physical families remained frozen.

## Heart/Wave runs

| experiment_id | dataset | parameter_value | heart_left_ankle_excursion_rad | heart_knee_excursion_rad | heart_waist_roll_excursion_rad | wave_right_knee_excursion_rad | wave_ankle_excursion_rad | base_pitch_excursion_rad | base_roll_excursion_rad | foot_slip_max_m | contact_penetration_max_m | arm_tracking_rmse_rad | safety_pass |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| bs_mass_lower_minus08 | heart | 0.92 | 0.0340654 | 0.0586745 | 0.00820998 |  |  | 0.0678679 | 0.00398944 | 0.00723882 | 0.000445495 | 0.110983 | True |
| bs_mass_lower_minus08 | wave | 0.92 |  |  |  | 0.0869497 | 0.0226012 | 0.118517 | 0.0092994 | 0.00984738 | 0.000574608 | 0.0778956 | True |
| bs_mass_lower_plus08 | heart | 1.08 | 0.0353732 | 0.0509179 | 0.00811199 |  |  | 0.0585506 | 0.00387778 | 0.00721887 | 0.000451352 | 0.110964 | True |
| bs_mass_lower_plus08 | wave | 1.08 |  |  |  | 0.0761836 | 0.0193707 | 0.105653 | 0.012493 | 0.00973679 | 0.000554782 | 0.0778368 | True |

## Central normalized local sensitivity

Definition: `(y_high - y_low) / |y_baseline| / (normalized_p_high - normalized_p_low)`. Sign is directional; magnitude ranks local sensitivity.

| physical_family | parameter | low_experiment | high_experiment | low_normalized_change | high_normalized_change | safety_effect | heart_left_ankle_excursion_normalized_sensitivity | heart_left_ankle_excursion_sweep_percent_change | heart_knee_excursion_normalized_sensitivity | heart_knee_excursion_sweep_percent_change | heart_waist_roll_excursion_normalized_sensitivity | heart_waist_roll_excursion_sweep_percent_change | wave_right_knee_excursion_normalized_sensitivity | wave_right_knee_excursion_sweep_percent_change | wave_ankle_excursion_normalized_sensitivity | wave_ankle_excursion_sweep_percent_change | base_pitch_excursion_normalized_sensitivity | base_pitch_excursion_sweep_percent_change | base_roll_excursion_normalized_sensitivity | base_roll_excursion_sweep_percent_change | foot_slipax_normalized_sensitivity | foot_slipax_sweep_percent_change | contact_penetrationax_normalized_sensitivity | contact_penetrationax_sweep_percent_change | arm_tracking_rmse_normalized_sensitivity | arm_tracking_rmse_sweep_percent_change |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| MASS_DISTRIBUTION | lower_limb_mass_scale_total_mass_preserved | bs_mass_lower_minus08 | bs_mass_lower_plus08 | -0.08 | 0.08 | PASS_ALL_HEART_WAVE | 0.233925 | 3.74279 | -0.878534 | -14.0565 | -0.0753564 | -1.2057 | -0.81967 | -13.1147 | -1.01126 | -16.1801 | -0.790491 | -12.6479 | 1.22452 | 19.5924 | -0.0478571 | -0.765714 | -0.0874712 | -1.39954 | -0.00256958 | -0.0411133 |

## Cross-motion assessment

| experiment_id | family | level | wave_knee_absolute_error_improvement_rad | heart_ankle_absolute_error_regression_rad | heart_waist_roll_absolute_error_regression_rad | mean_arm_tracking_rmse_ratio_to_baseline | heart_wave_safety_pass | shared_direction_criteria_pass | classification |
|---|---|---|---|---|---|---|---|---|---|
| bs_mass_lower_minus08 | MASS_DISTRIBUTION | minus | -0.00485768 | 0.000874576 | -8.20622e-05 | 1.00025 | True | False | PHYSICAL_SENSITIVITY_EXPERIMENT |
| bs_mass_lower_plus08 | MASS_DISTRIBUTION | plus | 0.00590846 | -0.000433157 | 1.59363e-05 | 0.999835 | True | True | SHARED_PHYSICAL_SENSITIVITY_DIRECTION |

No value above is claimed to be a real X2 parameter.
