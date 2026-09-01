# Phase 3B-S — Joint damping sensitivity

Classification: **PHYSICAL_SENSITIVITY_EXPERIMENT — NOT HARDWARE CALIBRATION**.

Only `pitch_chain_damping_Nm_s_rad` changed. Controller, source MJCF, gear, torque/force/ctrl limits, and all other physical families remained frozen.

## Heart/Wave runs

| experiment_id | dataset | parameter_value | heart_left_ankle_excursion_rad | heart_knee_excursion_rad | heart_waist_roll_excursion_rad | wave_right_knee_excursion_rad | wave_ankle_excursion_rad | base_pitch_excursion_rad | base_roll_excursion_rad | foot_slip_max_m | contact_penetration_max_m | arm_tracking_rmse_rad | safety_pass |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| bs_damping_005 | heart | 0.05 | 0.0350337 | 0.0551862 | 0.00812481 |  |  | 0.0637173 | 0.00390287 | 0.00723126 | 0.00044834 | 0.110973 | True |
| bs_damping_005 | wave | 0.05 |  |  |  | 0.0821359 | 0.0208727 | 0.110255 | 0.0114301 | 0.00978225 | 0.00054916 | 0.0778672 | True |
| bs_damping_015 | heart | 0.15 | 0.0347214 | 0.0552746 | 0.00811922 |  |  | 0.0639242 | 0.00384047 | 0.007227 | 0.000446002 | 0.110973 | True |
| bs_damping_015 | wave | 0.15 |  |  |  | 0.0806859 | 0.0208389 | 0.110286 | 0.0107664 | 0.00975454 | 0.000555181 | 0.0778675 | True |

## Central normalized local sensitivity

Definition: `(y_high - y_low) / |y_baseline| / (normalized_p_high - normalized_p_low)`. Sign is directional; magnitude ranks local sensitivity.

| physical_family | parameter | low_experiment | high_experiment | low_normalized_change | high_normalized_change | safety_effect | heart_left_ankle_excursion_normalized_sensitivity | heart_left_ankle_excursion_sweep_percent_change | heart_knee_excursion_normalized_sensitivity | heart_knee_excursion_sweep_percent_change | heart_waist_roll_excursion_normalized_sensitivity | heart_waist_roll_excursion_sweep_percent_change | wave_right_knee_excursion_normalized_sensitivity | wave_right_knee_excursion_sweep_percent_change | wave_ankle_excursion_normalized_sensitivity | wave_ankle_excursion_sweep_percent_change | base_pitch_excursion_normalized_sensitivity | base_pitch_excursion_sweep_percent_change | base_roll_excursion_normalized_sensitivity | base_roll_excursion_sweep_percent_change | foot_slipax_normalized_sensitivity | foot_slipax_sweep_percent_change | contact_penetrationax_normalized_sensitivity | contact_penetrationax_sweep_percent_change | arm_tracking_rmse_normalized_sensitivity | arm_tracking_rmse_sweep_percent_change |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| JOINT_DAMPING | pitch_chain_damping_Nm_s_rad | bs_damping_005 | bs_damping_015 | 0.5 | 1.5 | PASS_ALL_HEART_WAVE | -0.00893777 | -0.893777 | 0.00160254 | 0.160254 | -0.000688621 | -0.0688621 | -0.0176626 | -1.76626 | -0.00169252 | -0.169252 | 0.00135829 | 0.135829 | -0.0461613 | -4.61613 | -0.00187503 | -0.187503 | 0.00369091 | 0.369091 | 2.74211e-06 | 0.000274211 |

## Cross-motion assessment

| experiment_id | family | level | wave_knee_absolute_error_improvement_rad | heart_ankle_absolute_error_regression_rad | heart_waist_roll_absolute_error_regression_rad | mean_arm_tracking_rmse_ratio_to_baseline | heart_wave_safety_pass | shared_direction_criteria_pass | classification |
|---|---|---|---|---|---|---|---|---|---|
| bs_damping_005 | JOINT_DAMPING | low | -4.38674e-05 | -9.36464e-05 | 3.10771e-06 | 1.00004 | True | False | PHYSICAL_SENSITIVITY_EXPERIMENT |
| bs_damping_015 | JOINT_DAMPING | high | 0.00140609 | 0.00021864 | 8.70477e-06 | 1.00005 | True | False | PHYSICAL_SENSITIVITY_EXPERIMENT |

No value above is claimed to be a real X2 parameter.
