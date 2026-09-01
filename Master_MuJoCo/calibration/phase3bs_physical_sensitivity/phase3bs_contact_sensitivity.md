# Phase 3B-S — Contact/friction/compliance sensitivity

The two contact families were tested separately; collision topology remained unchanged in every formal run.

| experiment_id | dataset | family | parameter_value | wave_right_knee_excursion_rad | wave_ankle_excursion_rad | base_pitch_excursion_rad | base_roll_excursion_rad | foot_slip_max_m | contact_penetration_max_m | arm_tracking_rmse_rad | safety_pass |
|---|---|---|---|---|---|---|---|---|---|---|---|
| bs_friction_minus10 | heart | CONTACT_FRICTION | 0.9 |  |  | 0.0636947 | 0.00386414 | 0.00712639 | 0.000355184 | 0.110973 | True |
| bs_friction_minus10 | wave | CONTACT_FRICTION | 0.9 | 0.0791364 | 0.0196523 | 0.110961 | 0.0112129 | 0.00972028 | 0.000454391 | 0.0778593 | True |
| bs_friction_plus10 | heart | CONTACT_FRICTION | 1.1 |  |  | 0.0637993 | 0.00382444 | 0.00731784 | 0.000535975 | 0.110973 | True |
| bs_friction_plus10 | wave | CONTACT_FRICTION | 1.1 | 0.0828842 | 0.0207309 | 0.112629 | 0.0117261 | 0.00986813 | 0.000661343 | 0.0778656 | True |
| bs_compliance_stiffer10 | heart | CONTACT_COMPLIANCE | 0.9 |  |  | 0.0635057 | 0.00384603 | 0.00704695 | 0.000384038 | 0.110973 | True |
| bs_compliance_stiffer10 | wave | CONTACT_COMPLIANCE | 0.9 | 0.0826898 | 0.0212807 | 0.11138 | 0.0107082 | 0.00958224 | 0.000503261 | 0.0778655 | True |
| bs_compliance_softer10 | heart | CONTACT_COMPLIANCE | 1.1 |  |  | 0.0638117 | 0.00383074 | 0.00739677 | 0.000503797 | 0.110973 | True |
| bs_compliance_softer10 | wave | CONTACT_COMPLIANCE | 1.1 | 0.0825398 | 0.0217998 | 0.110579 | 0.013537 | 0.00993294 | 0.000618647 | 0.0778631 | True |

## Normalized local sensitivity

| physical_family | parameter | low_experiment | high_experiment | low_normalized_change | high_normalized_change | safety_effect | heart_left_ankle_excursion_normalized_sensitivity | heart_left_ankle_excursion_sweep_percent_change | heart_knee_excursion_normalized_sensitivity | heart_knee_excursion_sweep_percent_change | heart_waist_roll_excursion_normalized_sensitivity | heart_waist_roll_excursion_sweep_percent_change | wave_right_knee_excursion_normalized_sensitivity | wave_right_knee_excursion_sweep_percent_change | wave_ankle_excursion_normalized_sensitivity | wave_ankle_excursion_sweep_percent_change | base_pitch_excursion_normalized_sensitivity | base_pitch_excursion_sweep_percent_change | base_roll_excursion_normalized_sensitivity | base_roll_excursion_sweep_percent_change | foot_slipax_normalized_sensitivity | foot_slipax_sweep_percent_change | contact_penetrationax_normalized_sensitivity | contact_penetrationax_sweep_percent_change | arm_tracking_rmse_normalized_sensitivity | arm_tracking_rmse_sweep_percent_change |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| CONTACT_FRICTION | floor_and_foot_friction_scale | bs_friction_minus10 | bs_friction_plus10 | -0.1 | 0.1 | PASS_ALL_HEART_WAVE | 0.00543307 | 0.108661 | -0.000818873 | -0.0163775 | -0.0264907 | -0.529814 | 0.22827 | 4.5654 | 0.270109 | 5.40218 | 0.0505376 | 1.01075 | 0.150481 | 3.00961 | 0.0995094 | 1.99019 | 1.9423 | 38.8459 | 0.000164003 | 0.00328007 |
| CONTACT_COMPLIANCE | floor_and_foot_solref_timeconst_scale | bs_compliance_stiffer10 | bs_compliance_softer10 | -0.1 | 0.1 | PASS_ALL_HEART_WAVE | -0.0139806 | -0.279612 | 0.0152373 | 0.304746 | -0.0221898 | -0.443797 | -0.00913715 | -0.182743 | 0.129993 | 2.59985 | -0.0140963 | -0.281925 | 0.894312 | 17.8862 | 0.205443 | 4.10886 | 1.1779 | 23.558 | -6.52001e-05 | -0.001304 |

## Cross-motion assessment

| experiment_id | family | level | wave_knee_absolute_error_improvement_rad | heart_ankle_absolute_error_regression_rad | heart_waist_roll_absolute_error_regression_rad | mean_arm_tracking_rmse_ratio_to_baseline | heart_wave_safety_pass | shared_direction_criteria_pass | classification |
|---|---|---|---|---|---|---|---|---|---|
| bs_friction_minus10 | CONTACT_FRICTION | minus | 0.00295563 | 4.1259e-05 | -3.92883e-05 | 1 | True | False | PHYSICAL_SENSITIVITY_EXPERIMENT |
| bs_friction_plus10 | CONTACT_FRICTION | plus | -0.000792202 | 3.29271e-06 | 3.77452e-06 | 1.00004 | True | False | PHYSICAL_SENSITIVITY_EXPERIMENT |
| bs_compliance_stiffer10 | CONTACT_COMPLIANCE | minus | -0.000597803 | 0.000139981 | -2.24981e-05 | 1.00004 | True | False | PHYSICAL_SENSITIVITY_EXPERIMENT |
| bs_compliance_softer10 | CONTACT_COMPLIANCE | plus | -0.000447786 | 0.000237678 | 1.35734e-05 | 1.00002 | True | False | PHYSICAL_SENSITIVITY_EXPERIMENT |

These are local contact-model sensitivities, not identified floor/foot parameters.
