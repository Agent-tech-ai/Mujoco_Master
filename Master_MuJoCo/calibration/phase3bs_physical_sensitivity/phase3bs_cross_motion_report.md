# Phase 3B-S cross-motion consistency

Every physical perturbation ran on both independent datasets using the same frozen controller.

| experiment_id | family | level | wave_knee_absolute_error_improvement_rad | heart_ankle_absolute_error_regression_rad | heart_waist_roll_absolute_error_regression_rad | mean_arm_tracking_rmse_ratio_to_baseline | heart_wave_safety_pass | shared_direction_criteria_pass | classification |
|---|---|---|---|---|---|---|---|---|---|
| bs_mass_lower_minus08 | MASS_DISTRIBUTION | minus | -0.00485768 | 0.000874576 | -8.20622e-05 | 1.00025 | True | False | PHYSICAL_SENSITIVITY_EXPERIMENT |
| bs_mass_lower_plus08 | MASS_DISTRIBUTION | plus | 0.00590846 | -0.000433157 | 1.59363e-05 | 0.999835 | True | True | SHARED_PHYSICAL_SENSITIVITY_DIRECTION |
| bs_inertia_minus10 | ROTATIONAL_INERTIA | minus | 0.00072491 | 0.000137447 | -1.46766e-06 | 1.00005 | True | False | PHYSICAL_SENSITIVITY_EXPERIMENT |
| bs_inertia_plus10 | ROTATIONAL_INERTIA | plus | -0.00179346 | 2.41747e-05 | 4.75271e-06 | 1.00006 | True | False | PHYSICAL_SENSITIVITY_EXPERIMENT |
| bs_damping_005 | JOINT_DAMPING | low | -4.38674e-05 | -9.36464e-05 | 3.10771e-06 | 1.00004 | True | False | PHYSICAL_SENSITIVITY_EXPERIMENT |
| bs_damping_015 | JOINT_DAMPING | high | 0.00140609 | 0.00021864 | 8.70477e-06 | 1.00005 | True | False | PHYSICAL_SENSITIVITY_EXPERIMENT |
| bs_armature_minus20 | ARMATURE | minus | 0.000644648 | -0.000144783 | -3.23449e-06 | 1.00007 | True | False | PHYSICAL_SENSITIVITY_EXPERIMENT |
| bs_armature_plus20 | ARMATURE | plus | -0.000844308 | 0.000256203 | 7.87902e-06 | 1.00001 | True | False | PHYSICAL_SENSITIVITY_EXPERIMENT |
| bs_friction_minus10 | CONTACT_FRICTION | minus | 0.00295563 | 4.1259e-05 | -3.92883e-05 | 1 | True | False | PHYSICAL_SENSITIVITY_EXPERIMENT |
| bs_friction_plus10 | CONTACT_FRICTION | plus | -0.000792202 | 3.29271e-06 | 3.77452e-06 | 1.00004 | True | False | PHYSICAL_SENSITIVITY_EXPERIMENT |
| bs_compliance_stiffer10 | CONTACT_COMPLIANCE | minus | -0.000597803 | 0.000139981 | -2.24981e-05 | 1.00004 | True | False | PHYSICAL_SENSITIVITY_EXPERIMENT |
| bs_compliance_softer10 | CONTACT_COMPLIANCE | plus | -0.000447786 | 0.000237678 | 1.35734e-05 | 1.00002 | True | False | PHYSICAL_SENSITIVITY_EXPERIMENT |

Acceptance screen for a shared direction: Wave knee absolute error improves by at least 5%; Heart ankle and waist-roll errors do not regress beyond 10% (with a 0.001 rad numerical floor); mean active-arm RMSE stays within 5%; Heart/Wave safety both pass.

This screen selects directions for future validation only. It does not identify real hardware values and does not automatically promote a sensitivity experiment to a calibrated candidate.
