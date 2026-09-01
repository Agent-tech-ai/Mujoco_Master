# Phase 2G simulation controller alignment experiments

Every row is **NOT HARDWARE CALIBRATION**. No mass, inertia, friction, gear, torque limit, MJCF dynamics, or hardware mapping was changed; each experiment changes one simulation controller/reference category.

| experiment | single changed category | mean arm RMSE | median arm lag | mean balance RMSE | stable | status |
| --- | --- | --- | --- | --- | --- | --- |
| free_baseline | none | 0.19555 | 0.24 | 0.02113 | True | NOT HARDWARE CALIBRATION |
| fixed_base_baseline | base constraint | 0.19546 | 0.24 | UNKNOWN | True | NOT HARDWARE CALIBRATION |
| fixed_base_50hz_zoh | reference interpolation | 0.20182 | 0.26 | UNKNOWN | True | NOT HARDWARE CALIBRATION |
| free_reference_advance_030 | reference timing | 0.06876 | -0.06 | 0.02179 | True | NOT HARDWARE CALIBRATION |
| free_balance_gain_scale_060 | simulation balance gains | 0.19552 | 0.24 | 0.01715 | True | NOT HARDWARE CALIBRATION |
| free_equilibrium_target_compensation | standing equilibrium targets | 0.19555 | 0.24 | 0.02160 | True | NOT HARDWARE CALIBRATION |

- `free_reference_advance_030`: useful timing candidate, but one common advance over-corrects shoulders and under-corrects wrists.
- `free_balance_gain_scale_060`: reduces excessive ankle/hip/torso pitch response, but under-shoots left ankle excursion and does not resolve knee mismatch.
- `free_equilibrium_target_compensation`: improves knee absolute equilibrium but worsens ankle equilibrium; it exposes controller equilibrium, not robot zero.
- No candidates were combined or promoted to the current model/controller.
