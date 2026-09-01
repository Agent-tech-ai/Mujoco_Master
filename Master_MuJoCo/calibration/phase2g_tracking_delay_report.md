# Phase 2G replay tracking-delay decomposition

| experiment | joint | lag (s) | RMSE (rad) | max saturation fraction |
| --- | --- | --- | --- | --- |
| free_baseline | left_shoulder_roll_joint | 0.24 | 0.30136 | 0.189 |
| free_baseline | left_wrist_yaw_joint | 0.38 | 0.34151 | 0.018 |
| free_baseline | right_shoulder_roll_joint | 0.24 | 0.30105 | 0.189 |
| free_baseline | right_wrist_yaw_joint | 0.38 | 0.34203 | 0.018 |
| fixed_base_baseline | left_shoulder_roll_joint | 0.24 | 0.30148 | 0.189 |
| fixed_base_baseline | left_wrist_yaw_joint | 0.38 | 0.34145 | 0.018 |
| fixed_base_baseline | right_shoulder_roll_joint | 0.24 | 0.30104 | 0.189 |
| fixed_base_baseline | right_wrist_yaw_joint | 0.38 | 0.34197 | 0.018 |
| fixed_base_50hz_zoh | left_shoulder_roll_joint | 0.26 | 0.31375 | 0.180 |
| fixed_base_50hz_zoh | left_wrist_yaw_joint | 0.40 | 0.34944 | 0.014 |
| fixed_base_50hz_zoh | right_shoulder_roll_joint | 0.26 | 0.31329 | 0.181 |
| fixed_base_50hz_zoh | right_wrist_yaw_joint | 0.40 | 0.34997 | 0.014 |
| free_reference_advance_030 | left_shoulder_roll_joint | -0.06 | 0.09224 | 0.189 |
| free_reference_advance_030 | left_wrist_yaw_joint | 0.08 | 0.13393 | 0.018 |
| free_reference_advance_030 | right_shoulder_roll_joint | -0.06 | 0.09191 | 0.189 |
| free_reference_advance_030 | right_wrist_yaw_joint | 0.08 | 0.13390 | 0.018 |

## Finding

Fixed-base reproduces the free-base lag: shoulder roll remains about 0.24 s and wrist yaw about 0.38 s, with no difference at the 20 ms analysis resolution. The 50 Hz zero-order-hold test adds only one 20 ms sample (0.26/0.40 s), as expected for sample-and-hold, and does not explain the original 0.24/0.38 s lag. This prioritizes the simulation controller/actuator-following pipeline over free-base balance coupling or reference update rate.

The baseline uses piecewise-linear 50 Hz data evaluated every 1 ms physics/control step. `SimulationStabilityController` has no explicit command filter or velocity limiter. Control saturation is not active (all tested fractions remain well below 1). The remaining mechanism is the simulated joint PD/inertia/damping/friction response; wrist kp=12 and shoulder kp=38 are simulation controller settings, not hardware gains.

A common 0.30 s reference schedule advance changes lag against the original real timeline to approximately -0.06 s for shoulder roll and +0.08 s for wrist yaw and substantially reduces RMSE, but cannot align both joint families simultaneously. It is a `SIM_CONTROLLER_ALIGNMENT_CANDIDATE`, not a physical delay estimate and not hardware calibration.
