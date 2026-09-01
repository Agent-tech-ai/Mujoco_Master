# Phase 3A Balance Position-Response Alignment

Selected simulation balance gain scale: **0.70**. No mass, inertia, physical friction, gear, or torque/force limit was changed.

| joint | real excursion | base sim excursion | base ratio | candidate sim excursion | candidate ratio | base RMSE | candidate RMSE | candidate lag s | candidate recovery s |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| left_ankle_pitch | 0.04621 | 0.07313 | 1.583 | 0.02976 | 0.644 | 0.02748 | 0.02347 | -1.000 | 2.250 |
| right_ankle_pitch | 0.03202 | 0.07219 | 2.254 | 0.02742 | 0.856 | 0.02401 | 0.01598 | -0.640 | 0.790 |
| left_hip_pitch | 0.03566 | 0.04824 | 1.353 | 0.03183 | 0.893 | 0.02139 | 0.02013 | -0.920 | 3.010 |
| right_hip_pitch | 0.04103 | 0.05246 | 1.278 | 0.03310 | 0.807 | 0.02729 | 0.02370 | -1.000 | 0.010 |
| left_knee | 0.00748 | 0.08886 | 11.883 | 0.06017 | 8.046 | 0.02478 | 0.02082 | 0.540 | 0.430 |
| right_knee | 0.02378 | 0.08162 | 3.433 | 0.05748 | 2.417 | 0.02499 | 0.02297 | 0.420 | 0.510 |
| waist_pitch | 0.03234 | 0.04074 | 1.260 | 0.02475 | 0.765 | 0.01593 | 0.01338 | -0.380 | 0.010 |
| waist_roll | 0.01021 | 0.00041 | 0.040 | 0.00015 | 0.014 | 0.00321 | 0.00323 | 1.000 | 0.010 |

The original left ankle pitch discrepancy changes from real/sim `0.04621 / 0.07313 rad` (ratio **1.583**) to `0.04621 / 0.02976 rad` (ratio **0.644**). The overshoot is removed, but the candidate now undershoots the real excursion; this is an improvement in absolute excursion error, not a complete match. Right ankle ratio improves from **2.254** to **0.856**.

## Relative IMU auxiliary metrics

IMU transform is still PARTIAL. These metrics use only pre-roll-centered roll/pitch motion and gyro-norm shape; no absolute quaternion/yaw fitting is performed.

| experiment | real IMU | quantity | excursion ratio | relative RMSE | lag s | shape corr |
| --- | --- | --- | --- | --- | --- | --- |
| free_baseline | chest | relative_roll | 0.600 | 0.00330 | -0.920 | 0.457 |
| free_baseline | chest | relative_pitch | 2.890 | 0.02190 | 0.460 | 0.893 |
| free_baseline | chest | gyro_norm | 1.489 | 0.06454 | 0.500 | 0.496 |
| free_baseline | torso | relative_roll | 1.735 | 0.00222 | 1.000 | 0.223 |
| free_baseline | torso | relative_pitch | 1.523 | 0.02215 | 0.340 | 0.676 |
| free_baseline | torso | gyro_norm | 1.368 | 0.06885 | 0.360 | 0.643 |
| free_final_candidate | chest | relative_roll | 0.210 | 0.00240 | 0.660 | 0.577 |
| free_final_candidate | chest | relative_pitch | 2.796 | 0.02431 | 0.540 | 0.517 |
| free_final_candidate | chest | gyro_norm | 1.364 | 0.04457 | 0.460 | 0.513 |
| free_final_candidate | torso | relative_roll | 0.608 | 0.00193 | -1.000 | 0.096 |
| free_final_candidate | torso | relative_pitch | 1.473 | 0.02457 | 0.400 | 0.535 |
| free_final_candidate | torso | gyro_norm | 1.253 | 0.04490 | 0.260 | 0.422 |

## Base/contact safeguards

- free replay both-feet contact: 0.9987 -> 0.9994
- max foot-slip proxy (left/right): `0.00713 / 0.00785 m`
- max relative base tilt during replay (roll/pitch): `0.605 / 4.072 deg`
- self-collision and non-foot ground-contact samples: `0 / 0`

Plot: [balance_response_before_after.png](plots/balance_response_before_after.png)
