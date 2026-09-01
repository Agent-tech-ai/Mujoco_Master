# Phase 2E measured balance-response candidates

The dominant arm motion is accompanied by the following leg/waist motion. `BALANCE_COMPENSATION_CANDIDATE` means timing and magnitude are consistent with compensation; it does **not** identify or reconstruct the MC control law.

| joint | classification | excursion rad | peak \|dq\| rad/s | onset s | end s | recovery s | peak lagged corr | lag s |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| left_ankle_pitch_joint | BALANCE_COMPENSATION_CANDIDATE | 0.04621 | 0.15236 | 0.290 | 3.870 | 0.010 | 0.485 | 0.500 |
| left_ankle_roll_joint | STATIC | 0.00403 | 0.04273 | 1.110 | 3.650 | 0.010 | 0.303 | -0.240 |
| left_hip_pitch_joint | BALANCE_COMPENSATION_CANDIDATE | 0.03566 | 0.15263 | 0.390 | 4.910 | 0.010 | 0.513 | 0.580 |
| left_hip_roll_joint | STATIC | 0.00345 | 0.02728 | 3.090 | 3.490 | 0.010 | 0.227 | 0.480 |
| left_hip_yaw_joint | STATIC | 0.00019 | 0.00611 | UNKNOWN | UNKNOWN | 0.010 | UNKNOWN | UNKNOWN |
| left_knee_joint | BALANCE_COMPENSATION_CANDIDATE | 0.00748 | 0.06302 | 0.910 | 3.810 | 0.010 | 0.398 | 0.060 |
| right_ankle_pitch_joint | BALANCE_COMPENSATION_CANDIDATE | 0.03202 | 0.12766 | 0.950 | 3.930 | 0.010 | 0.419 | 0.460 |
| right_ankle_roll_joint | STATIC | 0.00249 | 0.04176 | 0.890 | 3.650 | 0.010 | 0.288 | -0.140 |
| right_hip_pitch_joint | BALANCE_COMPENSATION_CANDIDATE | 0.04103 | 0.13784 | 0.470 | 5.650 | 0.010 | 0.589 | 0.520 |
| right_hip_roll_joint | STATIC | 0.00269 | 0.02849 | 3.110 | 3.450 | 0.010 | 0.249 | 0.480 |
| right_hip_yaw_joint | STATIC | 0.00422 | 0.02913 | 1.430 | 3.590 | 0.010 | 0.396 | 0.600 |
| right_knee_joint | BALANCE_COMPENSATION_CANDIDATE | 0.02378 | 0.09405 | 0.710 | 4.050 | 0.010 | 0.579 | 0.300 |
| waist_pitch_joint | BALANCE_COMPENSATION_CANDIDATE | 0.03234 | 0.13240 | 0.350 | 4.270 | 0.010 | 0.567 | 0.380 |
| waist_roll_joint | BALANCE_COMPENSATION_CANDIDATE | 0.01022 | 0.06495 | 0.810 | 3.950 | 0.010 | 0.489 | 0.100 |
| waist_yaw_joint | STATIC | 0.00211 | 0.01653 | UNKNOWN | UNKNOWN | 0.010 | 0.116 | -0.040 |

## Relative IMU response

| IMU | peak \|relative roll\| deg | peak \|relative pitch\| deg | peak gyro norm rad/s | peak accel norm m/s² |
| --- | --- | --- | --- | --- |
| chest | 0.361 | 0.782 | 0.1032 | 10.0262 |
| torso | 0.186 | 1.517 | 0.1117 | 10.0520 |

The IMU frame relationship remains `UNKNOWN`; only relative changes, norms, shapes, and timing are used. No absolute quaternion component comparison or MC-law recovery is claimed.
