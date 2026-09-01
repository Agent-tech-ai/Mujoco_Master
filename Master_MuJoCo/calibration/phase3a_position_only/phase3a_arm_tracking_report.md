# Phase 3A Arm Tracking Calibration

Classification: **ACCEPTED_SIM_CONTROLLER_ALIGNMENT**  
Warning: **NOT HARDWARE CALIBRATION**. Gain scales are simulation-controller-only parameters and are not identified physical robot gains.

Selected candidate: shoulder `kp x8`, `kd x sqrt(8)`; wrist `kp x8`, `kd x sqrt(8)`. The original per-joint architecture is preserved; shoulder and wrist families were scanned independently.

## Free-base before/after

| joint | RMSE before | RMSE after | RMSE reduction | lag before s | lag after s | peak vel err before | peak vel err after | settling err after |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| left_shoulder_roll | 0.30136 | 0.11656 | 61.3% | 0.240 | 0.100 | 1.33369 | 0.81865 | 0.00002 |
| right_shoulder_roll | 0.30105 | 0.11637 | 61.3% | 0.240 | 0.100 | 1.36143 | 0.85657 | 0.00002 |
| left_shoulder_yaw | 0.21459 | 0.08339 | 61.1% | 0.240 | 0.100 | 0.92726 | 0.49958 | 0.00000 |
| right_shoulder_yaw | 0.21427 | 0.08337 | 61.1% | 0.240 | 0.100 | 0.94792 | 0.53998 | 0.00000 |
| left_wrist_yaw | 0.34151 | 0.14665 | 57.1% | 0.380 | 0.160 | 1.29511 | 0.86094 | 0.00000 |
| right_wrist_yaw | 0.34203 | 0.14665 | 57.1% | 0.380 | 0.160 | 1.20249 | 0.65719 | 0.00002 |
| left_wrist_roll | 0.22425 | 0.09737 | 56.6% | 0.380 | 0.160 | 0.85342 | 0.47503 | 0.00007 |
| right_wrist_roll | 0.22592 | 0.09775 | 56.7% | 0.380 | 0.160 | 0.83913 | 0.44245 | 0.00000 |

## Temporal train/validation check

Fit segments are motion onset and return. Validation segments are pre-roll, peak gesture, and post-roll.

| experiment | split | mean RMSE rad | mean MAE rad | metric rows |
| --- | --- | --- | --- | --- |
| free_baseline | fit | 0.29037 | 0.22413 | 16 |
| free_baseline | validation | 0.07485 | 0.04537 | 24 |
| free_final_candidate | fit | 0.10862 | 0.07566 | 16 |
| free_final_candidate | validation | 0.03658 | 0.02098 | 24 |

Only one complete heart capture is available, so this is temporal hold-out validation, not an independent-trajectory validation. The 8x optimum lies on the tested scan boundary and should be treated as a controller-alignment candidate, not a unique optimum or physical identification.

Plot: [arm_tracking_before_after.png](plots/arm_tracking_before_after.png)
