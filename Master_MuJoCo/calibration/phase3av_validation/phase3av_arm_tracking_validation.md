# Phase 3A-V arm tracking blind validation

| joint | real excursion | legacy RMSE | candidate RMSE | legacy lag s | candidate lag s | legacy overshoot | candidate overshoot |
| --- | --- | --- | --- | --- | --- | --- | --- |
| left_shoulder_pitch_joint | 0.00997 | 0.15771 | 0.00122 | -1.000 | 0.100 | 0.17995 | 0.00000 |
| right_elbow_joint | 0.57811 | 0.48135 | 0.07739 | 0.260 | 0.220 | 0.58215 | 0.00000 |
| right_shoulder_pitch_joint | 2.94635 | 1.58225 | 0.43441 | 0.300 | 0.240 | 0.26019 | 0.00000 |
| right_shoulder_roll_joint | 0.34667 | 0.10414 | 0.02656 | 0.840 | 0.080 | 0.00826 | 0.00000 |
| right_shoulder_yaw_joint | 0.54973 | 0.10326 | 0.05707 | 0.140 | 0.080 | 0.00000 | 0.00000 |
| right_wrist_yaw_joint | 1.55620 | 0.38912 | 0.14992 | 0.420 | 0.160 | 0.12031 | 0.00000 |

- shoulder-roll bandwidth: **GENERALIZES** — All 1 sufficiently excited mapped joints pass lag/RMSE/overshoot checks.
- shoulder-yaw bandwidth: **GENERALIZES** — All 1 sufficiently excited mapped joints pass lag/RMSE/overshoot checks.
- wrist-yaw bandwidth: **GENERALIZES** — All 1 sufficiently excited mapped joints pass lag/RMSE/overshoot checks.
- wrist-roll bandwidth: **INSUFFICIENT_EXCITATION** — No joint in this family exceeded 0.02 rad measured excursion.
- overall candidate vs legacy arm response: **IMPROVES**

The input is `MEASURED_REAL_TRAJECTORY`, not an observable MC internal command. No fixed global time advance or parameter optimization is used.
