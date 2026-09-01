# Phase 3A-X heart validation

No fall; contacts `0`; minimum distance
`2.897 mm`; limit margin
`0.04795 rad`; persistent saturation `0.000%`.

## Arm tracking

| joint | RMSE | lag s | peak error | settling error |
| --- | --- | --- | --- | --- |
| `left_shoulder_roll_joint` | 0.1165 | 0.100 | 0.2510 | 0.0001 |
| `right_shoulder_roll_joint` | 0.1163 | 0.100 | 0.2508 | 0.0001 |
| `left_shoulder_yaw_joint` | 0.0834 | 0.100 | 0.1812 | 0.0000 |
| `right_shoulder_yaw_joint` | 0.0833 | 0.100 | 0.1841 | 0.0000 |
| `left_wrist_yaw_joint` | 0.1466 | 0.160 | 0.3099 | 0.0000 |
| `right_wrist_yaw_joint` | 0.1466 | 0.160 | 0.3089 | 0.0001 |
| `left_wrist_roll_joint` | 0.0973 | 0.160 | 0.2068 | 0.0016 |
| `right_wrist_roll_joint` | 0.0977 | 0.160 | 0.2065 | 0.0001 |

## Balance response

| joint | real exc | sim exc | ratio | RMSE | gate |
| --- | --- | --- | --- | --- | --- |
| `left_ankle_pitch_joint` | 0.0462 | 0.0077 | 0.167 | 0.0187 | FAIL |
| `right_ankle_pitch_joint` | 0.0320 | 0.0112 | 0.350 | 0.0102 | PASS |
| `left_hip_pitch_joint` | 0.0357 | 0.0156 | 0.438 | 0.0122 | PASS |
| `right_hip_pitch_joint` | 0.0410 | 0.0154 | 0.376 | 0.0159 | PASS |
| `left_knee_joint` | 0.0075 | 0.0380 | 5.081 | 0.0177 | LOW_SIGNAL |
| `right_knee_joint` | 0.0238 | 0.0377 | 1.584 | 0.0284 | PASS |
| `waist_pitch_joint` | 0.0323 | 0.0197 | 0.609 | 0.0129 | PASS |
| `waist_roll_joint` | 0.0102 | 0.0001 | 0.012 | 0.0033 | FAIL |

Hard safety/tracking pass; response gate `FAIL`.
