# Phase 3A-X wave validation

No fall; contacts `0`; minimum distance
`1.134 mm` (`0.634 mm` above numerical tolerance);
limit margin `0.04623 rad`; persistent saturation `0.000%`.

## Arm tracking

| joint | RMSE | lag s | peak error | settling error |
| --- | --- | --- | --- | --- |
| `right_shoulder_roll_joint` | 0.0266 | 0.080 | 0.0673 | 0.0026 |
| `right_shoulder_yaw_joint` | 0.0571 | 0.080 | 0.1259 | 0.0001 |
| `right_wrist_yaw_joint` | 0.1499 | 0.160 | 0.2536 | 0.0491 |

## Balance response

| joint | real exc | sim exc | ratio | RMSE | gate |
| --- | --- | --- | --- | --- | --- |
| `left_ankle_pitch_joint` | 0.0301 | 0.0241 | 0.801 | 0.0077 | PASS |
| `right_ankle_pitch_joint` | 0.0280 | 0.0200 | 0.716 | 0.0074 | PASS |
| `left_hip_pitch_joint` | 0.0140 | 0.0523 | 3.739 | 0.0243 | PASS |
| `right_hip_pitch_joint` | 0.0171 | 0.0436 | 2.554 | 0.0173 | PASS |
| `left_knee_joint` | 0.0073 | 0.0707 | 9.700 | 0.0417 | LOW_SIGNAL |
| `right_knee_joint` | 0.0102 | 0.0843 | 8.298 | 0.0448 | FAIL |
| `waist_pitch_joint` | 0.0280 | 0.0290 | 1.035 | 0.0121 | PASS |
| `waist_roll_joint` | 0.0386 | 0.0233 | 0.602 | 0.0214 | PASS |

Contact safety and arm tracking pass; response gate `FAIL`.
