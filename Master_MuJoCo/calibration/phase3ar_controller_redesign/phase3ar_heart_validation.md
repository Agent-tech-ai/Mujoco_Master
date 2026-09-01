# Phase 3A-R heart validation

- stable/no fall: `True`
- safety pass: `True`
- pelvis/hip over-tolerance samples: `0`
- persistent saturation fraction: `0.00000`
- minimum limit margin: `0.04795 rad`

| active arm joint | real excursion | RMSE rad | lag s |
| --- | --- | --- | --- |
| `left_shoulder_roll_joint` | 2.2051 | 0.11653 | 0.100 |
| `right_shoulder_roll_joint` | 2.2026 | 0.11634 | 0.100 |
| `left_shoulder_yaw_joint` | 1.5808 | 0.08335 | 0.100 |
| `right_shoulder_yaw_joint` | 1.5730 | 0.08333 | 0.100 |
| `left_wrist_yaw_joint` | 1.6241 | 0.14659 | 0.160 |
| `right_wrist_yaw_joint` | 1.6314 | 0.14660 | 0.160 |
| `left_wrist_roll_joint` | 1.0778 | 0.09734 | 0.160 |
| `right_wrist_roll_joint` | 1.0910 | 0.09771 | 0.160 |

| balance joint | real exc | sim exc | ratio | relative RMSE |
| --- | --- | --- | --- | --- |
| `left_ankle_pitch_joint` | 0.04620 | 0.00789 | 0.171 | 0.02390 |
| `right_ankle_pitch_joint` | 0.03202 | 0.00815 | 0.255 | 0.01565 |
| `left_hip_pitch_joint` | 0.03566 | 0.01780 | 0.499 | 0.00987 |
| `right_hip_pitch_joint` | 0.04103 | 0.01859 | 0.453 | 0.01308 |
| `left_knee_joint` | 0.00748 | 0.03851 | 5.151 | 0.03219 |
| `right_knee_joint` | 0.02378 | 0.03851 | 1.620 | 0.04159 |
| `waist_pitch_joint` | 0.03234 | 0.01776 | 0.549 | 0.01292 |
| `waist_roll_joint` | 0.01018 | 0.00013 | 0.012 | 0.00321 |

Arm tracking remains consistent with the Phase 3A candidate. Balance does not
fully generalize: ankle under-response and left-knee over-response remain.
