# Phase 3A-R right-wave validation

- stable/no fall: `True`
- safety threshold pass: `True`
- maximum pelvis/hip penetration: `0.486 mm`
- over-tolerance samples: `0`
- persistent saturation fraction: `0.00000`
- minimum limit margin: `0.04606 rad`

| active arm joint | real excursion | RMSE rad | lag s |
| --- | --- | --- | --- |
| `right_shoulder_roll_joint` | 0.3467 | 0.02659 | 0.080 |
| `right_shoulder_yaw_joint` | 0.5490 | 0.05707 | 0.080 |
| `right_wrist_yaw_joint` | 1.5562 | 0.14992 | 0.160 |

| balance joint | real exc | sim exc | ratio | relative RMSE |
| --- | --- | --- | --- | --- |
| `left_ankle_pitch_joint` | 0.03010 | 0.02053 | 0.682 | 0.01883 |
| `right_ankle_pitch_joint` | 0.02799 | 0.02590 | 0.925 | 0.01870 |
| `left_hip_pitch_joint` | 0.01400 | 0.03042 | 2.173 | 0.01337 |
| `right_hip_pitch_joint` | 0.01707 | 0.03041 | 1.782 | 0.01039 |
| `left_knee_joint` | 0.00729 | 0.08221 | 11.283 | 0.04610 |
| `right_knee_joint` | 0.01015 | 0.06650 | 6.549 | 0.04550 |
| `waist_pitch_joint` | 0.02804 | 0.02452 | 0.874 | 0.01223 |
| `waist_roll_joint` | 0.03863 | 0.02041 | 0.528 | 0.02115 |

Shoulder roll (`RMSE 0.02659 rad`,
lag `0.080 s`) and wrist yaw
(`RMSE 0.14992 rad`, lag
`0.160 s`) retain their independent
tracking improvement. Knee/hip excursion over-response and persistent near-tolerance
contact prevent balance acceptance.
