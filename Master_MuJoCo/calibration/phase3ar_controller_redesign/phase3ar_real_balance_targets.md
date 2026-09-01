# Phase 3A-R real balance output-response targets

These are `OUTPUT_RESPONSE_DESIGN_TARGET` measurements. They are not MC gain identification.

| joint | heart excursion rad | wave excursion rad | wave/heart | heart peak sign | wave peak sign |
| --- | ---: | ---: | ---: | ---: | ---: |
| `left_ankle_pitch_joint` | 0.04621 | 0.03010 | 0.651 | 1 | 1 |
| `right_ankle_pitch_joint` | 0.03202 | 0.02799 | 0.874 | 1 | 1 |
| `left_hip_pitch_joint` | 0.03566 | 0.01400 | 0.392 | -1 | -1 |
| `right_hip_pitch_joint` | 0.04103 | 0.01707 | 0.416 | -1 | -1 |
| `left_knee_joint` | 0.00748 | 0.00729 | 0.974 | -1 | 1 |
| `right_knee_joint` | 0.02378 | 0.01015 | 0.427 | 1 | 1 |
| `waist_pitch_joint` | 0.03234 | 0.02804 | 0.867 | 1 | 1 |
| `waist_roll_joint` | 0.01022 | 0.03863 | 3.779 | -1 | 1 |

The two motions do not exhibit one fixed ankle/knee/hip/waist excursion ratio. This supports channel-specific allocation and hard safety constraints rather than a single global gain.
