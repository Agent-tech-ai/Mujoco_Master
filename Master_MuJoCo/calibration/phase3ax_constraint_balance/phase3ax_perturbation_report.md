# Phase 3A-X perturbation robustness

| dataset | perturbation | min distance mm | no fall | contact | limit | saturation |
| --- | --- | --- | --- | --- | --- | --- |
| heart | roll_plus_0p25 | 2.861 | True | True | True | True |
| heart | pitch_minus_0p25 | 2.896 | True | True | True | True |
| wave | roll_plus_0p25 | 0.984 | True | True | True | True |
| wave | roll_minus_0p25 | 0.819 | True | True | True | True |
| wave | pitch_plus_0p25 | 1.165 | True | True | True | True |
| wave | pitch_minus_0p25 | 1.109 | True | True | True | True |
| wave | left_hip_roll_plus_0p25 | 1.178 | True | True | True | True |
| wave | left_hip_roll_minus_0p25 | 1.114 | True | True | True | True |

`8/8` passed. Worst distance `0.819 mm`, above the `0.750 mm` hard zone.
These tests establish local, not global, robustness.
