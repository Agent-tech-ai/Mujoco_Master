# Phase 3A-R joint-limit analysis

| experiment | joint | first s | target | actual q | MJCF range | real ref legal | min margin | classification |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| current_07 | `left_ankle_pitch_joint` | 8.260 | -0.24188 | 0.46788 | [-0.803, 0.454] | True | -0.04501 | CONTROLLER_LIMIT_MANAGEMENT_FAILURE |
| current_07 | `left_ankle_roll_joint` | 3.400 | 0.01218 | 0.26195 | [-0.262, 0.262] | True | -0.02374 | CONTROLLER_LIMIT_MANAGEMENT_FAILURE |
| current_07 | `left_shoulder_roll_joint` | 8.300 | -0.01467 | -0.07169 | [-0.061, 3.046] | True | -0.01060 | CONTROLLER_LIMIT_MANAGEMENT_FAILURE |
| current_07 | `left_wrist_roll_joint` | 8.400 | -0.00439 | -1.55204 | [-1.510, 0.724] | True | -0.04233 | CONTROLLER_LIMIT_MANAGEMENT_FAILURE |
| current_07 | `right_ankle_pitch_joint` | 7.740 | -0.30554 | 0.46035 | [-0.803, 0.454] | True | -0.01874 | CONTROLLER_LIMIT_MANAGEMENT_FAILURE |
| current_07 | `right_ankle_roll_joint` | 6.640 | 0.01179 | 0.26263 | [-0.262, 0.262] | True | -0.01439 | CONTROLLER_LIMIT_MANAGEMENT_FAILURE |
| current_07 | `right_elbow_joint` | 11.160 | -1.17223 | -2.36350 | [-2.356, 0.000] | True | -0.00894 | CONTROLLER_LIMIT_MANAGEMENT_FAILURE |
| current_07 | `right_wrist_pitch_joint` | 10.180 | 0.00286 | -0.58405 | [-0.576, 0.576] | True | -0.02298 | CONTROLLER_LIMIT_MANAGEMENT_FAILURE |
| current_07 | `waist_pitch_joint` | 8.380 | -0.01791 | -0.31713 | [-0.314, 0.314] | True | -0.02039 | CONTROLLER_LIMIT_MANAGEMENT_FAILURE |
| phase3ar_final_candidate | `left_ankle_pitch_joint` | 7.400 | -0.26144 | 0.48139 | [-0.803, 0.454] | True | -0.03059 | CONTROLLER_LIMIT_MANAGEMENT_FAILURE |
| phase3ar_final_candidate | `left_ankle_roll_joint` | 7.360 | 0.01984 | -0.26497 | [-0.262, 0.262] | True | -0.05899 | CONTROLLER_LIMIT_MANAGEMENT_FAILURE |
| phase3ar_final_candidate | `right_ankle_pitch_joint` | 7.220 | -0.30006 | 0.46660 | [-0.803, 0.454] | True | -0.02075 | CONTROLLER_LIMIT_MANAGEMENT_FAILURE |
| phase3ar_final_candidate | `right_ankle_roll_joint` | 7.400 | 0.01312 | -0.27099 | [-0.262, 0.262] | True | -0.06282 | CONTROLLER_LIMIT_MANAGEMENT_FAILURE |
| phase3ar_final_candidate | `right_wrist_pitch_joint` | 10.040 | 0.00286 | -0.59662 | [-0.576, 0.576] | True | -0.02066 | CONTROLLER_LIMIT_MANAGEMENT_FAILURE |
| phase3ar_final_candidate | `right_wrist_roll_joint` | 11.720 | 0.00172 | -0.73031 | [-0.724, 1.510] | True | -0.00767 | CONTROLLER_LIMIT_MANAGEMENT_FAILURE |
| phase3ar_final_candidate | `waist_pitch_joint` | 7.460 | -0.02950 | -0.31914 | [-0.314, 0.314] | True | -0.01107 | CONTROLLER_LIMIT_MANAGEMENT_FAILURE |

The measured references at first violation are inside the current MJCF ranges;
actual simulated q crosses a limit while the target remains legal. The supported
classification is `CONTROLLER_LIMIT_MANAGEMENT_FAILURE`, not `MJCF_RANGE_ERROR`.
