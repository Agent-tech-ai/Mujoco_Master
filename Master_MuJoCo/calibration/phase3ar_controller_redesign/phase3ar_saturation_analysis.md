# Phase 3A-R saturation analysis

| experiment | joint | start s | ratio | max consecutive s | max |error| rad | max |balance add| N·m |
| --- | --- | --- | --- | --- | --- | --- |
| current_07 | `left_ankle_pitch_joint` | 7.160 | 0.4916 | 6.080 | 0.7428 | 216.294 |
| current_07 | `left_ankle_roll_joint` | 7.760 | 0.4164 | 3.380 | 0.2706 | 206.987 |
| current_07 | `left_elbow_joint` | 8.280 | 0.0070 | 0.080 | 0.3282 | 0.000 |
| current_07 | `left_shoulder_pitch_joint` | 8.220 | 0.0042 | 0.040 | 0.2934 | 0.000 |
| current_07 | `left_shoulder_roll_joint` | 8.280 | 0.0014 | 0.020 | 0.0331 | 0.000 |
| current_07 | `left_shoulder_yaw_joint` | 8.280 | 0.0125 | 0.180 | 0.3130 | 0.000 |
| current_07 | `left_wrist_pitch_joint` | 8.220 | 0.4220 | 5.820 | 0.4425 | 0.000 |
| current_07 | `left_wrist_roll_joint` | 8.200 | 0.4290 | 6.160 | 1.5477 | 0.000 |
| current_07 | `right_ankle_pitch_joint` | 7.920 | 0.4485 | 6.440 | 0.7867 | 216.294 |
| current_07 | `right_ankle_roll_joint` | 7.760 | 0.4150 | 3.380 | 0.2702 | 206.987 |
| current_07 | `right_elbow_joint` | 8.400 | 0.2911 | 4.060 | 1.1929 | 0.000 |
| current_07 | `right_shoulder_pitch_joint` | 8.420 | 0.1407 | 2.000 | 1.7028 | 0.000 |
| current_07 | `right_shoulder_roll_joint` | 8.400 | 0.0014 | 0.020 | 0.0628 | 0.000 |
| current_07 | `right_shoulder_yaw_joint` | 8.400 | 0.0014 | 0.020 | 0.0691 | 0.000 |
| current_07 | `right_wrist_pitch_joint` | 8.900 | 0.3552 | 4.780 | 0.6018 | 0.000 |
| current_07 | `right_wrist_roll_joint` | 8.400 | 0.0696 | 0.880 | 0.1717 | 0.000 |
| current_07 | `waist_pitch_joint` | 8.340 | 0.0042 | 0.060 | 0.2992 | 0.000 |
| current_07 | `waist_roll_joint` | 8.380 | 0.0014 | 0.020 | 0.2660 | 0.000 |
| phase3ar_final_candidate | `head_yaw_joint` | 7.660 | 0.0070 | 0.100 | 0.3302 | 0.000 |
| phase3ar_final_candidate | `left_ankle_pitch_joint` | 6.860 | 0.5070 | 6.960 | 0.7458 | 214.998 |
| phase3ar_final_candidate | `left_ankle_roll_joint` | 7.440 | 0.2437 | 2.400 | 0.3077 | 152.845 |
| phase3ar_final_candidate | `left_shoulder_pitch_joint` | 7.560 | 0.0042 | 0.060 | 0.2647 | 0.000 |
| phase3ar_final_candidate | `left_shoulder_roll_joint` | 7.620 | 0.0014 | 0.020 | 0.0253 | 0.000 |
| phase3ar_final_candidate | `left_wrist_pitch_joint` | 7.560 | 0.4136 | 2.800 | 0.3104 | 0.000 |
| phase3ar_final_candidate | `left_wrist_roll_joint` | 7.580 | 0.4680 | 6.700 | 0.7674 | 0.000 |
| phase3ar_final_candidate | `right_ankle_pitch_joint` | 7.380 | 0.4861 | 6.980 | 0.7868 | 214.998 |
| phase3ar_final_candidate | `right_ankle_roll_joint` | 7.440 | 0.2354 | 2.300 | 0.3167 | 152.845 |
| phase3ar_final_candidate | `right_elbow_joint` | 7.620 | 0.0042 | 0.040 | 0.6412 | 0.000 |
| phase3ar_final_candidate | `right_shoulder_pitch_joint` | 7.660 | 0.1309 | 1.860 | 1.7221 | 0.000 |
| phase3ar_final_candidate | `right_shoulder_roll_joint` | 7.620 | 0.0042 | 0.060 | 0.0854 | 0.000 |
| phase3ar_final_candidate | `right_shoulder_yaw_joint` | 10.820 | 0.1908 | 2.740 | 0.3297 | 0.000 |
| phase3ar_final_candidate | `right_wrist_pitch_joint` | 9.300 | 0.3482 | 4.740 | 0.5995 | 0.000 |
| phase3ar_final_candidate | `right_wrist_roll_joint` | 7.620 | 0.2507 | 3.460 | 0.7337 | 0.000 |
| phase3ar_final_candidate | `waist_pitch_joint` | 7.600 | 0.0084 | 0.120 | 0.2785 | 0.000 |
| phase3ar_final_candidate | `waist_roll_joint` | 7.600 | 0.0056 | 0.080 | 0.3092 | 0.000 |

The controller has no integral term or equivalent accumulating state; `CONTROLLER_WINDUP`
is therefore not supported. Saturation appears after early tracking/contact/limit
problems in whole-body replay and is classified as **`TRACKING_CONFLICT_WITH_LIMIT_COUPLING`**,
not a reason to increase torque limits or gear.
