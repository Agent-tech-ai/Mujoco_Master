# Phase 2F Replay 2 — whole-body measured reference

## Scope

All 30 name-matched MuJoCo joints track the measured real q(t). `head_pitch_joint` is not mapped because the current model fixes that DOF. Replay 2 checks tracking, range, contact, collision, and kinematic consistency; it is not an evaluation of balance-controller prediction.

## Tracking metrics

| joint | RMSE rad | MAE rad | peak error rad | shape corr | lag candidate s | min limit margin rad |
| --- | --- | --- | --- | --- | --- | --- |
| right_wrist_yaw_joint | 0.34118 | 0.25256 | 0.66812 | 0.993 | 0.380 | 0.92176 |
| left_wrist_yaw_joint | 0.34067 | 0.25148 | 0.67019 | 0.993 | 0.380 | 0.92485 |
| left_shoulder_roll_joint | 0.30042 | 0.19845 | 0.63836 | 0.999 | 0.240 | 0.04854 |
| right_shoulder_roll_joint | 0.29976 | 0.19795 | 0.63710 | 0.999 | 0.240 | 0.04815 |
| right_wrist_roll_joint | 0.22541 | 0.16578 | 0.44073 | 0.993 | 0.380 | 0.41551 |
| left_wrist_roll_joint | 0.22377 | 0.16358 | 0.43936 | 0.993 | 0.380 | 0.41802 |
| left_shoulder_yaw_joint | 0.21333 | 0.14139 | 0.45022 | 0.999 | 0.240 | 0.98751 |
| right_shoulder_yaw_joint | 0.21294 | 0.14050 | 0.44810 | 0.999 | 0.240 | 0.99441 |
| right_knee_joint | 0.13658 | 0.13400 | 0.18693 | 0.805 | 0.660 | 0.57285 |
| left_knee_joint | 0.12669 | 0.12472 | 0.17378 | 0.230 | 0.140 | 0.58845 |
| right_ankle_pitch_joint | 0.06598 | 0.06375 | 0.09957 | 0.172 | 0.120 | 0.42615 |
| right_shoulder_pitch_joint | 0.05246 | 0.03416 | 0.11283 | 0.999 | 0.240 | 1.63573 |
| left_shoulder_pitch_joint | 0.05216 | 0.03376 | 0.11225 | 0.999 | 0.240 | 1.63937 |
| left_hip_pitch_joint | 0.04326 | 0.04202 | 0.06597 | 0.672 | 0.120 | 2.17433 |
| right_elbow_joint | 0.03750 | 0.02341 | 0.09423 | 0.998 | 0.220 | 0.90780 |
| left_elbow_joint | 0.03713 | 0.02254 | 0.09871 | 0.998 | 0.220 | 0.91573 |
| left_ankle_pitch_joint | 0.03483 | 0.03151 | 0.06297 | 0.528 | 0.220 | 0.49372 |
| right_hip_pitch_joint | 0.01741 | 0.01371 | 0.04384 | 0.820 | 0.120 | 2.23594 |
| left_ankle_roll_joint | 0.01198 | 0.01107 | 0.01681 | 0.864 | 0.440 | 0.23072 |
| left_hip_roll_joint | 0.00731 | 0.00665 | 0.01284 | 0.152 | -1.000 | 0.25217 |
| right_ankle_roll_joint | 0.00492 | 0.00423 | 0.00954 | 0.841 | 0.340 | 0.23796 |
| waist_pitch_joint | 0.00394 | 0.00303 | 0.01075 | 0.931 | 0.080 | 0.27380 |
| left_hip_yaw_joint | 0.00376 | 0.00361 | 0.00570 | 0.409 | 0.000 | 1.67807 |
| right_hip_roll_joint | 0.00324 | 0.00288 | 0.00512 | 0.187 | -1.000 | 0.25044 |
| right_hip_yaw_joint | 0.00206 | 0.00174 | 0.00400 | 0.684 | 0.240 | 1.69657 |
| waist_roll_joint | 0.00187 | 0.00137 | 0.00650 | 0.919 | 0.200 | 0.48294 |
| waist_yaw_joint | 0.00038 | 0.00027 | 0.00106 | 0.479 | 0.140 | 2.22047 |
| left_wrist_pitch_joint | 0.00021 | 0.00013 | 0.00082 | 0.982 | 0.280 | 0.57365 |
| right_wrist_pitch_joint | 0.00018 | 0.00012 | 0.00056 | 0.935 | 0.280 | 0.56909 |
| head_yaw_joint | 0.00002 | 0.00001 | 0.00009 | 0.896 | 0.220 | 0.34892 |

## Consistency checks

- Reference outside current model range: `0` mapped joints.
- Runtime target clipping: `0` requests across `none`.
- Runtime joint-limit contacts: `0`.
- Self-collision / non-foot-ground samples: `0` / `0`.
- Stable/no fall: `True`.
- Controller saturation samples: `0`; maximum saturation fraction 0.3957.

No q-tracking sign conflict is observed under the identity-coordinate replay assumption. That does **not** confirm physical hardware-to-MuJoCo axis sign or zero; those remain `UNKNOWN` until physical single-joint verification.
