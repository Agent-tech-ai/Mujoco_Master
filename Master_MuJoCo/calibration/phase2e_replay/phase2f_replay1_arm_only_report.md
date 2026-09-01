# Phase 2F Replay 1 — arm-reference only

## Setup

- Scene: current `scene_x2_free.xml`; free base enabled.
- Controller: current `SimulationStabilityController`, unchanged.
- Reference: real **measured** arm q(t), not MC internal command.
- Controlled joints: left_elbow_joint, left_shoulder_pitch_joint, left_shoulder_roll_joint, left_shoulder_yaw_joint, left_wrist_roll_joint, left_wrist_yaw_joint, right_elbow_joint, right_shoulder_pitch_joint, right_shoulder_roll_joint, right_shoulder_yaw_joint, right_wrist_roll_joint, right_wrist_yaw_joint.
- Leg, waist, head yaw, and static wrist-pitch targets remain at the measured initial pose; simulated ankle attitude feedback remains active.
- Coordinate mapping is an identity-by-live-name candidate. Hardware sign/zero remain unverified.

## Arm tracking

| joint | RMSE rad | MAE rad | peak error rad | lag candidate s | peak velocity diff rad/s |
| --- | --- | --- | --- | --- | --- |
| right_wrist_yaw_joint | 0.34128 | 0.25261 | 0.66810 | 0.380 | -0.62018 |
| left_wrist_yaw_joint | 0.34076 | 0.25153 | 0.67017 | 0.380 | -0.66018 |
| left_shoulder_roll_joint | 0.30020 | 0.19833 | 0.63839 | 0.240 | -0.46622 |
| right_shoulder_roll_joint | 0.29989 | 0.19807 | 0.63722 | 0.240 | -0.50472 |
| right_wrist_roll_joint | 0.22542 | 0.16578 | 0.44078 | 0.380 | -0.37898 |
| left_wrist_roll_joint | 0.22375 | 0.16356 | 0.43929 | 0.380 | -0.40131 |
| left_shoulder_yaw_joint | 0.21376 | 0.14153 | 0.45007 | 0.240 | -0.35986 |
| right_shoulder_yaw_joint | 0.21344 | 0.14070 | 0.44798 | 0.240 | -0.48071 |
| right_shoulder_pitch_joint | 0.05333 | 0.03448 | 0.11307 | 0.240 | -0.16103 |
| left_shoulder_pitch_joint | 0.05296 | 0.03412 | 0.11274 | 0.240 | -0.29694 |
| right_elbow_joint | 0.03756 | 0.02341 | 0.09434 | 0.220 | -0.30800 |
| left_elbow_joint | 0.03704 | 0.02246 | 0.09871 | 0.220 | -0.29645 |

## Real MC response versus simulation-controller response

| joint | real excursion rad | sim excursion rad | delta RMSE rad | shape corr | lag candidate s |
| --- | --- | --- | --- | --- | --- |
| left_ankle_pitch_joint | 0.04621 | 0.07313 | 0.02750 | 0.234 | 1.000 |
| right_hip_pitch_joint | 0.04103 | 0.05246 | 0.02729 | 0.267 | 1.000 |
| right_knee_joint | 0.02378 | 0.08162 | 0.02498 | 0.340 | 0.420 |
| left_knee_joint | 0.00748 | 0.08886 | 0.02478 | 0.379 | 0.560 |
| right_ankle_pitch_joint | 0.03202 | 0.07219 | 0.02402 | 0.347 | -0.420 |
| left_hip_pitch_joint | 0.03566 | 0.04824 | 0.02140 | 0.294 | 1.000 |
| waist_pitch_joint | 0.03234 | 0.04074 | 0.01594 | 0.289 | -0.360 |
| waist_roll_joint | 0.01022 | 0.00041 | 0.00321 | 0.365 | -0.480 |

The largest gesture-induced delta-response difference is `left_ankle_pitch_joint` with delta RMSE 0.027501 rad (real excursion 0.046211, sim excursion 0.073133). This is a controller-response mismatch candidate, confounded by unverified coordinate mapping and physical-model differences.

Absolute q comparison is dominated by a standing-equilibrium offset at `left_knee_joint` (RMSE 0.117996 rad). The simulation settles away from the fixed measured-initial target before the gesture; this is separate from the delta-response ranking.

## Stability/contact

- Stable/no fall: `True`; fall time: `None`.
- Maximum absolute base roll/pitch: 0.491° / 3.233°.
- Both-feet-contact fraction: 0.998711.
- Foot slip proxy maxima: left 0.010012 m, right 0.006459 m.
- Target clips / limit contacts / self-collision samples / non-foot ground samples: 0 / 0 / 0 / 0.
