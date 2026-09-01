# Phase 2B-2 MuJoCo active-test rehearsal

Status: **REHEARSAL COMPLETED; TRACKING AND BASE STABILITY NOT VALIDATED; REAL SIGN/ZERO NOT INFERRED**

Sequence for each accepted candidate: current → smooth +delta → hold → return → smooth -delta → hold → return. J2 candidates were skipped by the offline 5° screening reserve.

The fixed-base model is used for repeatable command, logger, joint-limit, and modeled self-collision checks. Numeric hardware coordinates are applied to same-name MuJoCo joints only as an explicit rehearsal assumption; real sign/zero remains UNKNOWN.

| Joint | Delta (°) | Sign response | Tracking | Max error (°) | Peak velocity (°/s) | Peak force | Return error (°) | Max self contacts | Limit violations |
|---|---:|---|---|---:|---:|---:|---:|---:|---:|
| `left_wrist_roll_joint` | 2.000000 | SIM_SIGN_RESPONSE_MATCH | TRACKING_NOT_SETTLED | 2.051699 | 1.343196 | 0.332863 | -0.410644 | 0 | 0 |
| `right_wrist_roll_joint` | 2.000000 | SIM_SIGN_RESPONSE_MATCH | TRACKING_NOT_SETTLED | 2.051695 | 1.343189 | 0.340593 | -0.410643 | 0 | 0 |
| `right_wrist_yaw_joint` | 2.000000 | SIM_SIGN_RESPONSE_MATCH | TRACKING_NOT_SETTLED | 2.049541 | 1.338693 | 0.307079 | -0.410297 | 0 | 0 |
| `left_wrist_yaw_joint` | 2.000000 | SIM_SIGN_RESPONSE_MATCH | TRACKING_NOT_SETTLED | 2.049543 | 1.338697 | 0.302900 | -0.410298 | 0 | 0 |
| `left_wrist_pitch_joint` | 2.000000 | SIM_SIGN_RESPONSE_MATCH | TRACKING_NOT_SETTLED | 2.051664 | 1.343117 | 0.504327 | -0.410644 | 0 | 0 |
| `right_wrist_pitch_joint` | 2.000000 | SIM_SIGN_RESPONSE_MATCH | TRACKING_NOT_SETTLED | 2.051665 | 1.343117 | 0.504814 | -0.410644 | 0 | 0 |
| `left_elbow_joint` | 2.000000 | SIM_SIGN_RESPONSE_MATCH | TRACKING_NOT_SETTLED | 1.189064 | 2.847111 | 2.017599 | -0.333038 | 0 | 0 |
| `right_elbow_joint` | 2.000000 | SIM_SIGN_RESPONSE_MATCH | TRACKING_NOT_SETTLED | 1.189084 | 2.847170 | 2.002027 | -0.333095 | 0 | 0 |
| `right_shoulder_yaw_joint` | 2.000000 | SIM_SIGN_RESPONSE_MATCH | TRACKING_NOT_SETTLED | 1.178886 | 2.808353 | 0.791264 | -0.274361 | 0 | 0 |
| `left_shoulder_yaw_joint` | 2.000000 | SIM_SIGN_RESPONSE_MATCH | TRACKING_NOT_SETTLED | 1.178859 | 2.808264 | 0.830418 | -0.274310 | 0 | 0 |
| `left_shoulder_pitch_joint` | 2.000000 | SIM_SIGN_RESPONSE_MATCH | TRACKING_NOT_SETTLED | 1.214318 | 2.930908 | 0.705871 | -0.413885 | 0 | 0 |
| `right_shoulder_pitch_joint` | 2.000000 | SIM_SIGN_RESPONSE_MATCH | TRACKING_NOT_SETTLED | 1.214326 | 2.930920 | 0.618524 | -0.413781 | 0 | 0 |

## Collision and limit result

- Fixed-base candidate runs with modeled self-collision: 0.
- Fixed-base candidate runs with MuJoCo limit violation: 0.
- Collision scope is incomplete: the supplied MJCF ends at wrist-roll links and comments out some collision meshes. This does not prove physical clearance for hands, cabling, clothing, or surroundings.
- Runs classified `TRACKING_NOT_SETTLED`: 12. Target generation, logging, limit checks, and coordinate response worked, but the current uncalibrated MuJoCo controller did not meet the rehearsal tracking/return thresholds. Per scope, no gains or dynamics were changed.

## Base stability probe

A free-base no-action baseline over 7.5s reached max tilt 93.180°, max XY drift 0.911 m, and minimum pelvis height 0.077 m.
The top-candidate `left_wrist_roll_joint` rehearsal reached max tilt 93.110°, max XY drift 0.910 m, and minimum pelvis height 0.077 m.

Result: **BASE_STABILITY_NOT_DEMONSTRATED**. The current MuJoCo project uses joint PD control and has no validated X2 whole-body balance controller. The free-base fall/drift is therefore an infrastructure limitation and must not be attributed to the candidate joint. The fixed-base success is likewise not real balance evidence.

## Generated artifacts

- `calibration\active_tests\sim\left_wrist_roll_joint.csv` and `calibration\plots\phase2b2_sim\left_wrist_roll_joint.png`
- `calibration\active_tests\sim\right_wrist_roll_joint.csv` and `calibration\plots\phase2b2_sim\right_wrist_roll_joint.png`
- `calibration\active_tests\sim\right_wrist_yaw_joint.csv` and `calibration\plots\phase2b2_sim\right_wrist_yaw_joint.png`
- `calibration\active_tests\sim\left_wrist_yaw_joint.csv` and `calibration\plots\phase2b2_sim\left_wrist_yaw_joint.png`
- `calibration\active_tests\sim\left_wrist_pitch_joint.csv` and `calibration\plots\phase2b2_sim\left_wrist_pitch_joint.png`
- `calibration\active_tests\sim\right_wrist_pitch_joint.csv` and `calibration\plots\phase2b2_sim\right_wrist_pitch_joint.png`
- `calibration\active_tests\sim\left_elbow_joint.csv` and `calibration\plots\phase2b2_sim\left_elbow_joint.png`
- `calibration\active_tests\sim\right_elbow_joint.csv` and `calibration\plots\phase2b2_sim\right_elbow_joint.png`
- `calibration\active_tests\sim\right_shoulder_yaw_joint.csv` and `calibration\plots\phase2b2_sim\right_shoulder_yaw_joint.png`
- `calibration\active_tests\sim\left_shoulder_yaw_joint.csv` and `calibration\plots\phase2b2_sim\left_shoulder_yaw_joint.png`
- `calibration\active_tests\sim\left_shoulder_pitch_joint.csv` and `calibration\plots\phase2b2_sim\left_shoulder_pitch_joint.png`
- `calibration\active_tests\sim\right_shoulder_pitch_joint.csv` and `calibration\plots\phase2b2_sim\right_shoulder_pitch_joint.png`

MuJoCo `actuator_force` is logged in `measured_torque`; it is not asserted equivalent to real `JointState.effort`. No real hardware sign, zero, encoder offset, dynamics, mass, inertia, friction, actuator, Kp, or Kd conclusion is drawn.
