# Phase 2B-2 active-test neutral arm pose candidate

Status: **OFFLINE CANDIDATE ONLY — DO NOT COMMAND THIS POSE**

Design rule: keep all current arm targets except move hardware-control J2 to +7° left / -7° right. These are the smallest 0.5°-rounded targets that provide at least 10° field-limit margin for both J2 joints at the target. Legs and waist are unchanged by this candidate.

| Joint | Current (°) | Candidate (°) | Move (°) | Lower margin at target (°) | Upper margin at target (°) |
|---|---:|---:|---:|---:|---:|
| `left_shoulder_pitch_joint` | 22.219507 | 22.219507 | +0.000000 | 198.690507 | 94.663493 |
| `left_shoulder_roll_joint` | -0.939315 | 7.000000 | +7.939315 | 10.495000 | 164.486000 |
| `left_shoulder_yaw_joint` | -0.422953 | -0.422953 | +0.000000 | 146.025047 | 146.870953 |
| `left_elbow_joint` | -67.251873 | -67.251873 | +0.000000 | 67.713127 | 67.251873 |
| `left_wrist_yaw_joint` | 0.697608 | 0.697608 | +0.000000 | 147.145608 | 145.750392 |
| `left_wrist_pitch_joint` | 0.010928 | 0.010928 | +0.000000 | 31.981928 | 31.960072 |
| `left_wrist_roll_joint` | -4.382304 | -4.382304 | +0.000000 | 85.629696 | 45.864304 |
| `right_shoulder_pitch_joint` | 22.658989 | 22.658989 | +0.000000 | 199.129989 | 94.224011 |
| `right_shoulder_roll_joint` | 1.093131 | -7.000000 | -8.093131 | 164.486000 | 10.495000 |
| `right_shoulder_yaw_joint` | 0.390004 | 0.390004 | +0.000000 | 146.838004 | 146.057996 |
| `right_elbow_joint` | -67.163982 | -67.163982 | +0.000000 | 67.801018 | 67.163982 |
| `right_wrist_yaw_joint` | 0.379021 | 0.379021 | +0.000000 | 146.827021 | 146.068979 |
| `right_wrist_pitch_joint` | -0.710340 | -0.710340 | +0.000000 | 31.260660 | 32.681340 |
| `right_wrist_roll_joint` | 1.693887 | 1.693887 | +0.000000 | 43.175887 | 88.318113 |

## MuJoCo kinematic validation

- Linear interpolation samples checked: 201.
- Maximum simultaneous contacts: 0.
- Contact pairs: none.
- MuJoCo-limit violations: none.
- Result: `PASS_MODEL_ONLY` when applying the numeric hardware coordinates directly to same-name MuJoCo joints.

## Limitations

- Hardware control coordinates were applied numerically to same-name MuJoCo joints. Hardware-to-MuJoCo sign/zero is still UNKNOWN.
- The supplied MJCF ends at wrist-roll links and has some collision geoms commented out. Zero contacts is model-only evidence, not physical clearance proof.
- No attached hand/end-effector, cable, clothing, environment object, or human clearance is proven by this model.
- Transitioning the real robot into this pose is itself a motion operation and needs a separately approved whole-body procedure. This report does not authorize it.
