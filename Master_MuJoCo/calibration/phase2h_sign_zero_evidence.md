# Phase 2H physical sign and zero evidence

Overall result: the live name/index mapping is confirmed, but **physical sign and physical zero remain UNKNOWN for every requested fitting joint**.

No row in `joint_mapping.csv` was upgraded. Its SHA-256 remained `3975D90F7F9405F3D98F1E19C873FBD5688F02B68D1D239F11A7D12A4FB5FF04` during this closure pass.

## Evidence distinction

The following are confirmed but are not physical sign/zero evidence:

- Live `JointState.name` and group index identify each interface channel.
- Manufacturer limits define allowed hardware control-coordinate ranges.
- The heart trajectory establishes left/right mirrored control coordinates for shoulder roll and wrist roll as `FIELD_TEST_EVIDENCE`.
- Exact hardware/MuJoCo string equality establishes a name-level mapping only.
- `STAND_DEFAULT` identifies an MC state/profile; the inspected material does not supply a firmware-applicable numeric physical pose definition.

## Target-joint matrix

| Hardware joint | Name/index evidence | Physical sign | Physical zero | Evidence result |
| --- | --- | --- | --- | --- |
| `left_shoulder_roll` | live arm index 1, name `left_shoulder_roll_joint` | **UNKNOWN** | **UNKNOWN** | Left/right heart endpoints are mirrored, but no independently observed physical positive direction or zero landmark exists. |
| `right_shoulder_roll` | live arm index 8, name `right_shoulder_roll_joint` | **UNKNOWN** | **UNKNOWN** | Same limitation; symmetry does not establish the MuJoCo-axis relation. |
| `left_wrist_yaw` | live arm index 4, name `left_wrist_yaw_joint` | **UNKNOWN** | **UNKNOWN** | Dynamic response exists, but no documented physical pose/axis reference. |
| `right_wrist_yaw` | live arm index 11, name `right_wrist_yaw_joint` | **UNKNOWN** | **UNKNOWN** | Dynamic response exists, but no documented physical pose/axis reference. |
| `left_wrist_roll` | live arm index 6, name `left_wrist_roll_joint` | **UNKNOWN** | **UNKNOWN** | Left/right heart endpoints are mirrored; this is not a hardware-to-MuJoCo sign proof. |
| `right_wrist_roll` | live arm index 13, name `right_wrist_roll_joint` | **UNKNOWN** | **UNKNOWN** | Same limitation. |
| `left_hip_pitch` | live leg index 0, name `left_hip_pitch_joint` | **UNKNOWN** | **UNKNOWN** | `STAND_DEFAULT` was observed but has no captured numeric physical target/landmark. |
| `right_hip_pitch` | live leg index 6, name `right_hip_pitch_joint` | **UNKNOWN** | **UNKNOWN** | Same limitation. |
| `left_knee` | live leg index 3, name `left_knee_joint` | **UNKNOWN** | **UNKNOWN** | Standing equilibrium is controller-dependent and cannot be called encoder zero. |
| `right_knee` | live leg index 9, name `right_knee_joint` | **UNKNOWN** | **UNKNOWN** | Same limitation. |
| `left_ankle_pitch` | live leg index 4, name `left_ankle_pitch_joint` | **UNKNOWN** | **UNKNOWN** | Balance-controller bias and ground contact prevent a static value from defining physical zero. |
| `right_ankle_pitch` | live leg index 10, name `right_ankle_pitch_joint` | **UNKNOWN** | **UNKNOWN** | Same limitation. |
| `waist_pitch` | live waist index 1, name `waist_pitch_joint` | **UNKNOWN** | **UNKNOWN** | No independently specified pelvis-to-torso neutral angle was found. |

## Pose-source investigation

### `STAND_DEFAULT`

The deployed material proves that `STAND_DEFAULT` is an MC state and can represent different arm gain profiles. It is not a captured kinematic calibration pose. No numeric joint target table tied to the installed firmware was found, so measured standing values cannot be labeled encoder zero.

### Heart and recorded trajectories

The heart preset and physical recording contain real measured trajectories. They establish motion and mirror relationships in hardware control coordinates. They do not provide an independent physical angle definition, fixture, CAD landmark, or encoder datum. They therefore cannot close physical sign or zero.

### Agentech public elbow convention

The standing elbow-adjustment design states that its public semantic convention is positive flexion and negative extension, while the right-elbow encoder conversion remains private. This documents the public API convention, not the sign of `JointState.position` or its zero. It does not upgrade even the elbow hardware mapping without the hidden conversion and physical evidence.

### Offline fixtures and simulation poses

Files labeled `offline-fixture`, MuJoCo keyframes, RL default poses, and controller equilibrium targets are not physical metrology evidence. They are excluded from sign/zero confirmation.

## Evidence required for promotion

For each joint selected for fitting:

1. A manufacturer/deployed-controller definition of a physical pose or axis convention, with firmware/hardware applicability.
2. A passive JointState capture while an operator independently confirms that physical pose or landmark.
3. At least two distinct known physical angles, or one known nonzero pose plus an independently verified positive direction, to separate sign, scale, and offset.
4. Left and right sides verified separately.
5. Encoder zero kept separate from MC standing equilibrium and balance-controller bias.

Until those conditions are met:

- `PHYSICAL_SIGN = UNKNOWN`
- `PHYSICAL_ZERO = UNKNOWN`
- `ENCODER_OFFSET = UNKNOWN`

No calibrated MJCF was created and no UNKNOWN mapping value was modified.
