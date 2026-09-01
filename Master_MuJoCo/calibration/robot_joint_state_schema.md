# Robot JointStateArray schema

- Live evidence: `calibration/evidence/x2_phase2b_static_capture.txt`
- Raw `ros2 interface show` and four `echo --once` outputs: `calibration/evidence/x2_phase2b_schema_and_one_shots.txt`
- Official interface reference: https://x2-aimdk.agibot.com/en/v0.9.0/Interface/control_mod/joint_control.html
- Capture was subscription-only; no command, service, or action was sent.

## CONFIRMED

### Live-loaded ROS message fields

| Type | field | ROS type |
|---|---|---|
| aimdk_msgs/msg/JointStateArray | header | `aimdk_msgs/MessageHeader` |
| aimdk_msgs/msg/JointStateArray | state | `aimdk_msgs/DomainErrorState` |
| aimdk_msgs/msg/JointStateArray | joints | `sequence<aimdk_msgs/JointState>` |
| aimdk_msgs/msg/JointState | name | `string` |
| aimdk_msgs/msg/JointState | position | `double` |
| aimdk_msgs/msg/JointState | velocity | `double` |
| aimdk_msgs/msg/JointState | effort | `double` |
| aimdk_msgs/msg/JointState | coil_temp | `uint8` |
| aimdk_msgs/msg/JointState | motor_temp | `uint8` |
| aimdk_msgs/msg/JointState | motor_vol | `uint8` |
| aimdk_msgs/msg/MessageHeader | stamp | `builtin_interfaces/Time` |
| aimdk_msgs/msg/MessageHeader | frame_id | `string` |
| aimdk_msgs/msg/MessageHeader | sequence | `uint32` |
| aimdk_msgs/msg/MessageHeader | meas_stamp | `builtin_interfaces/Time` |
| aimdk_msgs/msg/DomainErrorState | value | `uint8` |

### Live array observations

| group | topic | serialized samples | observed array length(s) | documented length | length result |
|---|---|---:|---|---:|---|
| head | `/aima/hal/joint/head/state` | 1339 | `[2]` | 2 | MATCH |
| arm | `/aima/hal/joint/arm/state` | 1385 | `[14]` | 14 | MATCH |
| waist | `/aima/hal/joint/waist/state` | 1383 | `[3]` | 3 | MATCH |
| leg | `/aima/hal/joint/leg/state` | 1390 | `[12]` | 12 | MATCH |

### Live names by array index

| group | index | observed `JointState.name` values | documented candidate |
|---|---:|---|---|
| head | 0 | `['head_yaw_joint']` | head_yaw |
| head | 1 | `['head_pitch_joint']` | head_pitch |
| arm | 0 | `['left_shoulder_pitch_joint']` | left_shoulder_pitch |
| arm | 1 | `['left_shoulder_roll_joint']` | left_shoulder_roll |
| arm | 2 | `['left_shoulder_yaw_joint']` | left_shoulder_yaw |
| arm | 3 | `['left_elbow_joint']` | left_elbow |
| arm | 4 | `['left_wrist_yaw_joint']` | left_wrist_yaw |
| arm | 5 | `['left_wrist_pitch_joint']` | left_wrist_pitch |
| arm | 6 | `['left_wrist_roll_joint']` | left_wrist_roll |
| arm | 7 | `['right_shoulder_pitch_joint']` | right_shoulder_pitch |
| arm | 8 | `['right_shoulder_roll_joint']` | right_shoulder_roll |
| arm | 9 | `['right_shoulder_yaw_joint']` | right_shoulder_yaw |
| arm | 10 | `['right_elbow_joint']` | right_elbow |
| arm | 11 | `['right_wrist_yaw_joint']` | right_wrist_yaw |
| arm | 12 | `['right_wrist_pitch_joint']` | right_wrist_pitch |
| arm | 13 | `['right_wrist_roll_joint']` | right_wrist_roll |
| waist | 0 | `['waist_yaw_joint']` | waist_yaw |
| waist | 1 | `['waist_pitch_joint']` | waist_pitch |
| waist | 2 | `['waist_roll_joint']` | waist_roll |
| leg | 0 | `['left_hip_pitch_joint']` | left_hip_pitch |
| leg | 1 | `['left_hip_roll_joint']` | left_hip_roll |
| leg | 2 | `['left_hip_yaw_joint']` | left_hip_yaw |
| leg | 3 | `['left_knee_joint']` | left_knee |
| leg | 4 | `['left_ankle_pitch_joint']` | left_ankle_pitch |
| leg | 5 | `['left_ankle_roll_joint']` | left_ankle_roll |
| leg | 6 | `['right_hip_pitch_joint']` | right_hip_pitch |
| leg | 7 | `['right_hip_roll_joint']` | right_hip_roll |
| leg | 8 | `['right_hip_yaw_joint']` | right_hip_yaw |
| leg | 9 | `['right_knee_joint']` | right_knee |
| leg | 10 | `['right_ankle_pitch_joint']` | right_ankle_pitch |
| leg | 11 | `['right_ankle_roll_joint']` | right_ankle_roll |

### Field interpretation

- Array position field: `joints[].position` (double).
- Array velocity field: `joints[].velocity` (double).
- Array effort field: `joints[].effort` (double).
- Joint name field: `string`; observed values by group: `{'head': ['head_pitch_joint', 'head_yaw_joint'], 'arm': ['left_elbow_joint', 'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint', 'left_wrist_pitch_joint', 'left_wrist_roll_joint', 'left_wrist_yaw_joint', 'right_elbow_joint', 'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_wrist_pitch_joint', 'right_wrist_roll_joint', 'right_wrist_yaw_joint'], 'waist': ['waist_pitch_joint', 'waist_roll_joint', 'waist_yaw_joint'], 'leg': ['left_ankle_pitch_joint', 'left_ankle_roll_joint', 'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint', 'left_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint', 'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint', 'right_knee_joint']}`.
- Joint ID field: `NOT_PRESENT_IN_LIVE_SCHEMA`.
- Motor ID field: `NOT_PRESENT_IN_LIVE_SCHEMA`.
- Status/state fields in JointState: `[]`.
- Temperature/voltage fields: `['coil_temp', 'motor_temp', 'motor_vol']`.
- JointStateArray fields: `{'header': 'aimdk_msgs/MessageHeader', 'state': 'aimdk_msgs/DomainErrorState', 'joints': 'sequence<aimdk_msgs/JointState>'}`; its `header` is the message-level timestamp source.
- Live one-shot headers used frame IDs `x2_arm`, `x2_head`, `x2_leg`, and `x2_waist`; `stamp` and `sequence` were populated, while all four observed `meas_stamp` values were zero.
- `static_001.csv` uses the subscriber's monotonic elapsed receive time for its `timestamp`; source headers remain preserved in raw evidence.
- AimDK documents position as rad, velocity as rad/s, and effort as torque in N·m.

## FIELD_TEST_EVIDENCE

- Live samples establish the actual schema and array lengths on this robot/software version.
- All 31 live array indices carried stable, populated `JointState.name` values; these names confirm the interface-level index assignment directly.

## INFERRED_CANDIDATE

- Exact string equality between 30 live hardware names and current MuJoCo joint names is confirmed. Physical direction/zero correspondence remains only a candidate interpretation; static values were not used to guess it.

## UNKNOWN

- AimDK documentation labels `effort` as torque (N·m), but does not identify whether it is measured motor torque, estimated joint torque, commanded torque, current-derived torque, or another estimator output.
- Hardware joint IDs are unknown when no ID field is present.
- Hardware zero, encoder offset, and hardware-to-MuJoCo sign remain unknown.

## NEEDS_PHYSICAL_VERIFICATION

- Confirm the documented array-to-physical-joint order independently before any command-producing calibration phase.
- Confirm effort origin with manufacturer source/API documentation or implementation source.
