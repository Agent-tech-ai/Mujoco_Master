# Hardware vs current MuJoCo arm coordinates

- MJCF inspected: `assets/Master/ff_master_ultra_x2_limits.xml`
- No MJCF value was changed.
- MuJoCo `motor.ctrlrange` is reported verbatim; for these motor actuators it is not a position range.
- Range comparison tolerance: `0.1°` at each endpoint.

| Hardware coordinate | MuJoCo joint | body | axis | hardware field range (deg) | MuJoCo range (rad) | MuJoCo range (deg) | actuator | ctrlrange | range result | sign result | zero result | overall |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| left_shoulder_pitch | left_shoulder_pitch_joint | left_shoulder_pitch_link | `0 1 0` | [-176.471, 116.883] | `[-3.080506, 2.033309]` | [-176.500, 116.500] | motor_left_shoulder_pitch_joint | `-36 36` | RANGE_MISMATCH | INSUFFICIENT_EVIDENCE | ZERO_UNKNOWN | RANGE_MISMATCH |
| right_shoulder_pitch | right_shoulder_pitch_joint | right_shoulder_pitch_link | `0 1 0` | [-176.471, 116.883] | `[-3.080506, 2.033309]` | [-176.500, 116.500] | motor_right_shoulder_pitch_joint | `-36 36` | RANGE_MISMATCH | INSUFFICIENT_EVIDENCE | ZERO_UNKNOWN | RANGE_MISMATCH |
| left_shoulder_roll | left_shoulder_roll_joint | left_shoulder_roll_link | `1 0 0` | [-3.495, 171.486] | `[-0.061087, 3.045600]` | [-3.500, 174.500] | motor_left_shoulder_roll_joint | `-36 36` | RANGE_MISMATCH | INSUFFICIENT_EVIDENCE | ZERO_UNKNOWN | RANGE_MISMATCH |
| right_shoulder_roll | right_shoulder_roll_joint | right_shoulder_roll_link | `1 0 0` | [-171.486, 3.495] | `[-3.045600, 0.061087]` | [-174.500, 3.500] | motor_right_shoulder_roll_joint | `-36 36` | RANGE_MISMATCH | INSUFFICIENT_EVIDENCE | ZERO_UNKNOWN | RANGE_MISMATCH |
| left_shoulder_yaw | left_shoulder_yaw_joint | left_shoulder_yaw_link | `0 0 1` | [-146.448, 146.448] | `[-2.556908, 2.556908]` | [-146.500, 146.500] | motor_left_shoulder_yaw_joint | `-24 24` | MATCH | INSUFFICIENT_EVIDENCE | ZERO_UNKNOWN | INSUFFICIENT_EVIDENCE |
| right_shoulder_yaw | right_shoulder_yaw_joint | right_shoulder_yaw_link | `0 0 1` | [-146.448, 146.448] | `[-2.556908, 2.556908]` | [-146.500, 146.500] | motor_right_shoulder_yaw_joint | `-24 24` | MATCH | INSUFFICIENT_EVIDENCE | ZERO_UNKNOWN | INSUFFICIENT_EVIDENCE |
| left_elbow | left_elbow_joint | left_elbow_link | `0 1 0` | [-134.965, 0] | `[-2.356194, 0]` | [-135.000, 0.000] | motor_left_elbow_joint | `-24 24` | MATCH | INSUFFICIENT_EVIDENCE | ZERO_UNKNOWN | INSUFFICIENT_EVIDENCE |
| right_elbow | right_elbow_joint | right_elbow_link | `0 1 0` | [-134.965, 0] | `[-2.356194, 0]` | [-135.000, 0.000] | motor_right_elbow_joint | `-24 24` | MATCH | INSUFFICIENT_EVIDENCE | ZERO_UNKNOWN | INSUFFICIENT_EVIDENCE |
| left_wrist_yaw | left_wrist_yaw_joint | left_wrist_yaw_link | `0 0 1` | [-146.448, 146.448] | `[-2.556908, 2.556908]` | [-146.500, 146.500] | motor_left_wrist_yaw_joint | `-24 24` | MATCH | INSUFFICIENT_EVIDENCE | ZERO_UNKNOWN | INSUFFICIENT_EVIDENCE |
| right_wrist_yaw | right_wrist_yaw_joint | right_wrist_yaw_link | `0 0 1` | [-146.448, 146.448] | `[-2.556908, 2.556908]` | [-146.500, 146.500] | motor_right_wrist_yaw_joint | `-24 24` | MATCH | INSUFFICIENT_EVIDENCE | ZERO_UNKNOWN | INSUFFICIENT_EVIDENCE |
| left_wrist_pitch | left_wrist_pitch_joint | left_wrist_pitch_link | `0 1 0` | [-31.971, 31.971] | `[-0.575959, 0.575959]` | [-33.000, 33.000] | motor_left_wrist_pitch_joint | `-2.2 2.2` | RANGE_MISMATCH | INSUFFICIENT_EVIDENCE | ZERO_UNKNOWN | RANGE_MISMATCH |
| right_wrist_pitch | right_wrist_pitch_joint | right_wrist_pitch_link | `0 1 0` | [-31.971, 31.971] | `[-0.575959, 0.575959]` | [-33.000, 33.000] | motor_right_wrist_pitch_joint | `-2.2 2.2` | RANGE_MISMATCH | INSUFFICIENT_EVIDENCE | ZERO_UNKNOWN | RANGE_MISMATCH |
| left_wrist_roll | left_wrist_roll_joint | left_wrist_roll_link | `1 0 0` | [-90.012, 41.482] | `[-1.509710, 0.724312]` | [-86.500, 41.500] | motor_left_wrist_roll_joint | `-2.2 2.2` | RANGE_MISMATCH | INSUFFICIENT_EVIDENCE | ZERO_UNKNOWN | RANGE_MISMATCH |
| right_wrist_roll | right_wrist_roll_joint | right_wrist_roll_link | `1 0 0` | [-41.482, 90.012] | `[-0.724312, 1.509710]` | [-41.500, 86.500] | motor_right_wrist_roll_joint | `-2.2 2.2` | RANGE_MISMATCH | INSUFFICIENT_EVIDENCE | ZERO_UNKNOWN | RANGE_MISMATCH |

## Classification notes

- `MATCH` applies only to numeric range endpoints within tolerance; it does not confirm sign or zero.
- `SIGN_MISMATCH_CANDIDATE` is not assigned from the available static evidence. J2/J7 mirror evidence concerns left versus right hardware control coordinates, not hardware versus MuJoCo.
- Joints with otherwise matching ranges remain `INSUFFICIENT_EVIDENCE` overall because hardware-to-MuJoCo sign and zero are unresolved.
- `ctrlrange` values are actuator control limits and must not be compared numerically with joint angular ranges.

## CONFIRMED

- The table's MuJoCo joint, body, axis, range, actuator, and ctrlrange values were parsed directly from the current X2-limit MJCF.

## FIELD_TEST_EVIDENCE

- The hardware control-coordinate limits and J2/J7 mirrored endpoint observations are transcribed exactly from the operator's Phase 2B-1 request.

## INFERRED_CANDIDATE

- Hardware-to-MuJoCo rows use semantic name correspondence only and remain candidate mappings until correlated evidence exists.

## UNKNOWN

- Hardware-to-MuJoCo sign, zero, encoder offset, and exact physical effort origin.

## NEEDS_PHYSICAL_VERIFICATION

- A later, separately authorized physical-direction verification is required before changing any mapping sign or MJCF axis.
