# Phase 2B-1 summary

This phase used read-only system inspection, ROS 2 interface inspection, one-message state reads, and passive subscriptions. It did not publish a topic, call a service/action, change control mode, operate an actuator, alter a robot process/configuration, modify an MJCF, or create a calibrated model.

## CONFIRMED

- Existing robot runtime prefix: `/agibot/software/common`; its environment provided the installed `aimdk_msgs` Python module and ROS 2 type support without installation.
- `JointStateArray` contains `header`, `state`, and `joints`. Each `JointState` contains `name`, `position`, `velocity`, `effort`, `coil_temp`, `motor_temp`, and `motor_vol`.
- `MessageHeader` contains `stamp`, `frame_id`, `sequence`, and `meas_stamp`. Live one-shot frame IDs were `x2_arm`, `x2_head`, `x2_leg`, and `x2_waist`; the observed `meas_stamp` values were zero.
- Live group lengths: arm 14, head 2, leg 12, waist 3.
- All 31 array indices had stable populated names throughout the passive capture. Thirty names exactly equal current X2-limit MuJoCo joint names. `head_pitch_joint` is live/reserved but absent as a joint from the X2-limit MJCF.
- Passive static capture: 8,259 serialized topic messages across four joint and two IMU topics, producing 45,659 unified CSV rows over approximately 29.5 seconds per topic.
- AimDK documentation defines position as rad, velocity as rad/s, and `effort` as torque in N·m.
- The original `ff_master_ultra.xml` SHA-256 remains `89619295fcc372c57473224130865b2fe4f22e0741f72925fac243805f4353db`.

## FIELD_TEST_EVIDENCE

- The operator-supplied J1–J7 arm limits are stored separately from official-document and MuJoCo limits.
- Agentech.heart() endpoint observations establish mirrored left/right hardware control coordinates for J2: left `+126.042°`, right `-126.042°`.
- The same evidence establishes mirrored left/right hardware control coordinates for J7: left `-63.021°`, right `+63.021°`.
- These statements do not confirm a MuJoCo axis or hardware-to-MuJoCo sign.

## INFERRED_CANDIDATE

- Exact hardware/MuJoCo string-name equality confirms an interface-level name mapping for 30 joints, but physical coordinate equivalence remains a candidate until sign and zero are independently verified.
- Arm range comparison against `ff_master_ultra_x2_limits.xml` finds endpoint matches within 0.1° for J3, J4, and J5 on both sides. This is a numeric range match only.
- No joint is classified `SIGN_MISMATCH_CANDIDATE` from the available static evidence. Static position similarity was not used to infer mappings.

## UNKNOWN

- Hardware joint ID: no joint-ID or motor-ID field exists in the live schema.
- Hardware zero, encoder offset, and hardware-to-MuJoCo sign for every joint.
- Whether `JointState.effort` is measured motor torque, estimated joint torque, commanded torque, current-derived torque, normalized effort, or another estimator output. Only the documented torque label and N·m unit are confirmed.
- IMU gravity policy, bias/scale calibration, and physical sensor-to-base transforms beyond the published frame IDs.
- Why position and velocity remained exactly quantized/constant in the serialized static window; zero observed std must not be interpreted as zero sensor noise.

## NEEDS_PHYSICAL_VERIFICATION

- Independently verify every live name/index against the physical joint before enabling a command-producing calibration phase.
- Verify emergency stop, control mode, command arbitration/API, joint limits, velocity limits, and torque limits before any active test.
- Obtain manufacturer documentation or source describing the physical origin and sign convention of `JointState.effort`.
- Verify hardware zero pose, positive physical direction, encoder offset, and IMU mounting transform under a separately approved low-energy test plan.
- Confirm the robot was physically stationary and document support/contact loads for future static captures.
