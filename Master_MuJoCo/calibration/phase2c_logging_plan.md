# Phase 2C — active-test logging plan

Status: **PLAN ONLY / NOT EXECUTED**

## Current pipeline

The standing preset wrapper records high-level task ID/state and MC posture
before/after. It does not provide a unified joint/IMU log. The separate seated
controller has rich internal recording, but it is direct HAL and is not an
acceptable logger/control owner for an MC-preserving standing test.

## Recommended subscription-only logger

Run the logger in the robot's existing ROS 2/AimDK environment. It must only
subscribe and must never publish or call a service/action.

| Required value | Source |
|---|---|
| high-level request | local test-plan record: task, joint, delta, duration |
| MC command position | `/aima/hal/joint/arm/command` |
| measured position/velocity/effort | `/aima/hal/joint/arm/state` |
| MC mode/action | `/aima/mc/common/state` and read-only `GetMcAction` before/after |
| chest IMU | `/aima/hal/imu/chest/state` |
| torso IMU | `/aima/hal/imu/torso/state` |

The arm command topic represents MC's downstream joint command, not the
high-level `SetMcPresetMotion` request. Preserve both fields separately.

## Output schema

One normalized long-form record per timestamp/joint:

```text
timestamp
joint_name
command_position
measured_position
measured_velocity
effort
effort_source
mc_mode
mc_action
imu_sensor
imu_frame_id
imu_quaternion
imu_gyro
imu_accel
high_level_task_id
high_level_motion_id
```

Keep `effort_source=UNKNOWN` until manufacturer/source evidence identifies
whether it is measured, estimated, commanded, current-derived, or another
quantity. The documented N·m unit alone is insufficient.

## Timing and validation

- Capture at least 2 s before the request and 2 s after task completion.
- Preserve ROS header timestamps and host receive timestamps.
- Record publisher/subscriber counts before and after capture.
- Align command/state/IMU streams offline; do not assume identical arrival
  times.
- Fail the data-quality check on missing joint names, stale samples, sequence
  gaps, non-monotonic timestamps, or any unexpected arm-command publisher.

No logger was run during Phase 2C because no motion was authorized.
