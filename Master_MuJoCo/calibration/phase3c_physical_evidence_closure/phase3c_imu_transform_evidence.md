# Phase 3C IMU Transform Evidence

## Decision

`IMU_TRANSFORM = PARTIAL`

`IMU_TRANSFORM_READY = NO`

Absolute quaternion/base-attitude fitting remains prohibited. Relative roll/pitch excursions, gyro shape, peak timing and recovery trends remain usable only as auxiliary metrics.

## Confirmed / partial evidence

- Chest and torso topics use `sensor_msgs/msg/Imu` and publish `frame_id = base_link` in the captured robot data.
- AimDK X2 SDK `x2.xml` locates MuJoCo sites `imu_0` on pelvis at `(0.0239465, -0.000228682, 0.04171)` and `imu_1` on torso at `(-0.0248787, 0.00198528, 0.105938)`.
- The captured SDK model renders the corresponding IMU meshes with identity quaternion relative to their parent model bodies.
- Static chest-versus-torso quaternion differences were observed in Phase 2H, but were explicitly retained as observations rather than promoted to mounting transforms.

## Missing transform chain

The following are still not established by a deployed driver/TF/config source:

- `RAW_SENSOR_FRAME`: physical sensor axes and factory orientation.
- Whether the driver rotates raw data before publishing.
- The exact semantic frame represented by the message values despite the `base_link` string.
- A deployed static transform or calibration matrix from each physical IMU frame to pelvis/torso/base.
- Quaternion direction/order convention at the driver boundary and any board-specific sign/permutation.
- Accelerometer gravity convention as implemented by the deployed driver.

The SDK MuJoCo sites are simulator geometry. They strengthen model lineage but do not prove the live sensor driver's output transform. A `frame_id` string alone also does not prove that the numeric samples were transformed into that frame.

## Closure requirement

A complete deployed evidence chain must connect physical sensor axes → driver transformation/calibration → message numeric convention → `base_link`/pelvis/torso TF. Suitable evidence would be the deployed IMU driver assignment code and configuration, a published static TF from the running system, or manufacturer documentation tied to the exact firmware/board revision.

