# Phase 2H IMU transform closure

Overall result: **IMU_TRANSFORM = PARTIAL**.

No raw log was modified. No empirically fitted rotation was promoted to a sensor transform, and no IMU conversion tool was created because the complete transform chain is not confirmed.

## Evidence manifest

- Robot read-only capture: `../../work/phase2h_robot_evidence_readonly.txt`, SHA-256 `27CCD8124271F48F15585A2913BA85CBECB81D2F6752CE2A12E8C78AC8E10B72`.
- Soft-engineer/AimDK source capture: `../../work/phase2h_agentech01_evidence_readonly.txt`, SHA-256 `8FE7A087490E4788FA42DC313B3D3AF939503DA5A4A7531396B004233E5BBB5D`.
- Prior observation and relative-rotation analysis: `phase2g_imu_transform_report.md`.
- Manufacturer interface documentation: `https://x2-aimdk.agibot.com/en/v0.9.0/Interface/hal/sensor.html`.
- Manufacturer sensor overview: `https://x2-aimdk.agibot.com/en/latest/about_agibot_X2/sensor_fov.html`.

Both captures completed with zero publisher calls, zero state-changing service/action calls, zero control-mode/process/configuration changes, and zero robot motion.

## Closure matrix

| Required item | Status | Confirmed evidence | Missing evidence |
| --- | --- | --- | --- |
| `RAW_SENSOR_FRAME` | **UNKNOWN** | X2 Ultra has one FORSENSE FSS-IMU16460-DM at the chest and one at the hip; topics are identified as chest and torso/hip IMU streams. | Driver-side raw frame names, sensor package axis marks, mounting rotation, and calibration matrix. |
| `MESSAGE_FRAME_ID` | **CONFIRMED** | Both `/aima/hal/imu/chest/state` and `/aima/hal/imu/torso/state` emitted `sensor_msgs/msg/Imu` with `header.frame_id = base_link`; this is also true for every accepted Phase 2D message. | None for the literal string; its semantic correctness is a separate question. |
| `OUTPUT_AXIS_CONVENTION` | **PARTIAL** | The installed RL example consumes torso `angular_velocity` and quaternion components directly, with no transform in the shown callback/observation path. ROS message units are rad/s and m/s². | HAL IMU publisher source or vendor statement proving whether orientation, gyro, and acceleration are rotated into `base_link`, plus world/orientation convention and gravity/filter policy. |
| `BASE_LINK_TRANSFORM` | **UNKNOWN** | A live `/tf_static` publisher exists. The upstream model contains pelvis-to-IMU and torso-link-to-IMU candidate fixed joints with zero RPY. | The `/tf_static` sample timed out; live `/tf` was absent. No deployed `robot_description`, transform authority, or hash ties the upstream URDF to this firmware. The upstream root is `pelvis`, not `base_link`. |

## Runtime findings

- The live graph identifies `/soc0_hal_imu3041` as publisher of both IMU topics and `mc_ros2_node2524` as a consumer.
- The read-only one-shots again showed acceleration magnitude near gravity for both streams. This is observationally consistent with gravity being present while stationary, but it is not a driver/API definition.
- The official RL example stores each `sensor_msgs/msg/Imu` unchanged. Its observation path uses torso angular velocity directly and forms an Eigen quaternion directly from the message before calculating projected gravity. This confirms the example expects usable axes, but does not disclose what transform the HAL publisher applied.
- `/tf_static` had one publisher (`soc2_hal_sensor1473`) with transient-local durability, but the subscription-only echo timed out. Therefore no deployed chest/torso/base transform was captured.

## Consistency check

The stationary Phase 2D pre-roll gave a stable torso-relative-to-chest orientation of approximately `(-3.870, 2.397, 68.259)` degrees in Euler xyz, with about `0.0023` degree angular dispersion. This is strong evidence that the two absolute quaternion streams must not be treated as interchangeable merely because both carry the string `base_link`. It is not sufficient to solve or validate a physical mounting transform.

## Permitted comparisons

- Independently baselined relative roll/pitch changes for each real IMU.
- Rotation-invariant gyro magnitude comparisons.
- Joint-space analyses that do not fit absolute base attitude.

The following remain prohibited as calibrated comparisons:

- absolute real-versus-simulation roll/pitch/yaw fitting;
- axis-by-axis gyro or acceleration fitting across real and simulation frames;
- use of a stationary-data-derived rotation as ground truth;
- IMU-based mass/inertia or balance-dynamics fitting.

## Closure decision

`CONFIRMED_TRANSFORM`: none.

`PARTIAL_TRANSFORM`: topic identity, message type, literal `base_link` label, sensor locations/model, and downstream no-transform consumption path.

`UNKNOWN`: raw sensor axes, mounting rotations, deployed TF chain, HAL output rotation, orientation/world convention, and filtering/gravity contract.

**IMU_TRANSFORM = PARTIAL**. Absolute base-attitude fitting remains blocked.
