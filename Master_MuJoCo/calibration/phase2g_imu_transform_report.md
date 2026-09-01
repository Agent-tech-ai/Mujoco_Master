# Phase 2G IMU transform investigation

Overall status: **PARTIAL_TRANSFORM**. No raw log was modified and no transform is promoted to confirmed.

| Classification | Result |
|---|---|
| CONFIRMED_TRANSFORM | none |
| PARTIAL_TRANSFORM | message frame labels, upstream URDF mounting candidates, and a stable observed chest/torso relative rotation |
| UNKNOWN | deployed TF chain, driver rotation convention, and sensor-to-comparison-frame transform |

## CONFIRMED facts

- Phase 2D topics are `/aima/hal/imu/chest/state` and `/aima/hal/imu/torso/state`, both `sensor_msgs/msg/Imu`.
- All 1470 chest and 1469 torso messages in the accepted capture report `frame_id=base_link`.
- The Phase 2D graph contains `/tf_static` (`tf2_msgs/msg/TFMessage`), but the subscription-only recorder did not capture its messages. Phase 2A also discovered TF topics, without a transform snapshot.
- The supplied upstream `assets/Master/ff_master_fist.urdf` has candidate fixed joints: pelvis -> `imu_in_pelvis_link`, xyz `(0.0239465,-0.0002287,0.0417100)`, rpy `(0,0,0)`; torso_link -> `imu_in_torso_link`, xyz `(-0.0248787,0.0019853,0.1059381)`, rpy `(0,0,0)`.
- That URDF uses `pelvis` as its root and contains no `base_link`, so it does not resolve what the live message label means.

## PARTIAL_TRANSFORM evidence

During the stationary pre-roll, the observed torso orientation relative to chest was mean Euler xyz **(-3.870, 2.397, 68.259) deg**, with angular dispersion std 0.0023 deg (n=140). This stable offset is observational evidence, not a mounting calibration. It conflicts with treating both absolute quaternions as already interchangeable merely because both headers say `base_link`.

The upstream URDF is not proven to be the deployed robot description and it only specifies IMU-link mounting relative to different parent links; waist pose is also between pelvis and torso. No captured deployed TF tree, robot_description hash, driver convention, or static transform links the message orientation convention to the MuJoCo comparison frame.

## UNKNOWN

- Whether the driver rotates orientation, gyro and acceleration into `base_link`, or only labels the message.
- The deployed chest/torso sensor mounting rotations and TF authorities.
- Gravity inclusion remains consistent with the ~9.8 m/s² stationary acceleration norm, but driver filtering/convention is not source-confirmed.

Because the transform is not confirmed, no real-IMU conversion tool was created. `phase2g_imu_relative_comparison.csv` recomputes only independently baselined relative roll/pitch plus rotation-invariant gyro norm; it is not an axis-aligned IMU calibration.
