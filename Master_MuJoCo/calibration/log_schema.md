# Unified real/simulation log schema

Both robot and MuJoCo exporters must write the same long-form CSV header:

```text
timestamp,joint_name,command_position,measured_position,measured_velocity,measured_torque,imu_quaternion,imu_gyro,imu_accel
```

## Field contract

| Field | Type / encoding | Unit | Meaning |
|---|---|---|---|
| `timestamp` | finite float | seconds | Monotonic source timestamp. Record clock provenance separately. |
| `joint_name` | non-empty string | — | Canonical name from `joint_mapping.csv`, or a name that can be mapped to it. |
| `command_position` | float or blank | rad | Command observed at this timestamp. Blank when unavailable. |
| `measured_position` | float or blank | rad | Encoder/simulated joint position in the source coordinate. Do not silently flip sign or add offsets. |
| `measured_velocity` | float or blank | rad/s | Joint angular velocity. |
| `measured_torque` | float or blank | N·m | Reported joint effort/torque. Preserve the source convention. |
| `imu_quaternion` | quoted JSON array or blank | unit quaternion | `[w,x,y,z]`. Frame and orientation convention must be documented with the log. |
| `imu_gyro` | quoted JSON array or blank | rad/s | `[x,y,z]`, in the documented IMU frame. |
| `imu_accel` | quoted JSON array or blank | m/s² | `[x,y,z]`, including/excluding gravity exactly as documented with the log. |

Example row:

```csv
0.010000,left_knee,0.25,0.248,0.031,1.72,"[1.0,0.0,0.0,0.0]","[0.0,0.0,0.0]","[0.0,0.0,9.80665]"
```

One row represents one joint sample. An IMU sample may be repeated on all joint rows sharing a timestamp; readers deduplicate it. Alternatively, put it on the first joint row for a timestamp and leave the remaining IMU cells blank.

## Required metadata outside the CSV

For each capture, record robot/firmware version, control mode, timestamp clock and epoch, sample rate, dropped-message count, joint array group/order, IMU topic and frame, quaternion ordering, gravity convention, and whether torque is measured/estimated/commanded. None of these are inferred by the analysis tools.

