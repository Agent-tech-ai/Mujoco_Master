# AgiBot X2 Ultra robot interface report — Phase 2A

Discovery date: 2026-08-11  
Target: `run@192.168.4.66:22`  
Mode: READ-ONLY discovery

No ROS topic was published. No service or action was called. No control mode, actuator, process, configuration, or MuJoCo dynamics parameter was changed.

Evidence files:

- `calibration/evidence/x2_phase2_discovery_pass1.txt` — system, AimDK CLI, ROS2 graph, topic/service type inventory.
- `calibration/evidence/x2_phase2_discovery_pass2.txt` — type-support search, verbose topic info, and one-message IMU samples.
- The SSH login banner was captured by the operator and supplied in the task transcript.

## CONFIRMED

### Robot and operating system

| Item | Confirmed value | Evidence |
|---|---|---|
| hostname | `agi` | live `hostname` and `hostnamectl` |
| current user | `run`, uid/gid 1000 | live `id`, `whoami` |
| OS | Ubuntu 22.04.5 LTS (Jammy) | `/etc/os-release` |
| kernel | `Linux 5.15.148-tegra`, PREEMPT, build dated 2026-01-14 | `uname -a` |
| architecture | `aarch64` / arm64, 64-bit | `uname -m`, `getconf LONG_BIT`, `hostnamectl` |
| CPU | 8 × ARM Cortex-A78AE; 115.2–1984 MHz | `lscpu` |
| hardware platform | NVIDIA Jetson Orin NX Engineering Reference Developer Kit Super | `hostnamectl` |
| robot series / SKU | `x2` / `X2 Ultra` | sourced AgiBot environment |
| hardware version | `t2d5` | `AGIBOT_HARDWARE_VERSION` |
| serial number | `X240026C3Z0008` | `AGIBOT_SN`; also SSH banner |
| SoC | index `1`; SSH banner labels this host `SoC1` | `AGIBOT_SOC_INDEX`; banner |
| project ID | `lx2501_3` | `AGIBOT_PROJECT_ID` |
| OS image version | `lx2501_3_t2d5-soc1-v0.4.57_hotfix_v2` | SSH login banner |
| Agi release | `release-lx2501_3_t2d5-soc1-v0.9.6` | SSH login banner |
| NVIDIA userspace | L4T R36.4.3; `nvidia-l4t-firmware 36.4.3-20250107174145` | `/etc/nv_tegra_release`, dpkg |

### Network interfaces

| Interface | State | Address observed |
|---|---|---|
| `lo` | UNKNOWN flag from `ip -brief` | `127.0.0.1/8`, `::1/128` |
| `unused0` | DOWN | none |
| `wwan0` | DOWN | none |
| `can0` | DOWN | none |
| `wifi0` | UP | `192.168.4.66/22` plus IPv6 |
| `develop0` | UP | `10.0.1.41/24` |
| `sensor0` | UP | `10.11.1.1/24` |
| `l4tbr0` | DOWN | none |
| `usb0`, `usb1` | DOWN | none |
| `tailscale0` | UNKNOWN flag from `ip -brief` | `100.94.200.26/32` plus IPv6 |
| `ssh0@develop0` | UP | `10.0.200.41/24` |

Default route at discovery time was via `192.168.4.1` on `wifi0`. Full routes are retained in pass 1 evidence.

### AimDK / development environment

| Item | Confirmed value |
|---|---|
| AIMA CLI | `/usr/local/bin/aima` |
| AIMA CLI version | `0.2.28rc9` |
| ROS setup | `/opt/ros/humble/setup.bash` |
| AgiBot environment setup | `/agibot/data/home/agi/.aima/env/bashrc` (readable and sourced only in the temporary discovery shell) |
| DDS profile | `/agibot/data/home/agi/.aima/env/ros_dds_configuration.xml` |
| ROS | ROS 2 Humble (`ROS_DISTRO=humble`, `ROS_VERSION=2`) |
| Python | `/usr/bin/python3`, Python 3.10.12; `rclpy` installed |
| C/C++ toolchain | GCC/G++ 11.4.0; `colcon=/usr/bin/colcon` |
| ROS2 CLI | `/opt/ros/humble/bin/ros2`; Debian package `ros-humble-ros2cli` version `0.18.15-1jammy.20251222.202121` |

`ros2 --version` is not supported by this Humble CLI and returned an argument error; the distro and package version above are the available version evidence.

The `run` shell does **not** have the `aimdk_msgs` Python/C++ ROS package or its type-support installed in its accessible ROS prefix. Searches under `/agibot/data/home/agi`, `/home/run`, `/opt`, and `/usr/local` did not locate it. This does not imply the internal robot processes lack the package; live publishers clearly advertise its types.

### ROS2 graph

The discovery captured 45 node entries representing 43 unique names; `/HK_ros2_rpc_node` appeared three times and ROS2 warned about duplicate names. The unique names were:

```text
/agentech_web_front_main
/aimrt_agent_node1221
/app_proxy
/camera/camera
/camera/camera_container
/cobridge
/cobridge_component_manager
/face_ui_proxy
/hal_ethercat_x21436
/HK_ros2_rpc_node
/imu_sub_node
/launch_ros_66585
/mc_ros2_node2205
/ra_x2_aimrt_monitor
/realtime_mic_to_text
/soc0_cloud_proxy2394
/soc0_data_exporter2290
/soc0_drp_ros2_node1467
/soc0_hal_4852409
/soc0_hal_imu2985
/soc0_hal_pmu2526
/soc0_hds_slave1450
/soc0_housekeeper
/soc0_ota2894
/soc0_rc2197
/soc0_sm2316
/soc0_task_manager3033
/soc0_teleop_bridge2215
/soc1_dm2132
/soc1_drp_ros2_node2141
/soc1_ec2126
/soc1_hal_sensor_orin80521
/soc1_hds_master2119
/soc1_hds_slave2124
/soc1_housekeeper
/soc1_ota2909
/soc2_drp_ros2_node1232
/soc2_hal_audio1852
/soc2_hal_sensor1711
/soc2_hds_slave1230
/soc2_housekeeper
/soc2_interaction_slave1493
/soc2_ota1863
```

The live graph contained 141 unique topics and 540 unique services. Full inventories and types are in pass 1 evidence. Services were listed only; none was invoked. Notable state and potentially dangerous command interfaces observed include:

- State topics: `/aima/hal/joint/{arm,head,leg,waist}/state`, `/aima/hal/imu/{chest,torso}/state`, `/aima/mc/common/state`, `/aima/hal/pmu/state`.
- Command topics present but untouched: `/aima/hal/joint/{arm,head,leg,waist}/command`.
- Read-looking services present but not called: `GetAllJointState`, `GetRobotInfo`, `GetSystemState`, `GetMcAction`, `GetImuConfig`, `GetImuVersion`.
- State-changing services present and explicitly untouched: `SetMcAction`, `MotorTest`, `SetDcuMotorPowerState`, `StartImuDriver`, `SetImuOrientation`, and multiple internal `ExecuteAction` endpoints.

### Joint state interfaces

| Group | Live topic | Type | Publisher | Publisher QoS |
|---|---|---|---|---|
| arms | `/aima/hal/joint/arm/state` | `aimdk_msgs/msg/JointStateArray` | `/hal_ethercat_x21436` | BEST_EFFORT, TRANSIENT_LOCAL |
| head | `/aima/hal/joint/head/state` | `aimdk_msgs/msg/JointStateArray` | `/hal_ethercat_x21436` | BEST_EFFORT, TRANSIENT_LOCAL |
| legs | `/aima/hal/joint/leg/state` | `aimdk_msgs/msg/JointStateArray` | `/hal_ethercat_x21436` | BEST_EFFORT, TRANSIENT_LOCAL |
| waist | `/aima/hal/joint/waist/state` | `aimdk_msgs/msg/JointStateArray` | `/hal_ethercat_x21436` | BEST_EFFORT, TRANSIENT_LOCAL |

Each topic had one publisher and two existing robot subscribers (`/mc_ros2_node2205` and `/soc0_drp_ros2_node1467`) at discovery time.

One-message reads were attempted, but the ROS2 CLI rejected all four because `aimdk_msgs/msg/JointStateArray` type-support is absent from the accessible `run` environment. Therefore no live array contents, lengths, names, positions, velocities, or efforts were decoded.

### IMU interfaces and observed messages

ROS standard `sensor_msgs/msg/Imu` confirms quaternion storage fields `x,y,z,w`, angular velocity units rad/s, and linear acceleration units m/s². It does not by itself establish the physical mounting transform, world-orientation reference, or gravity-removal policy.

| Topic | Publisher/type evidence | One-message `frame_id` | Orientation availability evidence | Acceleration norm in sample |
|---|---|---|---|---:|
| `/aima/hal/imu/chest/state` | `/soc0_hal_imu2985`, `sensor_msgs/msg/Imu` | `base_link` | quaternion populated; covariance all zero (unknown covariance) | 9.830 m/s² |
| `/aima/hal/imu/torso/state` | `/soc0_hal_imu2985`, `sensor_msgs/msg/Imu` | `base_link` | quaternion populated; covariance all zero (unknown covariance) | 9.833 m/s² |
| `/aima/hal/sensor/lidar_chest_front/imu` | live `sensor_msgs/msg/Imu` | `lidar_imu_chest_front` | identity quaternion; covariance all zero; availability cannot be inferred | 9.801 m/s² |
| `/aima/hal/sensor/rgbd_head_front/imu` | live `sensor_msgs/msg/Imu` | `camera_accel_gyro_optical_frame` | `orientation_covariance[0] = -1`; ROS contract says orientation estimate is unavailable | 9.665 m/s² |
| `/aima/hal/sensor/stereo_head_front/imu` | live `sensor_msgs/msg/Imu` | `stereo_imu_head_front` | identity quaternion; covariance all zero; availability cannot be inferred | 9.760 m/s² |

The lidar and stereo one-message reads emitted lost-message warnings before delivering a sample, consistent with the live stream/QoS behavior. This is an observation, not a calibrated loss-rate measurement.

## UNKNOWN

### Joint mapping and arrays

- Physical joint represented by every live array index.
- Live arm/head/leg/waist array lengths and whether the `name` field is blank on this firmware.
- Whether position, velocity, and effort members share exactly the documented group order on this installed release.
- Hardware joint/motor IDs, encoder zero, encoder offset, and positive direction.
- Any mapping from hardware sign/zero to MuJoCo coordinates.

The official AimDK order remains useful documentation evidence, but it was **not live-verified** in this phase and is not promoted to a confirmed hardware mapping.

### Effort / torque definition

`effort` remains `UNKNOWN`. The accessible evidence establishes only the advertised `JointStateArray` type and EtherCAT HAL publisher. Neither live-decoded message definitions nor publisher source were available. There is no basis to classify it as measured motor torque, estimated joint torque, commanded torque, motor current, normalized effort, or another quantity. No MuJoCo torque mapping was changed.

### AimDK and firmware details

- AimDK SDK workspace/install location accessible to `run`.
- AimDK SDK package version distinct from the AIMA CLI version.
- C++/Python AimDK SDK headers/modules for custom messages in the `run` rootfs.
- Firmware component versions beyond the login-banner Agi/OS image and NVIDIA L4T evidence.

### IMU semantics

- Orientation reference convention/world frame for chest and torso quaternions.
- Whether chest/torso angular velocity and linear acceleration have already been rotated into `base_link` or are merely labelled that way.
- Gravity inclusion/removal policy. The single-sample norms near 9.8 m/s² are consistent with gravity being present while stationary, but a single observation is not an API definition.
- Sensor biases, scale factors, axis signs, timing delay, timestamp clock relationship, and synchronization between IMUs and joint state.
- Calibrated transforms from physical IMU mounting frames to the robot base and MuJoCo `imu_0` site.

## NEEDS_PHYSICAL_VERIFICATION

- Confirm the robot was stationary and its actual pose during the captured IMU samples.
- Confirm the physical meanings and locations of “chest” and “torso” IMUs and validate their transforms against CAD/URDF/manufacturer calibration data.
- Obtain the exact `aimdk_msgs` package matching Agi release `v0.9.6`, then decode one joint-state message from each group without changing robot state.
- Verify joint index order using manufacturer documentation for this exact firmware/hardware revision, followed by a separately authorized low-risk physical identification procedure if documentation is insufficient.
- Obtain publisher/API/source evidence defining `JointState.effort` before treating it as torque or comparing it to MuJoCo actuator force.
- Verify physical emergency stop, current control mode, command ownership/arbitration, position limits, velocity limits, and torque limits before any later active test phase.

Phase 2A ends with all active-control safety gates still closed. No calibrated MJCF was created.
