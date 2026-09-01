# X2 robot interface report

Date: 2026-08-11  
Discovery mode: read-only only

## Local SSH availability

- Windows OpenSSH client is installed (`OpenSSH_for_Windows_9.5p1`).
- The current process is not an SSH session (`SSH_CONNECTION`, `SSH_CLIENT`, and `SSH_TTY` were unset).
- No robot host, IP address, SSH username, or workspace SSH reference was available.
- Access to `C:\Users\xinga\.ssh` was denied by the execution environment even after a scoped read request, so no configured host alias could be confirmed.
- Therefore no SSH connection was attempted and none of `hostname`, OS, ROS/ROS2 nodes/topics, processes, SDK paths, or live message types was observed on the robot.

## Live robot findings

| Requested item | Live result |
|---|---|
| hostname | `UNKNOWN` |
| OS | `UNKNOWN` |
| ROS / ROS2 installed | `UNKNOWN` |
| ROS2 node list | `UNKNOWN` |
| ROS2 topic list | `UNKNOWN` |
| running robot processes | `UNKNOWN` |
| AimDK / AgiBot SDK installation path | `UNKNOWN` |
| emergency-stop state/interface | `UNKNOWN` |
| current control mode | `UNKNOWN` |
| hardware joint IDs | `UNKNOWN` |
| velocity limits | `UNKNOWN` |
| torque limits | `UNKNOWN` |

## Documented candidates — not live-verified

These are from AimDK X2 1.0.0 online documentation and must be verified with `ros2 topic list`, `ros2 topic info`, and message inspection on the actual robot/firmware.

| Data | Documented candidate interface | Status |
|---|---|---|
| joint position | `/aima/hal/joint/{head,arm,waist,leg}/state`, `hal/msg/JointStateArray.joints[].position` | documented; live `UNKNOWN` |
| joint velocity | same state topics, `.joints[].velocity` | documented; live `UNKNOWN` |
| joint torque/effort | same state topics, `.joints[].effort` | documented; whether measured or estimated is `UNKNOWN` |
| joint state query | `/aimdk_5Fmsgs/srv/GetAllJointState` | documented; live `UNKNOWN` |
| chest IMU | `/aima/hal/imu/chest/state` (`sensor_msgs/msg/Imu`, documented 500 Hz) | documented; live `UNKNOWN` |
| pelvis/torso IMU | `/aima/hal/imu/torso/state` (`sensor_msgs/msg/Imu`, documented 500 Hz) | documented; naming/frame live `UNKNOWN` |
| command interface | `/aima/hal/joint/{head,arm,waist,leg}/command`, `hal/msg/JointCommandArray` | documented only; **must not be used yet** |
| motion-mode query | `/aimdk_5Fmsgs/srv/GetMcAction` | documented only; live `UNKNOWN` |

The documentation says `JointState.name` is currently unused; joint meaning is determined by group array order. `joint_mapping.csv` records that documented group/order without claiming it is a hardware motor ID.

## Safety gate

No command publisher or mode-switch call is implemented in this phase. Before any motion command, independently confirm the physical emergency stop, current control mode, command API on the installed firmware, joint limits, velocity limits, torque limits, support/fixture state, and an operator-approved test procedure.

## Documentation sources

- Joint state/command API: <https://x2-aimdk.agibot.com/en/latest/Interface/control_mod/joint_control.html>
- IMU topics: <https://x2-aimdk.agibot.com/zh-cn/latest/Interface/hal/sensor.html>
