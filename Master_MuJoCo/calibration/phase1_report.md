# X2 Ultra MuJoCo calibration — phase 1 report

## Confirmed facts

- The supplied project was reused from `Master_MuJoCo.zip`; it was not recreated from scratch.
- `assets/Master/ff_master_ultra.xml` SHA-256 remains `89619295fcc372c57473224130865b2fe4f22e0741f72925fac243805f4353db`.
- The original model compiles with 32 joints (one free-base joint plus 31 hinge joints), 31 actuators, and 67 sensors.
- `ff_master_ultra_x2_limits.xml`, `scene_x2_fixed.xml`, and `scene_x2_free.xml` each compile with 31 joints (one free-base joint plus 30 hinge joints), 30 actuators, and 65 sensors. Head pitch is a fixed link in this derived model.
- The fixed scene adds a pelvis weld constraint. The free scene has no pelvis weld. Both include the same derived X2-limit model.
- The derived model supplies joint position and velocity sensors for each of its 30 hinge joints, plus body orientation, angular velocity, linear position, linear velocity, and linear acceleration sensors at MuJoCo site `imu_0`.
- The AimDK X2 1.0.0 documentation publishes arm/leg/head/waist motion envelopes and group array order. It documents joint state position/velocity/effort in rad, rad/s, and N·m.
- The current MJCF damping, armature, friction, motor control ranges, actuator force ranges, inertias, contact parameters, and controller gains have not been validated against a real X2.
- `export_sim_log.py` exports MuJoCo command/position/velocity/direct-drive actuator force and IMU readings using the shared schema. The physical meaning of robot `effort` versus MuJoCo `actuator_force` is not yet established.

## Parameters still UNKNOWN

- Hardware motor/joint IDs.
- Hardware-reported joint name strings (the documented `JointState.name` is currently unused).
- Hardware encoder zero and encoder offsets.
- Hardware positive direction for every joint and therefore the validity of every mirrored/reversed MJCF assumption.
- Exact interpretation of the documented symmetric/asymmetric limits on the left and right hardware coordinates.
- True joint damping, armature/reflected inertia, Coulomb friction, stiction, backlash, torque constants, saturation, and controller gains.
- Whether state `effort` on the installed robot is measured torque, estimated torque, or another effort signal.
- Timestamp clock/epoch and transport delay for each real topic.
- IMU frame, quaternion ordering confirmation, gravity convention, sensor biases, scale factors, and latency on the actual robot.
- Robot hostname/IP/user, OS, installed ROS distribution, live nodes/topics, AimDK path/version, live joint/IMU topics, emergency-stop state, control mode, velocity limits, and per-joint torque limits.

## Inputs needed for real calibration

1. Read-only SSH host/IP and username or an exported discovery bundle containing `hostnamectl`, OS release, ROS environment, node/topic lists, topic types/QoS, relevant message definitions, and running robot/AimDK processes.
2. Installed AimDK/firmware version and exact X2 Ultra hardware revision.
3. A safe, stationary real log using `log_schema.md`, plus metadata for clocks, group order, IMU topic/frame, quaternion order, gravity convention, and effort semantics.
4. Verified manufacturer or robot configuration data for joint IDs, encoder zeroing, direction, velocity limits, and torque limits. Do not infer these from the existing MJCF.
5. Later, only after the safety gate: operator-supervised single-joint low-amplitude logs at several frequencies/load cases, with the robot supported and limits independently enforced.

## Model audit caution

The existing X2-limit derivative changes official degree limits into per-side MJCF coordinates using several axis-reversal and mirror assumptions. Those values are useful hypotheses for comparison, not confirmed truth. `joint_mapping.csv` therefore preserves official and MJCF limits side by side while leaving `sign=UNKNOWN`.

## Sources

- Base model: <https://github.com/Faraday-Future-AI/Robothon-starter/tree/main/assets/Master>
- X2 Ultra documented joint envelopes: <https://x2-aimdk.agibot.com/zh-cn/latest/about_agibot_X2/joint_name_and_limit.html>
- AimDK joint state/command interface: <https://x2-aimdk.agibot.com/en/latest/Interface/control_mod/joint_control.html>
- AimDK sensor/IMU interface: <https://x2-aimdk.agibot.com/zh-cn/latest/Interface/hal/sensor.html>
