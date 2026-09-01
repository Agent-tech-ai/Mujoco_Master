# Calibration changelog

## 2026-08-12 — Phase 2C MC-compatible control-path discovery

- Completed static, read-only discovery on `agentech01`; no remote file was
  written, no target module imported, no ROS/SDK control call made, and no
  robot/process/mode state changed.
- Traced default standing `Agentech.heart(both)` to native AimDK
  `SetMcPresetMotion(area=3, motion=1007, interrupt=False)`, with
  `STAND_DEFAULT` checks before and after the task.
- Confirmed the standing route does not create a direct-HAL publisher, stop
  native MC, switch control mode, or call `SetMcInputSource`.
- Identified the separate seated controller as direct HAL: it publishes all
  joint command groups and explicitly stops native MC. It is excluded from an
  MC-preserving standing calibration test.
- Found no approved production MC-native arbitrary J2/J7 single-joint command
  path. Future real active tests remain NO-GO.
- Added six Phase 2C reports and a standard-library-only dry-run plan printer.
  No MJCF, mapping sign/zero/offset, dynamics, actuator, or robot configuration
  was modified.

## 2026-08-12 — MuJoCo standing-stability cleanup (SIMULATION ONLY)

- Reproduced 10-second free-zero, free-default, fixed-default, and free-cleanup baselines, recording base state, CoM, contacts/forces, foot slip, joint error, actuator force, and saturation.
- Established the primary root cause as missing free-base pelvis/balance feedback: fixed base remained stable, while friction, Kp, Kd, timestep, initial-height, and box-foot single-factor variants all retained the deterministic forward fall.
- Audited the 43.474797 kg mass/inertia model and exact convex hulls of the discrete foot-contact sphere centers. Initial CoM is inside the combined support polygon with 0.090032 m minimum boundary margin. No gross mass/inertia fault was found.
- Identified foot initialization (lowest spheres 5.05 mm above the plane) and discrete 5 mm-radius contact points as secondary contributors. Neither lowering the initial base nor substituting box contacts alone fixed standing.
- Added `SimulationStabilityController`, an explicitly simulation-only pelvis roll/pitch ankle-feedback layer plus smooth compensation for the MJCF's existing frictionloss deadband. It is not a hardware controller or calibrated parameter set.
- The free-base cleanup stood for 10 s with 2.100° maximum tilt, 0.0145 m base displacement, <0.15 mm per-foot slip, zero sustained actuator saturation, and both-foot contact for 99.68% of timesteps (the gap is initial contact acquisition).
- Reran all 12 fixed-base active-test rehearsals. All changed from `TRACKING_NOT_SETTLED` to `SETTLED`, with no modeled self-collision, limit violation, or actuator saturation.
- Preserved all MJCF files and `joint_mapping.csv`; did not create a calibrated or hardware-matched model. No SSH, ROS, or robot interface was used.

## 2026-08-12 — Phase 2B-2 offline preparation (NO ROBOT MOTION)

- Used the latest saved arm snapshot and operator-supplied field limits to compute directional ±1°, ±2°, ±3°, and ±5° margins for all 14 arm joints. The 5° reserve is an engineering screening value, not a vendor-approved safety margin.
- Ranked active-test candidates without assuming J2/J7 priority. Both shoulder-roll J2 joints are skipped for symmetric tests at the saved pose; wrist-roll J7 is the leading candidate, followed by wrist yaw/pitch.
- Proposed but did not command a neutral arm-pose candidate. A 201-sample MuJoCo kinematic interpolation reported no modeled contacts or MJCF limit violations, subject to the documented collision-model and hardware-coordinate limitations.
- Added an operator checklist and control-ownership plan. Status remains `NO-GO` because physical E-stop verification, approved ownership transfer/recovery, numeric velocity/acceleration/effort protections, abort behavior, and communication-loss behavior are not all confirmed.
- Added a dry-run-first active-test script. Default dry-run uses saved evidence and cannot import ROS, create a publisher, or send a command. Motion mode requires explicit `--enable-motion`, a complete machine-readable gate, and two interactive confirmations; the supplied gate intentionally blocks it.
- Rehearsed adaptive current/+delta/return/-delta/return sequences for 12 accepted joints in fixed-base MuJoCo. Both J2 joints were skipped. No modeled contacts or limits were violated, but tracking did not settle with the unchanged model and free-base stability was not demonstrated.
- Did not connect to the robot, publish, call a service/action, change mode, operate MC/actuators/configuration, modify `joint_mapping.csv`, modify an MJCF, or create a calibrated model.

## 2026-08-12 — Phase 2B-2 read-only active-test preflight

- Updated the SSH target to `run@192.168.4.114` and executed an audited read-only preflight. It used subscriptions, graph/interface reads, and three query-only Get services; no command was published and no state-changing service/action, mode change, actuator/process operation, or configuration change occurred.
- Confirmed active native-MC publication from `mc_ros2_node2263` to the full 14-joint arm HAL command topic, with `hal_ethercat_x21455` subscribing. `GetMcAction` and MC state reported `STAND_DEFAULT`.
- `GetCurrentInputSource` returned an empty name while native MC remained an active command publisher. Per official documentation, an empty source can occur before an effective input and is not evidence of exclusive command ownership.
- Stopped at NO-GO because command ownership, E-stop/operator/clearance, numeric velocity and effort thresholds, and torque-protection behavior were not confirmed. Official direct-HAL examples require stopping native MC, which was not authorized or performed.
- The observed pose also made the requested J2 symmetric ±2° sequence too close to the field limits: approximately 0.556° remaining for left J2 at -2° and 0.402° for right J2 at +2°.
- Created preflight, sign, zero-offset, effort, and real-vs-sim status reports. Active-test CSVs were intentionally not created because no physical test occurred.
- Did not update `joint_mapping.csv`, modify any MJCF, create a calibrated model, or perform dynamics calibration.

## 2026-08-11 — Phase 2B-1 JointStateArray decoding and static mapping

- Located the existing `aimdk_msgs` runtime at `/agibot/software/common` from the read-only environment and memory-map evidence of a running `run`-owned ROS process. No package was installed and no system environment was modified.
- Confirmed the live `JointStateArray`, `JointState`, `MessageHeader`, and `DomainErrorState` schemas and retained raw `ros2 interface show` plus four `echo --once` outputs.
- Confirmed live group lengths and stable names for all 31 indices: arm 14, head 2, leg 12, waist 3. Thirty live names exactly match current MuJoCo names; `head_pitch_joint` remains absent/fixed in the X2-limit MJCF.
- Captured approximately 30 seconds from four joint-state and chest/torso IMU topics using subscriptions only: 8,259 serialized topic samples and 45,659 unified CSV rows.
- Added operator-supplied arm limits and J2/J7 left/right mirrored control-coordinate observations as `FIELD_TEST_EVIDENCE`, separate from official and MuJoCo limits.
- Generated the JointStateArray schema, arm coordinate evidence, static analysis, and hardware-vs-MuJoCo arm reports; updated mapping cells with explicit evidence sources.
- AimDK documents `JointState.effort` as torque in N·m, but measured/estimated/commanded/current-derived physical origin remains `UNKNOWN`.
- Hardware joint IDs, zeros, signs, and encoder offsets remain `UNKNOWN`. No mapping was inferred from similar static positions.
- Did not publish any topic, call any service/action, change control mode, operate an actuator/process/configuration, modify an MJCF, or create a calibrated model.

## 2026-08-11 — Phase 2A read-only robot discovery

- Connected to `run@192.168.4.66` using operator-entered password authentication and executed only pre-audited read-only discovery scripts.
- Confirmed X2 Ultra SoC1 system/firmware banner information, Ubuntu/Jetson platform, ROS 2 Humble graph, live joint-state topic endpoints, and five IMU streams.
- Read exactly one message from each allowlisted IMU stream. Joint-state one-message reads could not be decoded because `aimdk_msgs` type-support is absent from the accessible `run` environment.
- Updated `joint_mapping.csv` notes only with confirmed live state-topic locations. Hardware IDs, live array indices, zero/sign/encoder parameters, and effort semantics remain `UNKNOWN`.
- Added `robot_interface_report_phase2.md` and retained raw discovery evidence.
- Did not publish any topic, call any service/action, change control mode, operate an actuator/process/configuration, modify MuJoCo dynamics, or create a calibrated model.

## 2026-08-11 — Phase 1 environment

- Preserved `assets/Master/ff_master_ultra.xml` without modification.
- Recorded extracted baseline SHA-256: `89619295fcc372c57473224130865b2fe4f22e0741f72925fac243805f4353db`.
- Audited the existing derived X2-limit model and fixed/free scenes; no existing dynamic, friction, actuator, zero, or direction parameter is accepted as real-robot calibrated.
- Added a report-only model inspector, shared log schema, explicit joint mapping with unknown hardware fields, static/single-joint analyses, and real/simulation comparison plots.
- Did **not** create `ff_master_ultra_calibrated.xml`: no real X2 log or verified zero/sign/encoder data was available, so naming any parameter set “calibrated” would be misleading. Future parameter changes must be made in that or another clearly derived MJCF and logged here.
- No robot motion command was sent.
