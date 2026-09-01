# Phase 2B-2 preflight report

Status: **NO-GO — no real-robot motion command was sent**

Evidence time: 2026-08-12 16:34:49–16:35:40 EDT, as reported by the robot host.

Target: `run@192.168.4.114`

Raw local evidence in the workspace: `../../work/x2_phase2b2_preflight.txt`. The delivery bundle stores a copy as `evidence/x2_phase2b2_preflight.txt`.

## Read-only scope actually executed

The preflight read robot identity, ROS interfaces, topic endpoint metadata, one existing arm-command message, selected state messages, process/ROS graph data, and parameter names. It called only these query services:

- `/aimdk_5Fmsgs/srv/GetCurrentInputSource`
- `/aimdk_5Fmsgs/srv/GetMcAction`
- `/aimdk_5Fmsgs/srv/GetSystemState`

The first two returned; `GetSystemState` timed out. No retry was made. The preflight contained no topic publisher, state-changing service/action call, control-mode change, actuator operation, process control, package installation, or configuration write. Its completion markers report `MOTION_COMMAND_SENT=0` and `MOTION_AUTHORIZED_BY_THIS_SCRIPT=0`.

## CONFIRMED

- Host identity: `agi`; Ubuntu 22.04.5 LTS; Linux `5.15.148-tegra`; `aarch64`; user `run`.
- The target owns `192.168.4.114` and also reported `10.0.1.41` among its addresses.
- ROS 2 Humble executable: `/opt/ros/humble/bin/ros2`.
- Existing AimDK message prefix: `/agibot/software/common`.
- `/aima/hal/joint/arm/command` had exactly one publisher, `mc_ros2_node2263`, and one subscriber, `hal_ethercat_x21455`.
- The command publisher was actively sending a complete 14-joint `JointCommandArray` in `STAND_DEFAULT`. The observed command values for all four proposed test joints were approximately zero position/velocity/effort. Observed controller stiffness/damping were 50/3 for J2 and 5/0.5 for J7.
- `GetMcAction` returned header code 0 and `action_desc="STAND_DEFAULT"`. The numeric `status=100` is retained without interpretation because the installed interface display for the nested action type failed to parse.
- `GetCurrentInputSource` returned header code 0, but the payload was `name=""`, `priority=0`, `timeout=0`, with task state value 0 (`UNKNOWN` in the installed `CommonState` definition). Official AimDK documentation says the current input source can be empty before MC has received an effective input. An empty source therefore does **not** prove that the HAL command topic is free.
- `/aima/mc/common/state` independently reported `action_desc: STAND_DEFAULT` and an empty input source.
- `/aima/sm/system_state` reported `cur_state: Business`, `cur_status.value: 1`. No system-state change was requested.
- One live arm state snapshot contained all 14 named joints and their position, velocity, effort, coil temperature, motor temperature, and motor voltage fields.

## Current J2/J7 snapshot and limit margin

The position snapshot below is read-only evidence, not a zero/encoder/sign calibration. Margins use the operator-supplied `FIELD_TEST_EVIDENCE` arm limits.

| Joint | Measured position | Field limits | Relevant requested target | Remaining margin at that target | Result |
|---|---:|---:|---:|---:|---|
| left J2 shoulder roll | -0.939315° | -3.495° to +171.486° | current - 2° = -2.939315° | 0.555685° above lower limit | NO-GO: too close without an approved safety margin |
| right J2 shoulder roll | +1.093131° | -171.486° to +3.495° | current + 2° = +3.093131° | 0.401869° below upper limit | NO-GO: too close without an approved safety margin |
| left J7 wrist roll | -4.382304° | -90.012° to +41.482° | current ± 2° | at least 43.864304° | Range margin alone passes; other gates fail |
| right J7 wrist roll | +1.693887° | -41.482° to +90.012° | current ± 2° | at least 41.175887° | Range margin alone passes; other gates fail |

The requested symmetric `+2° / return / -2° / return` sequence therefore violates the instruction not to test near a limit for both J2 joints at the observed pose. This preflight does not substitute a smaller or one-sided trajectory without a new, explicit test decision.

## Active safety-gate status

| Required gate | Status | Evidence and decision |
|---|---|---|
| Emergency stop available | NEEDS_PHYSICAL_VERIFICATION | Cannot be established over SSH. |
| Current control mode | CONFIRMED_SNAPSHOT_ONLY | `STAND_DEFAULT` at the evidence timestamp; must be rechecked immediately before any future test. |
| Joint-command arbitration | NO-GO | MC is an active publisher directly on the HAL arm-command topic. Empty MC input-source name does not remove that publisher. |
| No upper-level task/controller | NO-GO | `mc_ros2_node2263` is actively publishing the full arm command. |
| Numeric velocity limit | UNKNOWN | No installed-firmware-specific approved test limit was found. The 1 rad/s value in an SDK example is an example trajectory setting, not accepted as the robot's safe limit. |
| Numeric position limit | FIELD_TEST_EVIDENCE | Limits exist, but the two J2 requested targets have less than 0.56° margin. |
| Torque/effort protection | UNKNOWN | No matching protection parameter was exposed by read-only parameter-name discovery; threshold and trip behavior remain unknown. |
| Operator beside robot | NEEDS_PHYSICAL_VERIFICATION | Requires execution-time confirmation. |
| Clear surroundings/swept volume | NEEDS_PHYSICAL_VERIFICATION | Requires execution-time confirmation. |
| Command path ownership procedure | NO-GO | Official direct-HAL example requires stopping native MC first. No process stop or alternative vendor-supported handoff was authorized. |

Additional observations that are **not** sufficient to pass a gate:

- One command snapshot showed the controller's current gains, but this does not establish that those values may be copied into a second publisher or used after a control-ownership handoff.
- `/aima/hds/alert_code_list` produced no message within five seconds. Timeout is not evidence that no alert exists.
- `JointStateArray.state.value` was zero. AimDK documents nonzero meanings but treats other values as N/A; zero is not promoted to a general “all safe” assertion.

## Official interface constraints

- AimDK documents MC input arbitration as selecting the highest-priority valid input and recommends registering a secondary-development source; the empty-source condition has a documented benign case.
- The official direct joint-control example operates on the HAL command topic and explicitly instructs the operator to stop native MC on PC1 before running it to obtain control ownership.
- The official joint-control interface uses a full group array. For arms, it contains 14 entries, left seven first then right seven, with position, velocity, effort, stiffness, and damping fields.

Sources:

- https://x2-aimdk.agibot.com/zh-cn/latest/Interface/control_mod/MC_control.html
- https://x2-aimdk.agibot.com/zh-cn/latest/example/Python.html#id15
- https://x2-aimdk.agibot.com/en/latest/Interface/control_mod/joint_control.html

## Decision and data-integrity impact

Phase 2B-2 stops at the safety gate. No `j2_left.csv`, `j2_right.csv`, `j7_left.csv`, or `j7_right.csv` was created because no physical motion occurred; creating zero-row or synthetic “real” logs would be misleading. No real/simulation overlay was produced.

`joint_mapping.csv` was not changed: none of the four mappings was physically exercised, so hardware sign, hardware zero, and encoder offset remain `UNKNOWN`. No MJCF was changed and no calibrated MJCF was created.

## Evidence still required for a future GO decision

1. Physical E-stop verification, operator presence, and clear swept volume.
2. A vendor-supported control-ownership procedure compatible with the installed firmware that does not leave MC and the test node publishing to the same HAL topic.
3. Explicit authorization if that supported procedure requires stopping MC or changing system/control mode.
4. Numeric test velocity/acceleration limits and numeric effort stop threshold, with source and units.
5. Confirmation of torque/effort protection threshold and trip behavior.
6. A revised J2 test trajectory with an approved mechanical margin; the current symmetric ±2° sequence is not acceptable at this pose.
7. Immediate pre-motion recheck of mode, arbitration, current positions, temperatures, voltage, and error/alert state.
