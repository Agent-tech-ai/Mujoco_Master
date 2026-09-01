# Phase 2B-2 operator preflight confirmation

Status: **NO-GO**

Robot SSH target: `run@192.168.4.114`

This record does not authorize any robot motion. Fill physical items only from direct operator observation and technical items only from installed-firmware or vendor-supported evidence. Do not infer entries from MuJoCo.

| Required confirmation | Status | Recorded evidence / required value |
|---|---|---|
| Physical emergency stop is present, reachable, and known working | UNCONFIRMED | Operator must record the last check method and time. |
| Current robot control mode | CONFIRMED_SNAPSHOT_ONLY | `GetMcAction` returned `STAND_DEFAULT` at 2026-08-12 16:35 EDT; recheck immediately before any future motion. |
| Current joint-command arbitration/input source | NO-GO | Input-source name was empty, but `mc_ros2_node2263` was actively publishing the full arm HAL command. |
| No upper-level task/controller will issue or preempt commands | NO-GO | Native MC is an active command publisher. |
| Numeric velocity/acceleration limits approved for this test | UNCONFIRMED | Need values, units, joint scope, firmware/source, and selected conservative test values. |
| Numeric position limits and live margin approved for J2/J7 | NO-GO_FOR_REQUESTED_J2_SEQUENCE | At the snapshot, left J2 current-2° leaves 0.555685° lower margin; right J2 current+2° leaves 0.401869° upper margin. |
| Torque/effort protection active; threshold and behavior known | UNCONFIRMED | Need source, threshold, units, and stop behavior. |
| Operator is physically beside the robot for every motion | UNCONFIRMED | Operator statement required at execution time. |
| Robot surroundings and swept volume are clear | UNCONFIRMED | Operator statement required at execution time. |
| Vendor-supported command ownership procedure approved | NO-GO | Official direct HAL example says native MC must be stopped. That operation was not authorized or performed. |

Additional requirements:

- Define how all 14 arm command entries are held while one joint moves.
- Establish safe command stiffness/damping values for the actual ownership mode. A passively observed MC command is not authorization to reuse those values.
- Define numeric automatic stop thresholds for velocity and effort. Qualitative terms such as “low” or “large” are insufficient.
- Approve a revised J2 trajectory that remains outside a documented safety margin.

GO rule: every required row must be `CONFIRMED`, all numeric limits must have units and sources, current measured position must have approved margin to both limits, the command path must be vendor-supported for the installed firmware, and the operator must explicitly authorize exactly one joint/one round. Otherwise status remains **NO-GO**.

Do not place passwords, private keys, or other SSH credentials in this file.

