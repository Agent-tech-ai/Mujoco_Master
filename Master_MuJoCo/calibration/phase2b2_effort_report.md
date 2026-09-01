# Phase 2B-2 effort report

Status: **PASSIVE SNAPSHOT ONLY — INSUFFICIENT_EVIDENCE**

AimDK documents `JointState.effort` in N·m. Its physical source remains `UNKNOWN`: no evidence establishes whether it is measured motor torque, estimated joint torque, commanded torque, motor-current-derived torque, normalized effort, or another quantity.

One passive arm-state snapshot showed the following values while the observed MC arm command had zero command effort:

| Joint | State effort (N·m as documented) |
|---|---:|
| left J2 shoulder roll | +0.732598 |
| right J2 shoulder roll | -1.025642 |
| left J7 wrist roll | +0.351650 |
| right J7 wrist roll | -0.164103 |

These values only establish a nonzero static reading under the current standing controller. Without a controlled command transition, they do not establish effort response sign, peak behavior, return behavior, or torque source.

No effort-based mapping field was upgraded.

