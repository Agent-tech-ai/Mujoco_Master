# Phase 2B-2 operator GO/NO-GO checklist

Current result: **NO-GO**

Rule: every critical item must be `CONFIRMED` from current physical observation, installed-firmware evidence, or an approved vendor procedure. Any `UNKNOWN`, stale snapshot, conflict, or failure makes the result `NO-GO`.

| Critical item | Current status | Evidence required before GO |
|---|---|---|
| E-stop has been physically tested | UNKNOWN | Operator, method, result, timestamp |
| Operator is beside robot | UNKNOWN | Execution-time confirmation |
| Workspace and complete swept volume are clear | UNKNOWN | Execution-time confirmation |
| Current robot pose is mechanically stable | UNKNOWN | Operator plus live state/IMU/error evidence |
| Native-MC ownership handoff plan approved | NO-GO | AgiBot/vendor-supported direct-HAL procedure for installed firmware |
| Arm joint-command source is unique | NO-GO_SNAPSHOT | Saved preflight had one native-MC publisher; future test requires live count exactly zero before test publisher exists |
| Numeric joint velocity limit known | UNKNOWN | Value, units, source, applicable joints, approved test value |
| Numeric joint acceleration limit known | UNKNOWN | Value, units, source, applicable joints, approved test value |
| Position safety margin known | ENGINEERING_SCREEN_ONLY | Field limits exist; 5° reserve is only an offline screen, not vendor-approved |
| Torque/effort protection known | UNKNOWN | Threshold, units, physical signal semantics, trip behavior, source |
| Abort command/strategy known | UNKNOWN | Tested response and approved hold/disable behavior |
| Behavior after loss of SSH/ROS/state/command communications known | UNKNOWN | Controller/HAL timeout behavior and physical validation |
| Test-end control restoration steps known | UNKNOWN | Authoritative MC restart, ownership cleanup, mode restoration, and verification procedure |
| All 14 command stiffness values approved | UNKNOWN | Numeric array and source; do not copy observed MC values without approval |
| All 14 command damping values approved | UNKNOWN | Numeric array and source; do not copy observed MC values without approval |
| Current command publisher graph rechecked | STALE | Must be read live immediately before publisher creation |
| Current mode/input source rechecked | STALE | Must be read live immediately before any ownership change |
| Current joint states, margin, temperatures, voltage, errors rechecked | STALE | Must pass immediately before each joint/round |
| Exactly one joint and one round explicitly authorized | UNKNOWN | Named joint, selected delta, operator, timestamp |

## Machine-readable gate

The companion file `active_tests/phase2b2_operator_gate.json` is intentionally `NO-GO` with null numeric limits and false confirmations. The active-test program refuses `--enable-motion` before importing ROS while any entry remains incomplete.

Editing the JSON is not itself approval. Values must be copied from signed/operator or vendor evidence, reviewed, and recorded here. Passwords and private keys must never be recorded.

## Execution-time stop conditions

Stop the current test without automatic retry if direction conflicts, velocity or effort thresholds are exceeded, base/IMU change exceeds the approved threshold, another publisher appears, state becomes stale, error state changes, an upper controller appears, or the operator judges the situation unsafe.

The current checklist cannot authorize motion.

