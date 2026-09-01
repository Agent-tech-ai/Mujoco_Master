# Phase 2H JointState effort semantics closure

Final classification: **UNKNOWN**.

The unit/label is confirmed as torque in N·m, but the producing algorithm and physical origin are not confirmed. A unit label is not sufficient to classify the field as measured, estimated, commanded, or current-derived torque.

## Evidence chain

| Layer | Confirmed evidence | Interpretation limit |
| --- | --- | --- |
| Live ROS graph | `/hal_ethercat_x21443` publishes `/aima/hal/joint/{arm,head,leg,waist}/state` as `aimdk_msgs/msg/JointStateArray`. | Identifies the publisher boundary, not the assignment source. |
| Message schema | `JointStateArray` contains `JointState[] joints`; each live/installed `JointState` contains `effort`. | A field name/type has no physical provenance. |
| Manufacturer documentation | `JointState.effort` is documented as torque, unit N·m, in a joint-state feedback interface. | Does not state motor-side vs joint-side, measured vs estimated vs current-derived, sign, filtering, or conversion. |
| Official RL example | `jointCallback` copies `joint.effort` unchanged into its internal state. | This is a consumer and does not expose EtherCAT/HAL assignment. |
| Agentech code/docs | Records the value as `reportedEffort`/measured effort and uses it for safety/qualitative response checks. | Downstream naming does not prove upstream semantics. |
| Command path | Example commands often set `JointCommand.effort = 0.0` while native MC retains gravity support and actuator effort. | Confirms command effort and state effort must not be conflated; does not identify the state estimator. |
| Real trajectory | Values are nonzero and change with pose/motion. | Correlation does not prove source, conversion, or sign. |

## Required classification

| Candidate | Result | Reason |
| --- | --- | --- |
| `MEASURED_TORQUE` | not established | No torque-sensor/HAL assignment or sensor location found. |
| `ESTIMATED_TORQUE` | not established | No estimator source or formula found. |
| `CURRENT_DERIVED_TORQUE` | not established | No motor-current field, torque constant, gear ratio, efficiency model, or conversion path found. |
| `COMMANDED_TORQUE` | not established | No state-to-command copy found; observed command examples use zero additive effort. |
| `OTHER` | not established | No alternative physical definition found. |
| `UNKNOWN` | **selected** | Only field, unit, publisher boundary, and downstream pass-through are confirmed. |

## Semantic fields

- Source field: `aimdk_msgs/msg/JointState.effort`.
- Advertised physical quantity/unit: torque, N·m.
- Actual source field inside EtherCAT/motor data: **UNKNOWN**.
- Motor-side or joint-side: **UNKNOWN**.
- Conversion formula: **UNKNOWN**.
- Gear/efficiency compensation: **UNKNOWN**.
- Sign convention: **UNKNOWN**.
- Filtering and delay: **UNKNOWN**.
- Measured/estimated/commanded/current-derived classification: **UNKNOWN**.

## Calibration consequence

The field may continue to be plotted as **reported effort (N·m label)** and used for qualitative response/anomaly detection. It must not be used as a torque target or residual for actuator, friction, motor, or torque-limit calibration.

Torque/actuator calibration remains blocked until the HAL publisher implementation, a binary/API contract, or manufacturer documentation provides the assignment source, formula, sign, and filtering semantics.
