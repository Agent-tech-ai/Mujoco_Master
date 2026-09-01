# Phase 3C JointState Effort Semantics Evidence

## Decision

`JointState.effort = UNKNOWN`

`EFFORT_SEMANTICS_READY = NO`

Reported effort must not be used for torque, gear, damping, friction or actuator identification.

## Confirmed boundary facts

- `aimdk_msgs/msg/JointStateArray` carries per-joint `position`, `velocity`, `effort`, voltage and temperature-related state fields.
- AimDK interface material labels the effort quantity in N·m.
- The live joint-state publisher boundary is the HAL EtherCAT process (`/hal_ethercat_x21443` in the captured audit).
- Values are observable in passive logs and respond during motion.

## What remains unknown

No captured source or official statement traces the published `effort` assignment to one of:

- directly measured joint torque;
- estimated joint torque;
- motor-current-derived torque;
- commanded torque;
- normalized effort;
- another internal quantity.

The source field, motor-side versus joint-side convention, gear conversion, torque constant, sign, zero bias, filtering, clipping and timestamp alignment are not documented by the evidence currently available. Message field names and N·m units do not distinguish these alternatives.

## Closure requirement

Required evidence is the deployed HAL/EtherCAT assignment chain or an official interface statement that identifies the source signal and its conversion formula, units, side of transmission, sign convention, filtering and saturation behavior. Until then, effort remains a descriptive logged channel only.

