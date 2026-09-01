# Phase 3C MC Command Observability

## Decision

`MC_INTERNAL_COMMAND = UNOBSERVABLE`

`MC_COMMAND_OBSERVABLE = NO`

This does not block output-response fitting, but it prevents actuator/system identification from being claimed.

## Observable interfaces

- MC/common robot state and preset execution state are readable.
- Measured joint state, velocity, reported effort and IMU topics are readable.
- High-level preset requests and soft-engineer SDK call chains identify the motion request entering the MC-compatible path.

## Missing interface

No already-running read-only topic, service response, debug stream or SDK API was verified to expose the time-indexed per-joint reference actually used inside MC after interpolation, arbitration, balance blending, clamps and safety processing.

Potential HAL command topics are not accepted as proof of the internal MC target unless their ownership, location in the pipeline and semantics are documented. No debug mode was enabled and no configuration was changed.

## Consequence

Real measured trajectory → simulation response comparison remains `OUTPUT_RESPONSE` work. Without the internal MC joint reference, it is not `ACTUATOR_SYSTEM_IDENTIFICATION` and cannot uniquely separate preset generation, MC blending, actuator tracking and physical dynamics.

