# Phase 2H dynamics calibration gate

## Decision

**Gate level: `POSITION_ONLY_CALIBRATION_READY`**

**DYNAMICS_CALIBRATION_READY = NO**

This level permits only native-coordinate position/velocity diagnostics and replay-pipeline work. It does not authorize physical dynamics fitting, absolute hardware-to-MuJoCo alignment, torque calibration, or absolute base-attitude fitting.

## Readiness matrix

| Item | Status | Basis / consequence |
| --- | --- | --- |
| joint name/index mapping | **READY** | Live names and array indices are confirmed for all groups. |
| real position | **READY** | Decoded static and Phase 2D trajectories are available. |
| real velocity | **READY** | Decoded static and Phase 2D trajectories are available. |
| target-joint physical sign | **BLOCKED** | Mirror/control-coordinate evidence is not a physical hardware-to-MuJoCo sign proof. |
| target-joint physical zero | **BLOCKED** | No known physical pose plus measured JointState evidence; standing equilibrium is not encoder zero. |
| encoder offset | **BLOCKED** | No independent physical datum. |
| IMU comparison transform | **PARTIAL** | Literal frame labels and downstream use are known; raw axes, mounting rotation, and deployed transform are not. |
| reported effort unit | **READY** | Manufacturer label is torque in N·m. |
| reported effort semantics | **BLOCKED** | Measured/estimated/current-derived/commanded origin, formula, sign, and filtering remain UNKNOWN. |
| MC internal joint command | **UNOBSERVABLE** | This does not block output-response fitting by itself, but prevents actuator system identification claims. |
| simulation controller alignment | **PARTIAL** | Simulation-only candidates separate timing/equilibrium effects; none is hardware calibration. |
| contact baseline | **PARTIAL** | Simulation contact baseline exists; no equivalent real foot-force/contact measurement. |
| balance/absolute attitude baseline | **BLOCKED** | Relative observations exist, but the common IMU comparison frame is not confirmed. |

## Gate-level evaluation

| Gate | Result | Reason |
| --- | --- | --- |
| `FULL_DYNAMICS_CALIBRATION_READY` | **NO** | Physical sign/zero, IMU transform, and effort semantics are not closed. |
| `OUTPUT_RESPONSE_CALIBRATION_READY` | **NO** | MC internal command may remain unobservable, but physical sign/zero are still required for the selected fitted joints. |
| `POSITION_ONLY_CALIBRATION_READY` | **YES** | Native-coordinate position/velocity, relative motion, timing, settling, and repeatability can be analyzed without claiming a physical mapping. |
| `NOT_READY` | **NO** | Accepted real logs and a working replay/analysis pipeline support the restricted position-only scope. |

## Allowed work at the current gate

- Validate timestamp alignment, sampling rate, interpolation, delay, settling, repeatability, and relative position/velocity response.
- Compare fixed-base and free-base simulation controller behavior as simulation-only experiments.
- Report relative deltas in native hardware coordinates with explicit sign/zero caveats.
- Continue improving logging and observability without controlling the robot.

## Prohibited claims/actions at the current gate

- No mass, inertia, physical friction, gear, motor, actuator, or torque-limit fitting.
- No use of `JointState.effort` as measured/estimated torque.
- No absolute real-versus-simulation IMU axis or attitude fitting.
- No promotion of mirrored heart coordinates to physical sign.
- No promotion of `STAND_DEFAULT`, controller equilibrium, or static JointState values to encoder zero.
- No `ACTUATOR_SYSTEM_IDENTIFICATION`; MC internal joint command remains unobservable.
- No calibrated MJCF and no modification of UNKNOWN mapping fields.

## Minimum evidence to advance

### To `OUTPUT_RESPONSE_CALIBRATION_READY`

Confirm physical sign and zero for every selected fitted joint using a documented physical pose/axis convention plus passive measured JointState evidence. MC internal command may remain unobservable; the result must be called output-response fitting, not system identification.

### To include absolute base attitude

Confirm the deployed IMU raw/output frame and transform to the selected comparison frame. Until then, only independently baselined relative attitude and rotation-invariant gyro magnitude are admissible.

### To torque/actuator calibration

Confirm `JointState.effort` assignment source, physical location, conversion formula, sign, filtering, and whether it is measured, estimated, current-derived, or commanded torque.

## Final status

- `IMU_TRANSFORM = PARTIAL`
- `PHYSICAL_SIGN = UNKNOWN` for target joints
- `PHYSICAL_ZERO = UNKNOWN` for target joints
- `EFFORT_SEMANTICS = UNKNOWN`
- `MC_INTERNAL_COMMAND = UNOBSERVABLE`
- `POSITION_ONLY_CALIBRATION_READY = YES`
- `OUTPUT_RESPONSE_CALIBRATION_READY = NO`
- `FULL_DYNAMICS_CALIBRATION_READY = NO`
- `DYNAMICS_CALIBRATION_READY = NO`

All simulation controller candidates remain **NOT HARDWARE CALIBRATION**.
