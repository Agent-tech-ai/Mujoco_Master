# Phase 3D Open Hardware-Evidence Blockers

## Gate summary

| Blocker | State | Prevents |
|---|---|---|
| Physical joint sign | `UNKNOWN` | Absolute hardware↔MuJoCo coordinate fitting |
| Physical zero / encoder offset | `UNKNOWN` | Absolute posture/equilibrium and range calibration |
| Mass magnitude / CoM / inertia | `INSUFFICIENT_EVIDENCE` | Evidence-backed physical model update |
| IMU transform | `PARTIAL` | Absolute base-attitude and axis-specific IMU fitting |
| Effort semantics | `UNKNOWN` | Torque/actuator/friction/gear calibration |
| MC internal command | `UNOBSERVABLE` | Actuator/system identification |
| Real contact/force baseline | `PARTIAL` | Physical contact/friction/slip calibration |
| Command/state clock and latency | `PARTIAL` | Unique delay/bandwidth identification |
| Hardware actuator and safety limits | `PARTIAL/UNKNOWN` | Safe controlled excitation and actuator-limit identification |

## 1. Joint coordinate closure

Live names and indices are ready, but physical positive rotation is not proven for the selected shoulder roll, wrist yaw/roll, hip pitch, knee, ankle pitch and waist pitch joints. Multi-joint preset mirroring, same-name mapping, symmetric ranges and current MJCF axes cannot close sign.

No manufacturer-defined physical zero or calibration fixture pose has been tied to measured `JointState.position`. `STAND_DEFAULT`, a controller equilibrium and a static encoder reading are not hardware zero. Encoder offsets remain unknown.

Required evidence: manufacturer/deployed axis-frame conventions plus a known physical pose/direction observed in read-only state, with exact joint, firmware, units and configuration recorded.

## 2. Physical inertial properties

The project lacks independent X2 Ultra per-link or assembly mass, CoM and inertia tensors for the exact payload/end-effector configuration. AimDK simulator source confirms model lineage only. The cross-motion `bs_mass_lower_plus08` result does not identify mass as the unique cause or `+8%` as the real magnitude.

The pelvis mesh/URDF discrepancy must remain unresolved until an independent source establishes the physical assembly mass and CoM. Correcting a conversion inconsistency based only on another same-family model would still be an assumption.

Required evidence: CAD mass-property export or manufacturer table with link frames, configuration/BOM, units, tensor convention, payload state, source revision and uncertainty; alternatively a documented physical measurement protocol with equivalent traceability.

## 3. IMU transform

Message types, `frame_id=base_link`, simulator IMU sites and relative signal shapes are known. Physical sensor axes, mounting rotation, any driver-side remapping/calibration and the exact numeric output frame are not established.

Required evidence: deployed IMU driver assignment code/configuration and TF/static transform chain tied to the installed board/firmware. Static posture inference alone is insufficient.

## 4. Effort semantics

The interface advertises N·m, but the published value has not been traced to measured torque, estimator output, motor-current conversion, commanded torque or another signal. Motor-side/joint-side location, torque constant, gear conversion, sign, bias, filtering, clipping and timing are unknown.

Required evidence: deployed HAL/EtherCAT assignment chain or official specification defining the complete conversion and semantics.

## 5. MC internal command observability

MC state and preset execution are observable, but the post-interpolation, post-arbitration, post-balance-blending per-joint reference is not. Measured trajectories are outputs, not internal commands.

Required evidence for system identification: an already-supported read-only internal target/debug stream or logging API with timestamp and semantics. Without it, future work must remain `OUTPUT_RESPONSE_COMPARISON`.

## 6. Contact, actuation and timing

Simulation foot slip, penetration and contact continuity are proxies. There is no synchronized real foot wrench/pressure/contact state or external slip measurement. Real gear ratios, torque/current limits, velocity/acceleration limits, compliance, backlash, damping and friction are not evidence-closed. Capture alignment uses receive-monotonic time and does not prove source clocks or command/state latency.

Required evidence: manufacturer actuation/safety data, synchronized state and internal target timestamps, real foot force/contact sensing and an external pose/slip reference. Controlled motion must remain separately safety-approved and is not authorized by this freeze package.

## Final blocker state

`EVIDENCE_BACKED_PHYSICAL_MODEL_UPDATE_READY = NO`

`DYNAMICS_CALIBRATION_READY = NO`

`CALIBRATED_MJCF_CREATION_ALLOWED = NO`

