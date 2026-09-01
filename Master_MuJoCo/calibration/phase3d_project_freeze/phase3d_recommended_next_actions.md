# Phase 3D Recommended Next Actions

No further simulation optimization should start from this freeze. Resume only to ingest new physical evidence or to verify the integrity of the frozen package.

## Priority 0 — evidence required before any physical-model update

1. **Exact robot configuration record**
   - X2 Ultra serial/hardware revision, firmware, installed hands/end effectors, covers, batteries and payloads.
   - A configuration-specific mass budget is required so CAD/manufacturer values refer to the captured robot.

2. **Manufacturer/CAD mass-property package**
   - Per-link or separable assembly mass, CoM and inertia tensor.
   - Link/frame definition, tensor convention, units, revision, payload state and uncertainty.
   - Explicit pelvis/torso assembly data to resolve the `5.031811 kg` mesh-derived versus `3.523487 kg` URDF discrepancy.

3. **Joint sign and zero package**
   - Official/deployed joint axis convention for target joints.
   - Manufacturer-defined home/calibration pose or fixture datum.
   - Passive measured `JointState.position` in that pose and the encoder-offset procedure.
   - Do not promote Heart/Wave/Clap mirroring to physical sign.

4. **Deployed IMU transform package**
   - Physical chest/torso sensor frames and mounting rotations.
   - Driver remapping/calibration, quaternion direction/order, gravity convention and published-frame semantics.
   - Deployed TF/static transforms tied to firmware and sensor revision.

5. **Effort provenance package**
   - HAL/EtherCAT source field feeding `JointState.effort`.
   - Measured/estimated/current-derived/commanded classification.
   - Motor/joint side, torque/current conversion, gear handling, units, sign, bias, filtering, clipping and timestamp.

## Priority 1 — required for stronger output-response work

6. **Read-only MC internal target observability**
   - Find an existing joint-reference/debug/logging interface after preset interpolation, ownership arbitration, balance blending and clamps.
   - If unavailable, explicitly keep all work as output-response fitting.

7. **Clock/latency evidence**
   - Establish robot source-clock relationships and end-to-end command/state timestamps.
   - Quantify transport, buffering and logging delay before attributing phase lag to physical dynamics.

8. **Physical contact reference**
   - Synchronized foot force/pressure or wrench data, real contact state, and external pelvis/foot pose or slip measurement.
   - Preserve the narrowly scoped Clap expected-contact exception; do not weaken global self-collision checks.

## Priority 2 — only after the applicable gates pass

9. Define a controlled validation matrix with independently approved robot safety limits, amplitudes, speeds, loads and abort behavior. This document does not authorize motion.

10. Split evidence into fitting and untouched validation sets across multiple motions, speeds and load cases.

11. Create a derived candidate MJCF only for parameters whose evidence chain is closed. Record source, units, uncertainty, exact old/new values and reason in a changelog. Never overwrite `ff_master_ultra.xml`.

12. Re-run controller validation after any evidence-backed physical change, because the current controller candidates were evaluated against the frozen original physical baseline.

## Restart gate

Before any physical change, produce an explicit review stating which A/B/C evidence closes which parameter and whether the proposed edit is mass, CoM, inertia, contact, mapping or actuation. Sensitivity-direction evidence alone cannot pass this review.

Until then:

- `POSITION_SPACE_PHYSICAL_DIRECTION_VALIDATED = YES`
- `EVIDENCE_BACKED_PHYSICAL_MODEL_UPDATE_READY = NO`
- `DYNAMICS_CALIBRATION_READY = NO`
- `CALIBRATED_MJCF_CREATION_ALLOWED = NO`

