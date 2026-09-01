# Phase 3A Before/After and Final Gate

## Immutable baseline

- locked files verified unchanged: **43/43**
- Phase 2 overwritten: **NO**
- reported effort used for fitting: **NO**
- absolute IMU quaternion used for fitting: **NO**
- MJCF/controller source/hardware mapping modified: **NO**

## Required final answers

1. Shoulder-roll lag: **0.240 s -> 0.100 s**.
2. Wrist-yaw lag: **0.380 s -> 0.160 s**.
3. Knee equilibrium: bilateral mean absolute offset **0.12030 -> 0.00843 rad**; largely explained/reduced by simulation equilibrium alignment, not assigned to hardware zero.
4. Ankle excursion: left ratio **1.583 -> 0.644**, right **2.254 -> 0.856**. Error improves, but left now undershoots and is not fully matched.
5. Free-base 10 s standing stable: **YES**; max tilt `2.835 deg`, foot-slip proxy L/R `0.00668/0.00608 m`, no collision/non-foot contact, no persistent saturation.
6. Prior rehearsal: **12/12 SETTLED**.
7. Explained at position/controller layer: most arm phase lag and RMSE, knee equilibrium bias, and much of the excessive ankle excursion. Reference rate/timestep/global shift/free-base coupling are not the primary arm-delay cause.
8. Still requiring physical-dynamics calibration or closed evidence: residual balance amplitude/phase/recovery mismatch, ankle equilibrium tradeoff, relative base/gyro mismatch, contact/foot-slip fidelity, actuator/torque behavior, and any mass/inertia/friction effects. Sign/zero, effort semantics, and full IMU transform gates remain outside Phase 3A.
9. **POSITION_RESPONSE_BASELINE_READY = YES**.

## Acceptance evidence

- classification: **ACCEPTED_SIM_CONTROLLER_ALIGNMENT**
- free-base replay stable/no fall: `True`
- target clipping: `0` samples
- persistent saturation: `0.00000`
- collision/non-foot ground contact: `0/0` samples
- 10 s free standing safety gate: `PASS`
- 12-joint rehearsal gate: `PASS`

The accepted file is `simulation_controller_alignment_candidate.json`. It is explicitly **NOT HARDWARE CALIBRATION**. The gain scan ending at its upper boundary and use of one temporally split heart capture limit generalization; a second independent motion should validate it before treating the response baseline as frozen for broader use.

**DYNAMICS_CALIBRATION_READY = NO.** Phase 3A does not change the Phase 2H dynamics gate.
