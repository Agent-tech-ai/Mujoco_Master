# Standing-stability root-cause ranking

1. **PRIMARY_ROOT_CAUSE — missing free-base balance feedback.** The original controller regulates only 30 actuated joints and has no pelvis attitude/CoM feedback. Fixed base stands, all modest contact/gain/solver variants still fall, and adding simulated pelvis roll/pitch feedback makes free base stand for 10 s.
2. **SECONDARY_CONTRIBUTOR — foot initialization and discrete collision contact.** Sole spheres begin 5.05 mm above the floor and land through small 5 mm-radius points. This adds a transient/chatter, but lowering the base and replacing each foot by a box individually do not prevent the fall.
3. **SECONDARY_CONTRIBUTOR — modeled friction deadband in fixed-base small-motion tracking.** `frictionloss=0.3` plus pure PD explains the joint-family error scale. Smooth compensation changes all 12 rehearsals from `TRACKING_NOT_SETTLED` to `SETTLED` without saturation.
4. **POSSIBLE_CONTRIBUTOR — detailed physical fidelity of masses, inertias and contact parameters.** No gross sanity fault was found, but hardware correctness is UNKNOWN without manufacturer or identification evidence.
5. **RULED_OUT AS PRIMARY — CoM initially outside support, gross left/right mass imbalance, friction magnitude, modest Kp/Kd scaling, and timestep.** The initial CoM is inside the combined support hull; left/right mass differs by only 0.017689 kg; every corresponding single-factor run still falls.
6. **UNKNOWN — real X2 whole-body controller, ground/contact properties, actuator dynamics and protection behavior.** No real-robot evidence was used.

## Final answers

1. The original free-base run falls because a joint-space PD controller cannot regulate unactuated base pitch/roll; once the body starts pitching forward, the controller has no balance objective and later saturates.
2. Controller architecture is primary. Foot/contact initialization is secondary. CoM, gross mass distribution and numerical integration are not primary based on the experiment matrix.
3. Minimal accepted changes: a separate simulation-only pelvis-attitude feedback layer and smooth compensation of the model's already-declared frictionloss. No MJCF, mass, inertia, friction, mapping, or hardware parameter was changed.
4. Yes. The candidate runs continuously for 10 s: max tilt 2.100°, displacement 0.014 m, foot slip below 0.147 mm, and saturation ratio 0.000000.
5. All **12/12** rehearsals changed from `TRACKING_NOT_SETTLED` to `SETTLED` under the documented simulation thresholds.
6. Both feedback gains and modeled-friction compensation are simulation cleanup only. They are not realistic, calibrated, hardware-matched, or deployable robot parameters.
7. Real dynamics calibration still needs approved single-joint command ownership/recovery, measured command/position/velocity/effort logs, effort-source semantics, hardware sign/zero/encoder offset, actuator limits/torque-current mapping, rigid-body/inertial evidence, IMU-frame extrinsics, contact/foot geometry, and safe physical excitation data.
