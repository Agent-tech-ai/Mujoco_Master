# Phase 3A-V generalization report

- selected motion: `wave(right)`, native MC preset 1002 / area 2
- independence: **SUFFICIENTLY_INDEPENDENT_VALIDATION_MOTION**
- legacy vs candidate overall arm tracking: **CANDIDATE_BETTER**

| frozen Phase 3A change | blind-validation result |
| --- | --- |
| shoulder/wrist bandwidth | shoulder_roll=GENERALIZES; shoulder_yaw=GENERALIZES; wrist_yaw=GENERALIZES; wrist_roll=INSUFFICIENT_EXCITATION |
| standing-reference alignment | PARTIAL_GENERALIZATION: bilateral knee mean absolute equilibrium mismatch improves `0.08327 -> 0.05823 rad`, but the candidate validation contains persistent pelvis/left-hip contact |
| 0.7x balance gain | PARTIAL_GENERALIZATION: stable arm-only replay and lower aggregate RMSE, but ankle/knee excursion over-response and the contact safety blocker remain |

No component was retuned. The shoulder-roll, shoulder-yaw, and wrist-yaw
bandwidth changes are independently supported by this motion. Wrist roll remains
unvalidated because it was not excited. The standing/balance components are
downgraded rather than accepted as a package. Residual collision/geometry
evidence is recorded as `PHYSICAL_MODEL_MISMATCH_CANDIDATE`, but Phase 3B is not
enabled while the final gate is `NO`.
