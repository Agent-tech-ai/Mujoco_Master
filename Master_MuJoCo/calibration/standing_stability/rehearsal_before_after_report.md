# Active-test rehearsal before/after

Before: 12/12 fixed-base runs were `TRACKING_NOT_SETTLED`; zero collisions and zero limit violations.

After simulation-only frictionloss compensation: **12/12 SETTLED**, 0/12 `TRACKING_NOT_SETTLED`; zero collisions and zero limit violations. The free-base cleanup controller also passes the 10-second standing gate before the rehearsal infrastructure is considered valid.

| Joint | Before | After | Before steady error (°) | After steady error (°) | After overshoot (°) | After oscillation p-p (°) | After settling (s) | After saturation |
|---|---|---|---:|---:|---:|---:|---:|---:|
| `left_wrist_roll_joint` | TRACKING_NOT_SETTLED | SETTLED | 1.642 | 0.109 | 0.000 | 0.069 | 0.400 | 0.000000 |
| `right_wrist_roll_joint` | TRACKING_NOT_SETTLED | SETTLED | 1.642 | 0.109 | 0.000 | 0.069 | 0.400 | 0.000000 |
| `right_wrist_yaw_joint` | TRACKING_NOT_SETTLED | SETTLED | 1.637 | 0.102 | 0.000 | 0.071 | 0.370 | 0.000000 |
| `left_wrist_yaw_joint` | TRACKING_NOT_SETTLED | SETTLED | 1.637 | 0.102 | 0.000 | 0.071 | 0.370 | 0.000000 |
| `left_wrist_pitch_joint` | TRACKING_NOT_SETTLED | SETTLED | 1.641 | 0.109 | 0.000 | 0.069 | 0.400 | 0.000000 |
| `right_wrist_pitch_joint` | TRACKING_NOT_SETTLED | SETTLED | 1.641 | 0.109 | 0.000 | 0.069 | 0.400 | 0.000000 |
| `left_elbow_joint` | TRACKING_NOT_SETTLED | SETTLED | 0.534 | 0.092 | 0.000 | 0.051 | 0.330 | 0.000000 |
| `right_elbow_joint` | TRACKING_NOT_SETTLED | SETTLED | 0.534 | 0.092 | 0.000 | 0.051 | 0.330 | 0.000000 |
| `right_shoulder_yaw_joint` | TRACKING_NOT_SETTLED | SETTLED | 0.520 | 0.071 | 0.000 | 0.055 | 0.260 | 0.000000 |
| `left_shoulder_yaw_joint` | TRACKING_NOT_SETTLED | SETTLED | 0.520 | 0.071 | 0.000 | 0.055 | 0.260 | 0.000000 |
| `left_shoulder_pitch_joint` | TRACKING_NOT_SETTLED | SETTLED | 0.526 | 0.116 | 0.000 | 0.041 | 0.510 | 0.000000 |
| `right_shoulder_pitch_joint` | TRACKING_NOT_SETTLED | SETTLED | 0.526 | 0.116 | 0.000 | 0.041 | 0.510 | 0.000000 |

Per-joint diagnosis: the target generator completes all required phases; actuator saturation ratio is zero; these are fixed-base runs, so base and contact instability do not create the tracking error. The before-run error magnitude follows the model's frictionloss/Kp deadband. Timestep sensitivity did not change the standing failure. Evidence therefore supports `MODELED_FRICTION_DEADBAND / CONTROLLER_FORM` rather than actuator-force shortage or contact/base motion for the fixed-base tracking result.

The before/after metrics are in `rehearsal_summary.json`, with per-joint CSVs and plots under `rehearsal_after/`. Results are simulation infrastructure evidence only; they do not determine hardware sign, zero, friction, torque semantics or dynamics.
