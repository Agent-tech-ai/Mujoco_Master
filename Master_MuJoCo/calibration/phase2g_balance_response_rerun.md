# Phase 2G balance-response comparison rerun

| experiment | quantity | real excursion | sim excursion | ratio | RMSE | peak Δt | phase lag | real recovery | sim recovery |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| free_baseline | left_ankle_pitch_joint | 0.04621 | 0.07313 | 1.582 | 0.02748 | -0.42 | 1.00 | 0.01 | UNKNOWN |
| free_baseline | right_ankle_pitch_joint | 0.03202 | 0.07218 | 2.254 | 0.02401 | -0.40 | -0.42 | 0.01 | 0.01 |
| free_balance_gain_scale_060 | left_ankle_pitch_joint | 0.04621 | 0.02934 | 0.635 | 0.01602 | 0.12 | -0.38 | 0.01 | UNKNOWN |
| free_balance_gain_scale_060 | right_ankle_pitch_joint | 0.03202 | 0.02796 | 0.873 | 0.01167 | 0.08 | -0.46 | 0.01 | UNKNOWN |

The earlier left ankle result is reproduced: real 0.04621 rad versus baseline sim 0.07313 rad (ratio 1.582). Scaling only the simulation balance gains to 0.60 produces 0.02934 rad (ratio 0.635) and reduces relative-response RMSE from 0.02748 to 0.01602 rad. The direction of improvement shows that controller alignment explains a material portion of the excessive baseline response, but the candidate over-corrects excursion and does not validate physical dynamics.

Right ankle improves from ratio 2.254 to 0.873. Hip pitch and torso pitch metrics also improve, while knee excursion remains excessive and waist roll remains under-responsive. Full metrics, including peak timing, recovery, phase lag, torso relative roll/pitch and gyro norm, are in `phase2g_balance_metrics.csv`. Torso axes remain `PARTIAL_TRANSFORM`; gyro is compared as a norm only.
