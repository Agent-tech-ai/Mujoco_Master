# Phase 3A-Y Cross-Validation Report

| experiment_id | fit_dataset | dataset | evaluation | safety_pass | balance_shape_score | arm_tracking_retained |
|---|---|---|---|---|---|---|
| phase3ay_cv_heart_only | heart | heart | FIT | True | 0.554995 | True |
| phase3ay_cv_heart_only | heart | wave | BLIND | False | 1.3193 | True |
| phase3ay_cv_wave_only | wave | heart | BLIND | True | 1.45166 | True |
| phase3ay_cv_wave_only | wave | wave | FIT | True | 0.998621 | True |

- **Heart-only -> blind wave: NO generalization.** It produces 180 hip-contact samples and a worse response score.
- **Wave-only -> blind heart: NO response generalization.** It stays safe but returns the poor fixed-allocation heart score.
- The joint state-conditioned architecture is designed only after both blind experiments. It is not selected by motion name.

This is strong overfitting evidence with only two motions; the final baseline therefore remains unvalidated when wave knee/timing gates fail.
