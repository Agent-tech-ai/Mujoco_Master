# Phase 3B-V Clap safety report

| condition | mode | stable_no_fall | self_collision_samples | pelvis_hip_contact_samples | nonfoot_ground_contact_samples | max_contact_penetration_m | limit_violation_samples | persistent_saturation_fraction | max_saturation_fraction | max_foot_slip_m | max_abs_tilt_deg | absolute_safety_pass |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| original | arm_only | YES | 28 | 0 | 0 | 0.00130997 | 0 | 0 | 0.676618 | 0.00796222 | 4.07109 | NO |
| original | whole_body | YES | 27 | 0 | 0 | 0.0013099 | 0 | 0 | 0.675128 | 0.00788754 | 3.71087 | NO |
| mass_direction | arm_only | YES | 28 | 0 | 0 | 0.0012886 | 0 | 0 | 0.670669 | 0.00795427 | 3.94474 | NO |
| mass_direction | whole_body | YES | 27 | 0 | 0 | 0.00128934 | 0 | 0 | 0.670017 | 0.00782766 | 3.61414 | NO |

- New candidate fall/contact/limit/persistent saturation: `NO`
- Candidate self-collision sample count: not greater than baseline in either replay mode.
- Both conditions contain the same brief non-pelvis self-collision condition; absolute safety fails even though the candidate does not worsen baseline safety.
- The existing safety logger does not record the colliding geom/body pair, so the exact self-collision pair remains `UNKNOWN` rather than inferred.

`SAFETY_BASELINE_PRESERVED = YES`

`ABSOLUTE_CLAP_SAFETY_PASS = NO`
