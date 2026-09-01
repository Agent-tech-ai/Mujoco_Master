# Phase 3B-C first-contact causal timeline

Scope: original physical baseline, `arm_only`, first collision, `1.103` to `2.103 s`. The complete 50 Hz aligned timeline is `phase3bc_contact_timeline.csv`; wrist surface distance/contact is sourced from the 1 kHz diagnostic probe. Contact columns shown at 50 Hz are 20 ms bin aggregates; exact onset/end values come from the 1 kHz rows.

| Event | Time |
|---|---:|
| first negative wrist-geom distance / contact onset | 1.603 s |
| maximum penetration in episode | 1.623 s |
| contact end | 1.760 s |
| contact duration | 0.158 s |

| timestamp_s | signed_wrist_geom_distance_m | contact_active | contact_penetration_m | contact_normal_force_n | arm_reference_velocity_norm_rad_s | arm_velocity_norm_rad_s | arm_tracking_rms_rad | arm_tracking_max_abs_rad | base_roll_rad | base_pitch_rad | com_support_margin_m |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1.120000 | 0.182035 | 0 | 0.000000 | 0.000000 | 0.000000 | 1.371950 | 0.088947 | 0.249562 | -0.009714 | 0.062733 | 0.044216 |
| 1.400000 | 0.045584 | 0 | 0.000000 | 0.000000 | 0.741613 | 1.226644 | 0.079669 | 0.208904 | -0.008748 | 0.070919 | 0.045062 |
| 1.600000 | 0.000440 | 1 | 0.000701 | 9.334339 | 1.220487 | 0.375118 | 0.021711 | 0.064166 | -0.008272 | 0.069354 | 0.047680 |
| 1.620000 | -0.001114 | 1 | 0.001130 | 10.460880 | 1.238734 | 0.198971 | 0.016689 | 0.049040 | -0.008289 | 0.068919 | 0.047990 |
| 1.760000 | -0.000018 | 1 | 0.000207 | 3.442373 | 2.239451 | 0.605453 | 0.034924 | 0.087762 | -0.008318 | 0.065375 | 0.050264 |
| 2.100000 | 0.000000 | 0 | 0.000000 | 0.000000 | 0.713893 | 1.175238 | 0.075669 | 0.203433 | -0.007512 | 0.056130 | 0.055407 |

## Tracking/posture → contact onset

1. During pre-roll the wrist collision meshes remain separated by about 0.449 m; there is no initial overlap.
2. The measured-Clap arm reference brings both end-effector meshes together. Distance approaches zero, then becomes negative at `1.603 s`.
3. Contact persists through the closed-hand phase and ends at `1.760 s` as the arms separate. The same sequence repeats twice more.
4. Arm tracking RMS is `0.080341 rad` in the preceding 0.5 s, `0.022000 rad` during contact, and `0.090975 rad` in the following 0.5 s. Contact does not coincide with a tracking-error blow-up; during-contact maximum absolute tracking error is `0.087762 rad`.
5. Maximum base tilt during the first episode is `3.949 deg`; minimum COM support margin is `0.047990 m`. No fall, contact cascade, limit violation, or non-foot ground contact occurs.

The temporal direction is therefore: **Clap reference/posture closure → wrist surface convergence → contact**, not instability or a numerical one-step event → posture disturbance.
