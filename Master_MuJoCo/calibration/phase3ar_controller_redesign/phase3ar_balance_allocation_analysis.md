# Phase 3A-R balance allocation analysis

The Phase 3A controller applies pitch/roll attitude torque only to both ankles;
hip, knee, and waist balance additions are zero. It has no integral state.

| family | best experiment | safety | shape score | decision |
| --- | --- | --- | --- | --- |
| A_B_SUBSYSTEM_ISOLATION | ab_improved_arm_legacy_balance | FAIL | 14.412 | REJECTED_OR_DIAGNOSTIC_ONLY |
| A_CLAMP | family_a_roll_clamp_3 | FAIL | 2.479 | REJECTED_OR_DIAGNOSTIC_ONLY |
| B_RATE_LIMIT | family_b_rate_50_25 | FAIL | 2.717 | REJECTED_OR_DIAGNOSTIC_ONLY |
| CURRENT | sweep_current_07 | FAIL | 2.735 | REJECTED_OR_DIAGNOSTIC_ONLY |
| C_JOINT_ALLOCATION | family_c_ankle_0p7_hip_0p10_knee_0p15 | PASS | 1.701 | ACCEPTED_FOR_FINAL_VALIDATION |
| D_STANDING_REFERENCE | family_d_standing_0p75 | FAIL | 2.932 | REJECTED_OR_DIAGNOSTIC_ONLY |
| E_COMBINATION | family_e_roll3_pitch0p85_hip0p10_knee0p15 | FAIL | 1.566 | REJECTED_OR_DIAGNOSTIC_ONLY |

Heart and wave raw pitch-feedback peaks are similar (`~10.95/12.27 N·m`), while
raw roll-feedback peaks differ strongly (`~0.77/7.58 N·m`). This explains why a
single global 0.7x scalar cannot preserve heart response and suppress wave
over-response simultaneously. Joint/channel-specific allocation is structurally
more appropriate and reduced the wave contact and proximal excursions without
changing arm gains. However, the final candidate still has serious knee response
ratios and persistent near-tolerance contact, so **`BALANCE_GENERALIZES_HEART_AND_WAVE = NO`**.

All real targets are `OUTPUT_RESPONSE_DESIGN_TARGET`, not `MC_GAIN_IDENTIFICATION`.
