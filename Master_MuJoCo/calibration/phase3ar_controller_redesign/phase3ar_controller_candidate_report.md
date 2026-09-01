# Phase 3A-R controller candidate report

Selected for full validation: `family_c_ankle_0p7_hip_0p10_knee_0p15`.

- ankle pitch allocation: `0.7`
- hip pitch allocation: `0.10`
- knee pitch allocation: `0.15`
- ankle roll allocation: `0.7`
- shoulder/wrist bandwidth: unchanged at Phase 3A `8x`
- standing-reference alignment: unchanged
- physical dynamics/MJCF/hardware mapping changes: `none`
- reported effort used: `False`

Result: **`REJECTED_AFTER_FINAL_ROBUSTNESS_VALIDATION`**. It preserves arm tracking,
passes 12/12 rehearsal, and reduces contact depth, but it does not provide a
robust contact margin, does not generalize balance response across both motions,
and does not stabilize whole-body measured-reference replay.
