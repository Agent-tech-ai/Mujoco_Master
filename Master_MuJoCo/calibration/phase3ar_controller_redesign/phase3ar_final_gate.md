# Phase 3A-R final gate

1. Contact root cause: **controller settling/attitude posture drives a low-margin
   left pelvis/hip geometry into contact**; static initialization is clear.
2. Geometry vs controller: primary evidence supports `CONTROLLER_POSTURE_ISSUE`;
   geometry asymmetry exists but is not proven erroneous.
3. Whole-body instability: legal measured targets develop tracking error, early
   contact and limit crossing; large balance excursion and saturation follow.
4. Causal order: `TRACKING_FIRST -> CONTACT -> LIMIT -> BALANCE_EXCURSION -> SATURATION -> FALL`.
5. Global 0.7x does not generalize because heart/wave roll disturbance and
   joint contribution structure differ substantially.
6. Joint-specific allocation is more reasonable and improves safety/response,
   but the tested architecture remains insufficient.
7. Shoulder-roll/wrist-yaw independent tracking improvement: **PRESERVED**.
8. Heart candidate arm-only stable: **YES**.
9. Wave candidate arm-only stable: **YES**.
10. Rehearsal: **12/12 SETTLED**.
11. Persistent pelvis/hip contact robustly resolved: **NO**; final wave
    standing/arm maxima are `0.494 / 0.486 mm`, only barely
    below the `0.500 mm` numerical threshold and with persistent contact.
12. **`VALIDATED_SIM_CONTROLLER_BASELINE = NO`**.

`PELVIS_HIP_CONTACT_RESOLVED = NO`  
`BALANCE_GENERALIZES_HEART_AND_WAVE = NO`  
`ARM_TRACKING_GENERALIZES = YES`  
`MJCF_COLLISION_GEOMETRY_FIX_REQUIRED = NO`  
`DYNAMICS_CALIBRATION_READY = NO`

Do not enter Phase 3B physical tuning from this result. Continue controller
architecture work, especially whole-body target arbitration/limit management and
a posture envelope with meaningful contact clearance.
