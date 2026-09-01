# Phase 3A-V final gate

1. Second real motion: **wave(right)**, native MC preset `1002`, area `2`.
2. Independence from heart: **SUFFICIENTLY_INDEPENDENT_VALIDATION_MOTION**;
   active-set Jaccard `0.500`, excursion-vector cosine similarity `0.335`, and
   right-vs-left excursion asymmetry `0.996`.
3. Shoulder-roll improvement: **GENERALIZES**. On the excited right joint,
   legacy-to-candidate RMSE is `0.10414 -> 0.02656 rad` and lag is
   `0.840 -> 0.080 s`. Shoulder yaw also generalizes (`0.10326 -> 0.05707 rad`,
   `0.140 -> 0.080 s`).
4. Wrist-yaw improvement: **GENERALIZES** on the excited right joint; RMSE
   `0.38912 -> 0.14992 rad`, lag `0.420 -> 0.160 s`. Wrist roll has
   **INSUFFICIENT_EXCITATION**.
5. Standing knee equilibrium improvement: **PARTIAL_GENERALIZATION**. Bilateral
   mean absolute mismatch improves `0.08327 -> 0.05823 rad`, but the candidate
   has a persistent standing self-contact.
6. The `0.7x` balance gain is **PARTIAL_GENERALIZATION**, not fully accepted.
   Arm-only remains upright and aggregate balance RMSE improves, with no repeat
   of the heart left-ankle under-response; wave response over-shoots excursion
   at ankles (`2.599x/1.666x`) and knees (`20.455x/8.296x`).
7. Legacy vs candidate: **CANDIDATE_BETTER** for active-arm tracking; mean RMSE
   `0.46964 -> 0.12443 rad`, mean absolute lag `0.493 -> 0.147 s`.
8. Free-base stability: **YES for candidate arm-only**; **NO for candidate
   whole-body measured-reference replay**, which falls at `8.037 s`. Replay 2
   is not used to evaluate autonomous balance prediction.
9. Collision/limit/saturation: **BLOCKER PRESENT**. Arm-only has no limit
   violation or persistent saturation but has 690 samples of the single
   `pelvis <-> left_hip_roll_link` contact (maximum penetration `1.289 mm`).
   Whole-body additionally reaches `-0.04501 rad` minimum limit margin and
   `0.11708` persistent saturation fraction.
10. Independently supported: shoulder-roll, shoulder-yaw, and wrist-yaw
    simulation bandwidth changes; the candidate is clearly better than legacy
    for the active arm trajectory.
11. Downgraded/unvalidated: standing-reference and `0.7x` balance candidates are
    `PARTIAL_GENERALIZATION`; wrist-roll is `INSUFFICIENT_EXCITATION`. No
    component was retuned in Phase 3A-V.
12. **VALIDATED_SIM_CONTROLLER_BASELINE = NO**.

`DYNAMICS_CALIBRATION_READY = NO`. Do not enter Phase 3B or physical dynamics
tuning from this result. This validation does not enable torque calibration or
physical system identification.
