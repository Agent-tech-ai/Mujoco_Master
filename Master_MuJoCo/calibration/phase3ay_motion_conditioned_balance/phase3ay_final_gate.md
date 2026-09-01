# Phase 3A-Y Final Gate

- `ARM_TRACKING_GENERALIZES = YES`
- `CONTACT_SAFETY_ROBUST = YES`
- `LIMIT_MANAGEMENT_ROBUST = YES`
- `SATURATION_MANAGEMENT_ROBUST = YES`
- `BALANCE_AMPLITUDE_GENERALIZES = NO`
- `BALANCE_TIMING_GENERALIZES = NO`
- `HEART_BALANCE_ACCEPTABLE = NO`
- `WAVE_BALANCE_ACCEPTABLE = NO`
- `WHOLE_BODY_STRESS_TEST_PASSES = YES`
- `PERTURBATION_ROBUSTNESS = YES`
- `VALIDATED_SIM_CONTROLLER_BASELINE = NO`
- `DYNAMICS_CALIBRATION_READY = NO`

## Fixed engineering bands

- Amplitude GOOD: 0.67–1.50×; ACCEPTABLE: 0.50–2.00×; otherwise POOR.
- Timing GOOD: onset/peak/lag within 0.25 s and recovery within 0.50 s; ACCEPTABLE: within 0.50/1.00 s; otherwise POOR.
- A sign conflict is always POOR.

These bands were fixed before final gate evaluation. They reflect the 50 Hz measurement grid (20 ms), the limited two-motion dataset, and an engineering tolerance that does not require 1.000× matching.

## Gate rationale

Heart left-ankle and waist-roll amplitudes are repaired to order-one ratios, but timing and other knee channels remain poor. Wave right-knee remains about eight times the very small real excursion, so amplitude/timing generalization is not achieved. Hard safety, whole-body stress, arm tracking, and 8/8 perturbation robustness are retained.

`DYNAMICS_CALIBRATION_READY` remains NO because PHYSICAL_SIGN/PHYSICAL_ZERO, effort semantics, complete IMU transform, and MC internal command remain unresolved.
