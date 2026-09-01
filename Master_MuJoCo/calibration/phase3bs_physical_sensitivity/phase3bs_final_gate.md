# Phase 3B-S final gate

## Gates

| gate | status |
|---|---|
| PHYSICAL_SENSITIVITY_INFORMATIVE | YES |
| SHARED_PHYSICAL_SENSITIVITY_DIRECTION | YES |
| CONTROLLER_BASELINE_PRESERVED | YES |
| SAFETY_BASELINE_PRESERVED | YES |
| POSITION_SPACE_PHYSICAL_SENSITIVITY_READY | YES |
| DYNAMICS_CALIBRATION_READY | NO |

## Evidence summary

- Formal local sensitivity runs: 26 Heart/Wave runs; safety pass: 26/26.
- Frozen-source perturbation suite remains 8/8 PASS and frozen 12-joint rehearsal remains 12/12 SETTLED by locked Phase 3A-X/Y evidence.
- Shared direction validation: perturbation 8/8 PASS; rehearsal 12/12 SETTLED.
- Shared-direction screen passes: 1 experiment(s): bs_mass_lower_plus08.
- Highest local Wave knee sensitivity: MASS_DISTRIBUTION = -0.81967.
- Controller architecture and hashes are unchanged. All physical perturbations are runtime-only and labeled NOT HARDWARE CALIBRATION.

`DYNAMICS_CALIBRATION_READY` remains **NO** because sign/zero, effort semantics, absolute IMU transform, and MC internal command observability gates are not closed.
