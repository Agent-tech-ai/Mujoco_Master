# Phase 3B-V gate reinterpretation after Phase 3B-C

## Evidence update

- baseline and +8% have the exact same sole geom/body pair: `TRUE`
- both have three contact episodes in each mode
- onset deltas are within `1.000 ms`
- maximum pair penetration: baseline `1.158956 mm`, candidate `1.177893 mm`
- peak normal force: baseline `10.500154 N`, candidate `10.496856 N`
- candidate added a contact pair: `FALSE`
- root cause: repeated Clap end-effector closure, not collision topology and not a numerical transient

`MASS_DIRECTION_CAUSES_CONTACT = NO`

The +8% condition does not create the contact. Arm-only pair penetration is slightly higher in one episode while whole-body penetration/force are lower or comparable; there is no consistent cross-mode severity increase. Therefore the contact must not be used as evidence that the mass direction *caused* a safety regression.

## Gate status

| Gate | Phase 3B-C interpretation |
|---|---|
| PHYSICAL_DIRECTION_MAGNITUDE_SUPPORT | PARTIAL |
| ABSOLUTE_CLAP_SAFETY_INTERPRETATION | PREEXISTING_REPEATABLE_CLAP_END_EFFECTOR_CONTACT; NOT CANDIDATE-CAUSED |
| CONTROLLER_BASELINE_PRESERVED | YES |
| SAFETY_BASELINE_PRESERVED | YES |
| POSITION_SPACE_PHYSICAL_DIRECTION_VALIDATED | NO |
| DYNAMICS_CALIBRATION_READY | NO |

This contact should **not** remain a candidate-specific veto against `bs_mass_lower_plus08`. However, the frozen generic rule "any self-contact fails absolute safety" still evaluates to NO until the project explicitly approves a gesture-aware expected-contact policy and, ideally, verifies the real Clap contact surface from video/physical evidence. Phase 3B-C does not silently weaken that rule and does not auto-promote the physical direction to VALIDATED.

Persistent blockers remain: `PHYSICAL_SIGN=UNKNOWN`, `PHYSICAL_ZERO=UNKNOWN`, `EFFORT_SEMANTICS=UNKNOWN`, `IMU_TRANSFORM=PARTIAL`, `MC_INTERNAL_COMMAND=UNOBSERVABLE`.
