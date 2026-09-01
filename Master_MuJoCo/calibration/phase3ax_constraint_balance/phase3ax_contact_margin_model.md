# Phase 3A-X contact-margin model

The controller reads current geom distance and evaluates a local finite-difference
gradient over hip roll/pitch and waist roll/pitch every 0.05 s inside the warning zone.

- warning / hard zones: `3.000 / 0.750 mm`
- numerical tolerance for reporting: `0.500 mm`
- avoidance equivalent-target cap: `0.070 rad`
- wave nominal left distance: `1.523 mm`
- left hip roll `+/-0.25 deg`: `1.755 / 1.291 mm`
- safe-standing `+0.025 rad`: `2.840 mm`

Contact is therefore locally predictable before active contact. This is a
simulation estimator, not robot geometry calibration. Final wave standing/arm
minimum distances are `1.134` /
`1.134 mm`, with zero contact samples.
