# Standing baseline report

Status: **BASELINE FAILED; SIMULATION-ONLY CLEANUP PASSED 10 s**

| Run | Fall time | Final base XYZ (m) | Max tilt | Base displacement | Max joint error | Max saturation | Both-feet contact |
|---|---:|---|---:|---:|---:|---:|---:|
| Free, zero command | 2.825 s | [0.012993446398207173, -0.23947985794789184, 0.21070873910076562] | 200.698° | 0.527 m | 143.012° | 0.000 | 0.931 |
| Free, original controller | 1.606 s | [0.9435419295773859, 0.003725701678707336, 0.06454151939247321] | 86.637° | 1.127 m | 24.285° | 1.000 | 0.961 |
| Fixed, original controller | none | [0.0006481181950335458, 2.9025690112675055e-05, 0.6795177584746275] | 0.078° | 0.000808 m | 0.012° | 0.066 | N/A |
| Free, cleanup controller | none | [0.013239394408583745, 6.692287132292435e-06, 0.6741747567698644] | 2.100° | 0.014 m | 1.285° | 0.247 | 0.997 |

The free original controller falls forward at 1.606 s. It is deterministic, not random: repeated/sensitivity runs share positive-X drift and pitch divergence. Fixed-base joints remain numerically stable, so the immediate fall is a missing free-base balance loop rather than a solver blow-up.

All initial actuator targets equal initial actuated qpos and initial qvel is zero. The complete arrays are retained in `summary.json`.
