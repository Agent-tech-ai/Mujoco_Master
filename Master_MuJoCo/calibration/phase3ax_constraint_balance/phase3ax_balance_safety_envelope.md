# Phase 3A-X simulation balance safety envelope

These are simulation design values, not X2 official limits.

| item | value |
| --- | ---: |
| joint reserve / warning width | 0.050 / 0.120 rad |
| pelvis/hip warning / hard zone | 3.000 / 0.750 mm |
| contact avoidance cap | 0.070 rad equivalent target |
| actuator warning / hard | 0.75 / 0.95 ctrlrange fraction |
| whole-body reference slew | 0.35 rad/s |
| tracking error warning / hard | 0.060 / 0.180 rad |
| ankle pitch / roll slew | 40 / 25 N m/s |
| hip pitch / roll slew | 30 / 20 N m/s |
| knee slew | 35 N m/s |
| waist pitch / roll slew | 22 / 16 N m/s |
| safe-standing left hip-roll offset | +0.025 rad |

Continuous margin scaling is followed by an analytic final equivalent-target
clamp. Contact correction begins before collision. No value is a hardware limit.
