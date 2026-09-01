# Phase 3A-X wave whole-body stress test

This is `STRESS / CONSTRAINT TEST`, not a fit target or robot prediction.

| metric | Phase 3A-R | Phase 3A-X |
| --- | ---: | ---: |
| fall | `7.450 s` | `NO FALL` |
| min joint margin | `-0.06282` | `0.04623` |
| pelvis/hip penetration | `2.575 mm` | `0.000 mm` |
| contact samples | present | `0` |
| persistent saturation | present | `0.000%` |

Tracking arbitration slews balance-joint references at `0.35 rad/s` and reduces
progression above `0.060 rad` error, prioritizing constraints over whole-body RMSE.

`WHOLE_BODY_STRESS_TEST_PASSES = YES`
