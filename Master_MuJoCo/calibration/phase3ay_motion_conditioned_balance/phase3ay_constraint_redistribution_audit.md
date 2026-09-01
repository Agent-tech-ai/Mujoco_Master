# Phase 3A-Y Constraint Redistribution Audit

Per-timestep evidence is in `phase3ay_constraint_redistribution_audit.csv` and contains raw desired allocation, priority/pre-allocation scaling, redistributed allocation, post-safety scaling, and final allocation.

## Wave right-knee root-cause check

- Frozen 3A-X raw knee weight: **0.150**.
- Mean redistributed right-knee weight during wave: **0.101984**; maximum **0.102655**.
- Ankle limit-scaling activation fraction: **0.000000**.
- Right-knee limit/saturation activation fractions: **0.000000 / 0.000000**.

Conclusion: **Phase 3A-X constraint redistribution did not manufacture the knee over-response.** The priority-normalized knee share is below its raw 0.15 weight, ankle corrections were not limit-clipped, and there was no knee saturation-driven transfer. The remaining 8× ratio is a passive/coupled position response against a very small real denominator; safely removing it was not possible with controller allocation alone.
