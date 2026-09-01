# Phase 3A-X saturation management

AX-D alone failed earlier (`7.249 s`) and had
`54.9%` time-saturation. Saturation is a
downstream consequence; standalone scaling is rejected.

Combined warning/hard thresholds are `0.75 / 0.95` of unchanged ctrlrange. There
is no integral term, so this is not anti-windup. Upstream contact/limit/rate and
tracking gates prevent saturation onset. Final whole-body persistent saturation:
`0.000%`, max consecutive `0.000 s`.

`SATURATION_MANAGEMENT_ROBUST = YES`
