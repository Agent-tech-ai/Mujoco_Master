# Phase 3A-X limit management

AX-A alone improved whole-body minimum margin from `-0.06282`
to `-0.02100 rad`, but still crossed a limit and fell:
`PARTIAL_OR_INSUFFICIENT`.

Combined architecture uses directional/velocity warning scaling plus an analytic
equivalent-target clamp to `lower/upper +/- 0.050 rad`. Final whole-body minimum
actual margin is `0.04623 rad`; violation samples `0`.

`LIMIT_MANAGEMENT_ROBUST = YES`
