# Phase 3A-V selected independent motion

## Selection

| Field | Evidence-backed value |
| --- | --- |
| Motion | `wave`, right hand |
| Operator call entry | `Agentech.wave(right)` |
| Native preset ID | `1002` |
| Preset/control area | `2` (right-hand area) |
| Required posture | `STAND_DEFAULT` |
| MC route | `wave -> _gesture("wave", right) -> _execute -> client.start_preset(area=2, motion=1002, interrupt=False) -> SetMcPresetMotion` |
| Direct HAL | **NO evidence of direct HAL; native MC preset route** |
| Software physical-test evidence | `physically_tested=(Hand.RIGHT,)` in the inspected standing catalog |
| Runtime validation on the current capture | **PENDING_OPERATOR_CAPTURE** |

## Source evidence

- Read-only source capture: `work/phase2c_agentech01_code_discovery_readonly.txt`.
- Catalog source path: `/mnt/c/Users/wesle/.agentech/staging/master-turn-head-7-5-2bf99b0/agentech/robots/master/actions/standing.py`.
- Catalog source SHA-256: `8c4207f263fd975cccb1713ebf02514441fd0e806d9fb25841f1c8d8abc8286a`.
- `STANDING_GESTURES["wave"]` specifies motion 1002, left/right support, and right-hand physical-test evidence.
- `area_for(Hand.RIGHT)` resolves to area 2.
- The wrapper checks `STAND_DEFAULT` before submission, calls native `SetMcPresetMotion`, waits on the returned task ID, and checks `STAND_DEFAULT` again afterward.

## Independence hypothesis

`heart(both)` is preset 1007 / area 3 and produced a bilateral mirrored arm motion. `wave(right)` is preset 1002 / area 2 and is expected to be unilateral and cyclic. This makes it a strong independent-motion candidate, but final independence is **not assumed**: the captured active-joint set, excursion vector, duration, velocity, symmetry, and balance response must pass the numerical independence check.

## Safety ownership

Codex and the recorder do not invoke this motion. Only the onsite operator/soft engineer may run the already-approved preset after the recorder announces that five seconds of pre-roll are complete.
