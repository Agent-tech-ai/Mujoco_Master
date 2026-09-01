# Phase 3B-C2 — Real Clap contact evidence

Date: 2026-08-28

## Decision

`REAL_CLAP_HAND_CONTACT = CONFIRMED`

`SIMULATION_WRIST_CONTACT_TASK_SEMANTICS = CONFIRMED_CONSISTENT`

`CONTACT_PAIR_CLASSIFICATION = EXPECTED_TASK_CONTACT`

## New field evidence

Evidence source: `SOFT_ENGINEER_EXPLICIT_CONFIRMATION`, relayed by the operator on 2026-08-28.

Confirmed statement and scope:

- robot: X2;
- control path: MC preset;
- preset: `area=11`, `motion=3017` (`clap`);
- normal physical behavior: the robot makes physical hand-to-hand contact during the Clap motion.

This is direct human field evidence about the contact semantics of the exact preset used by the accepted Phase 3B-V
capture. It closes the contact/no-contact ambiguity that remained after the earlier code, telemetry, documentation, and
simulation-only review.

## Corroborating frozen evidence

| Evidence | Confirmed fact |
|---|---|
| `phase3bv_capture_metadata.json` | The accepted independent real capture executed `clap`, `motion=3017`, `area=11`; MC reported motion status 3017 while standing. |
| `work/phase2c_agentech01_code_discovery_readonly.txt`, lines 156553–156560 | `clap` maps to `StandingGesture("clap", 3017, fixed_area=11)`, is marked physically tested, and uses both hands. |
| Phase 3B-C contact trace | The sole simulated pair is `left_wrist_roll_link <-> right_wrist_roll_link`; it appears in exactly three episodes aligned with the three Clap closures. |
| Phase 3B-C baseline/candidate comparison | Original and `bs_mass_lower_plus08` have the same pair and three episodes; onset differs by no more than `1.000 ms`. |
| Phase 3B-C root-cause checks | `MASS_DIRECTION_CAUSES_CONTACT = NO`; the contact is neither a numerical transient nor a collision-filter/topology error. |

## Bounded expected-contact exception

The following specific simulation contact is classified as `EXPECTED_TASK_CONTACT`:

`left_wrist_roll_link <-> right_wrist_roll_link`

The exception is valid only when all of these conditions hold:

1. motion semantics are X2 MC preset Clap `area=11`, `motion=3017`;
2. contact occurs inside one of the three expected Clap closure windows established by the frozen contact timeline;
3. the colliding pair is exactly the left/right wrist-roll end-effector pair above;
4. no additional self-contact pair or unrelated safety failure is present.

It does not authorize any of the following:

- globally ignoring self-collision;
- ignoring this pair outside the expected Clap closure windows;
- ignoring any other self-contact pair during Clap or another motion;
- changing MJCF collision masks or filters;
- treating new penetration, force, duration, fall, slip, limit, or saturation regressions as expected.

## Interpretation

The three simulation wrist-contact episodes are consistent with the confirmed real task semantics: physical
hand-to-hand contact is a normal part of preset 3017, and the simulated pair closes at the three Clap closures.
Because baseline and `bs_mass_lower_plus08` reproduce the same pair, episode count, and onset timing, this expected
contact is not evidence of a candidate-caused safety regression.

## Scope and integrity

No replay or sensitivity experiment was rerun. No controller, mass, physical parameter, MJCF, collision filter,
hardware mapping, robot state, or robot configuration was changed. No robot connection or command was used for this
reinterpretation.

