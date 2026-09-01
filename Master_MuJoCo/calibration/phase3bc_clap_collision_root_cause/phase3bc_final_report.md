# Phase 3B-C final report

## Final classification

`CLAP_SELF_COLLISION_ROOT_CAUSE = CONTROLLER_POSTURE`

`CONTACT_PAIR_CLASSIFICATION = CONTROLLER_POSTURE_SELF_CONTACT`

`MASS_DIRECTION_CAUSES_CONTACT = NO`

`MJCF_COLLISION_FILTER_REVIEW_REQUIRED = NO`

`POSITION_SPACE_PHYSICAL_DIRECTION_VALIDATED = NO`

`DYNAMICS_CALIBRATION_READY = NO`

## Answers to the eight closure questions

1. **Which bodies/geoms?** `left_wrist_roll_link` geom `67` (`UNNAMED`, mesh `left_wrist_roll_link`) versus `right_wrist_roll_link` geom `80` (`UNNAMED`, mesh `right_wrist_roll_link`).
2. **Are baseline and +8% identical?** They have the exact same sole pair and three episodes per replay mode. Onsets differ by at most `1.000 ms`; row counts differ only by 2 of roughly 1,048 1-kHz contact samples across both modes.
3. **Maximum penetration and duration?** Across all four replays, maximum penetration is `1.177893 mm`; maximum single-episode duration is `0.186 s`. Peak normal force is `10.500154 N`.
4. **Gesture-peak relation?** Yes. There are exactly three contact closures at approximately 1.60, 2.78, and 3.98 s, aligned with the three closed-hand phases of the Clap trajectory; no pre-roll contact exists.
5. **Controller posture or topology?** `CONTROLLER_POSTURE`: the replayed Clap posture brings two physical end-effector mesh surfaces together. The bodies are not adjacent and collision masks are functioning as authored.
6. **Does +8% aggravate contact?** No systematic evidence. It adds no pair or episode; mode-dependent changes are tens of micrometres / about 1 ms and do not consistently increase penetration, force, or duration.
7. **Should it veto physical-direction validation?** Not as evidence against the +8% direction. It remains an absolute gesture-safety/policy item pending expected-contact semantics and physical verification.
8. **Does Phase 3B-V need reinterpretation?** Yes: the previous absolute failure is a pre-existing Clap end-effector contact, not candidate-caused. Magnitude support is `PARTIAL`, but formal validation remains `NO`; no gate is auto-promoted.

## Integrity

All four diagnostic replays reproduce the frozen Phase 3B-V fall, self-contact, pelvis/hip-contact, non-foot-ground-contact, limit, target-clip, and safety counts exactly. Instrumentation is read-only after `mj_step`. No controller, physical parameter, MJCF, collision mask, robot state, or hardware mapping was modified, and no `reported_effort` was used.

The +8% candidate remains **SHARED PHYSICAL SENSITIVITY DIRECTION — NOT IDENTIFIED HARDWARE PARAMETER**. This report is not `REAL_MASS_CALIBRATION`, `HARDWARE_MASS_IDENTIFICATION`, or `ACTUATOR_SYSTEM_IDENTIFICATION`.
