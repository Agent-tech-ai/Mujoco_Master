# Phase 3B-C contact-pair analysis

## Result

Only one internal pair occurs in all four frozen replays:

- body pair: `left_wrist_roll_link` ↔ `right_wrist_roll_link`
- geom IDs: `67` ↔ `80`
- MJCF geom names: both `UNNAMED`; deterministic resolved names are included in the contact CSVs
- mesh assets: `left_wrist_roll_link` ↔ `right_wrist_roll_link`
- classification: `CONTROLLER_POSTURE_SELF_CONTACT`
- semantic interpretation: repeated cross-arm end-effector surface contact at the three Clap closures; exact real contact surface still needs video/physical verification

The pair is absent throughout pre-roll: minimum pre-roll separation is `0.449011 m`. It first penetrates at about `1.602–1.604 s`, then repeats near `2.778 s` and `3.978 s`. This is not an initial model overlap.

## Pair summary

| condition | mode | body1_name | body2_name | episodes | first_onset_s | total_contact_duration_s | max_episode_duration_s | max_penetration_mm | peak_normal_force_n | max_abs_relative_normal_velocity_m_s | classification |
|---|---|---|---|---|---|---|---|---|---|---|---|
| mass_direction | arm_only | left_wrist_roll_link | right_wrist_roll_link | 3 | 1.604000 | 0.524000 | 0.184000 | 1.177893 | 10.496856 | 0.197421 | CONTROLLER_POSTURE_SELF_CONTACT |
| mass_direction | whole_body | left_wrist_roll_link | right_wrist_roll_link | 3 | 1.602000 | 0.523000 | 0.186000 | 1.104949 | 10.418687 | 0.194325 | CONTROLLER_POSTURE_SELF_CONTACT |
| original | arm_only | left_wrist_roll_link | right_wrist_roll_link | 3 | 1.603000 | 0.527000 | 0.185000 | 1.130048 | 10.460880 | 0.197001 | CONTROLLER_POSTURE_SELF_CONTACT |
| original | whole_body | left_wrist_roll_link | right_wrist_roll_link | 3 | 1.602000 | 0.522000 | 0.185000 | 1.158956 | 10.500154 | 0.193712 | CONTROLLER_POSTURE_SELF_CONTACT |

## Episode-aligned baseline versus +8%

| mode | episode | same_geom_pair | baseline_onset_s | candidate_onset_s | onset_delta_ms | baseline_duration_s | candidate_duration_s | duration_delta_ms | baseline_max_penetration_mm | candidate_max_penetration_mm | penetration_delta_mm | baseline_peak_normal_force_n | candidate_peak_normal_force_n | peak_force_delta_n |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| arm_only | 1 | True | 1.603000 | 1.604000 | 1.000000 | 0.158000 | 0.157000 | -1.000000 | 1.130048 | 1.177893 | 0.047846 | 10.460880 | 10.496856 | 0.035975 |
| arm_only | 2 | True | 2.778000 | 2.779000 | 1.000000 | 0.184000 | 0.183000 | -1.000000 | 1.071627 | 1.134662 | 0.063036 | 10.091179 | 10.170170 | 0.078990 |
| arm_only | 3 | True | 3.978000 | 3.978000 | 0.000000 | 0.185000 | 0.184000 | -1.000000 | 1.097206 | 1.062451 | -0.034755 | 10.186123 | 10.163295 | -0.022828 |
| whole_body | 1 | True | 1.602000 | 1.602000 | 0.000000 | 0.153000 | 0.153000 | 0.000000 | 1.158956 | 1.104949 | -0.054006 | 10.500154 | 10.418687 | -0.081467 |
| whole_body | 2 | True | 2.778000 | 2.778000 | 0.000000 | 0.184000 | 0.184000 | 0.000000 | 1.133272 | 1.074476 | -0.058796 | 10.164761 | 10.109710 | -0.055051 |
| whole_body | 3 | True | 3.977000 | 3.977000 | 0.000000 | 0.185000 | 0.186000 | 1.000000 | 1.133035 | 1.084240 | -0.048795 | 10.238908 | 10.174034 | -0.064875 |

## Classification rationale

- `EXPECTED_ADJACENT_LINK_CONTACT`: **NO**. The bodies are on separate left/right arm branches, not parent-child or grandparent-child.
- `MODEL_GEOMETRY_OVERLAP_CANDIDATE`: **NO**. Pre-roll separation is about 0.449 m and no internal contact occurs before the motion.
- `CONTROLLER_POSTURE_SELF_CONTACT`: **YES**. Exactly three finite-duration episodes follow the three Clap closures, with the same pair in baseline and candidate.
- `NUMERICAL_TRANSIENT`: **NO**. Durations are 0.153–0.186 s, penetration reaches `1.178 mm`, and normal force reaches `10.500 N`; the episodes are repeatable rather than isolated one-step events.
- `UNKNOWN`: **NO** for the simulation root cause. Whether the real robot's physical palms/housings touched in the captured Clap remains `NEEDS_PHYSICAL_VERIFICATION`.

The source collision meshes correspond to the actual wrist-roll/end-effector link surfaces, so this is a physically possible robot-surface contact. No `reported_effort` is used.
