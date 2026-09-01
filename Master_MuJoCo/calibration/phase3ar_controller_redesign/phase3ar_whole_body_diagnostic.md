# Phase 3A-R whole-body measured-reference diagnostic

This is a `SIMULATION_REPLAY_STABILITY_TEST`, not a claim about real-robot stability
and not an observable MC command replay.

| scenario | event | sim s | details |
| --- | --- | --- | --- |
| current_07__wave__whole_body | FIRST_LARGE_TRACKING_ERROR | 0.120 | joint=right_knee_joint; error=-0.104160 rad |
| current_07__wave__whole_body | FIRST_PELVIS_HIP_CONTACT | 0.540 | side=left; dist=-0.000054285 m |
| current_07__wave__whole_body | FIRST_CONTACT_OVER_TOLERANCE | 0.780 | side=left; dist=-0.000516290 m |
| current_07__wave__whole_body | FIRST_LIMIT_VIOLATION | 3.400 | joint=left_ankle_roll_joint; margin=-0.000149 rad |
| current_07__wave__whole_body | FIRST_BALANCE_EXCURSION_GT_10_DEG | 6.300 | roll=-0.175890; pitch=0.091308 |
| current_07__wave__whole_body | FIRST_ACTUATOR_SATURATION | 7.160 | joint=left_ankle_pitch_joint; fraction=0.981778 |
| current_07__wave__whole_body | FALL_THRESHOLD | 8.040 | z=0.507768; roll=-0.792345; pitch=0.514636 |
| phase3ar_final_candidate__wave__whole_body | FIRST_LARGE_TRACKING_ERROR | 0.140 | joint=right_knee_joint; error=-0.107355 rad |
| phase3ar_final_candidate__wave__whole_body | FIRST_PELVIS_HIP_CONTACT | 0.580 | side=left; dist=-0.000005346 m |
| phase3ar_final_candidate__wave__whole_body | FIRST_CONTACT_OVER_TOLERANCE | 1.000 | side=left; dist=-0.000500920 m |
| phase3ar_final_candidate__wave__whole_body | FIRST_BALANCE_EXCURSION_GT_10_DEG | 6.140 | roll=-0.025816; pitch=0.174573 |
| phase3ar_final_candidate__wave__whole_body | FIRST_ACTUATOR_SATURATION | 6.860 | joint=left_ankle_pitch_joint; fraction=0.985166 |
| phase3ar_final_candidate__wave__whole_body | FIRST_LIMIT_VIOLATION | 7.220 | joint=right_ankle_pitch_joint; margin=-0.012811 rad |
| phase3ar_final_candidate__wave__whole_body | FALL_THRESHOLD | 7.460 | z=0.278733; roll=0.617416; pitch=0.776386 |

Current fall: `8.037 s`; final-candidate fall:
`7.450 s`. Current/final minimum limit margin:
`-0.04501/-0.06282 rad`.
Current/final persistent saturation fraction:
`0.11708/0.11031`.

For the current replay the causal order is **`TRACKING_FIRST`**: large tracking
error, pelvis/hip contact, limit violation, large balance excursion, saturation,
then fall. The final candidate does not correct the whole-body tracker conflict
and falls earlier. Root classification:
`WHOLE_BODY_TRACKER_DRIVES_SELF_CONTACT_AND_LIMIT_COUPLING`; saturation is a later
consequence, not the initiating event.
