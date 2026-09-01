# Phase 3A-R pelvis/hip contact root cause

## Conclusion

Most-supported classification: **`CONTROLLER_POSTURE_ISSUE_WITH_LOW_LEFT_GEOMETRIC_MARGIN`**.

The wave static initial pose has positive left clearance (`1.523 mm`), so the
MJCF is not in pelvis/hip penetration at initialization. The unchanged current
controller creates persistent contact during wave standing alone (maximum
`0.733 mm`) and
deepens it during arm motion (`1.289 mm`).
Heart standing/arm produces no self-contact. Therefore motion is not required;
dynamic settling and attitude correction drive the low-margin wave posture into contact.

The specific collision geoms are MuJoCo geom `2` (`pelvis` collision mesh) and
geom `7` (`col_left_hip_roll_link`). The symmetric right collision geom is `30`
(`col_right_hip_roll_link`). The XML contains a small kinematic asymmetry:
left hip-roll body x offset `-0.000575 m`, right `0 m`. This may explain part of
the lower left margin, but the evidence does **not** prove that the geometry is wrong.

`MJCF_COLLISION_GEOMETRY_FIX_REQUIRED = NO` (not supported by current evidence).

## Static signed-distance check

| dataset | state | left mm | right mm |
| --- | --- | --- | --- |
| heart | raw_initial | 3.361 | 3.570 |
| heart | static_offset_scale_0.25 | 3.361 | 3.570 |
| heart | static_offset_scale_0.50 | 3.361 | 3.570 |
| heart | static_offset_scale_0.75 | 3.361 | 3.570 |
| heart | static_offset_scale_1.00 | 3.362 | 0.000 |
| wave | raw_initial | 1.523 | 0.000 |
| wave | static_offset_scale_0.25 | 1.522 | 2.044 |
| wave | static_offset_scale_0.50 | 1.521 | 2.044 |
| wave | static_offset_scale_0.75 | 1.521 | 2.043 |
| wave | static_offset_scale_1.00 | 1.521 | 2.043 |

The `0.000 mm` right-side entries are non-negative mesh-distance query boundary
values. No active right pelvis/hip contact or negative penetration was present,
so they are not treated as right-side penetration evidence.

## Current wave arm-only first-contact state

- onset: sim `0.560 s`, reference `-4.440 s`
- contact position: `[0.041888, 0.115008, 0.607018] m`
- raw contact normal: `[0.027389, 0.987125, -0.157591]`
- pelvis roll/pitch/yaw: `-0.026932 / 0.038411 / 0.012841 rad`
- CoM xyz: `0.043266 / 0.028396 / 0.693109 m`
- applied pitch/roll feedback: `7.688 / 2.376 N·m`

| joint | q rad | target rad | ctrl N·m | sat fraction |
| --- | --- | --- | --- | --- |
| `left_hip_pitch_joint` | -0.342419 | -0.289971 | 3.985 | 0.034 |
| `left_hip_roll_joint` | -0.050322 | -0.021380 | 3.296 | 0.028 |
| `left_hip_yaw_joint` | 0.003598 | 0.000096 | -0.909 | 0.008 |
| `left_knee_joint` | 0.535276 | 0.417691 | -11.635 | 0.099 |
| `left_ankle_pitch_joint` | -0.229848 | -0.231976 | 6.755 | 0.188 |
| `left_ankle_roll_joint` | 0.074995 | 0.012176 | -2.770 | 0.115 |
| `right_hip_roll_joint` | -0.010148 | 0.011600 | 3.111 | 0.026 |
| `waist_pitch_joint` | -0.025212 | -0.028711 | -2.038 | 0.042 |
| `waist_roll_joint` | -0.000520 | -0.001162 | 0.767 | 0.016 |

## Numerical tolerance

`NUMERICAL_CONTACT_TOLERANCE = 0.500 mm`. Basis: in the accepted Phase 3A
free-standing baseline, all-contact penetration p99 was about `0.421 mm` at
`0.001 s` timestep with the unchanged solver/contact settings; a rounded 0.5 mm
threshold provides a small numerical allowance. Persistent `1.289 mm` penetration
is outside it. The final candidate's `0.486–0.494 mm` persistent contact is below
the threshold but has only `0.006–0.014 mm` margin, so it is not considered robustly resolved.
