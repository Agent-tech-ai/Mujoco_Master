# Phase 2B-2 safe joint candidates

Status: **OFFLINE GEOMETRIC SCREENING ONLY — ROBOT REMAINS NO-GO**

Snapshot: `2026-08-12T16:35:06.797402172-04:00` from `run@192.168.4.114`; source `READ_ONLY_PREFLIGHT: ../../work/x2_phase2b2_preflight.txt`.

Screening reserve: **5.0°** from each FIELD_TEST_EVIDENCE limit. This is a configurable engineering screening value, **not** a vendor-approved safety margin.

Movement labels:

- `PASS_GEOMETRIC_RESERVE`: the listed directional target remains inside limits and retains the screening reserve.
- `INSIDE_LIMIT_RESERVE_FAIL`: the listed directional target is mechanically inside the supplied limits but too close for this screening rule.
- `OUTSIDE_LIMIT`: the listed directional target crosses the supplied limit.

## All-arm margin table

| Joint | Current (°) | Lower (°) | Upper (°) | To lower (°) | To upper (°) | +1° | -1° | +2° | -2° | +3° | -3° | +5° | -5° |
|---|---:|---:|---:|---:|---:|---|---|---|---|---|---|---|---|
| `left_shoulder_pitch_joint` | 22.219507 | -176.471 | 116.883 | 198.690507 | 94.663493 | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE |
| `left_shoulder_roll_joint` | -0.939315 | -3.495 | 171.486 | 2.555685 | 172.425315 | INSIDE_LIMIT_RESERVE_FAIL | INSIDE_LIMIT_RESERVE_FAIL | INSIDE_LIMIT_RESERVE_FAIL | INSIDE_LIMIT_RESERVE_FAIL | PASS_GEOMETRIC_RESERVE | OUTSIDE_LIMIT | PASS_GEOMETRIC_RESERVE | OUTSIDE_LIMIT |
| `left_shoulder_yaw_joint` | -0.422953 | -146.448 | 146.448 | 146.025047 | 146.870953 | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE |
| `left_elbow_joint` | -67.251873 | -134.965 | 0.000 | 67.713127 | 67.251873 | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE |
| `left_wrist_yaw_joint` | 0.697608 | -146.448 | 146.448 | 147.145608 | 145.750392 | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE |
| `left_wrist_pitch_joint` | 0.010928 | -31.971 | 31.971 | 31.981928 | 31.960072 | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE |
| `left_wrist_roll_joint` | -4.382304 | -90.012 | 41.482 | 85.629696 | 45.864304 | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE |
| `right_shoulder_pitch_joint` | 22.658989 | -176.471 | 116.883 | 199.129989 | 94.224011 | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE |
| `right_shoulder_roll_joint` | 1.093131 | -171.486 | 3.495 | 172.579131 | 2.401869 | INSIDE_LIMIT_RESERVE_FAIL | INSIDE_LIMIT_RESERVE_FAIL | INSIDE_LIMIT_RESERVE_FAIL | INSIDE_LIMIT_RESERVE_FAIL | OUTSIDE_LIMIT | PASS_GEOMETRIC_RESERVE | OUTSIDE_LIMIT | PASS_GEOMETRIC_RESERVE |
| `right_shoulder_yaw_joint` | 0.390004 | -146.448 | 146.448 | 146.838004 | 146.057996 | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE |
| `right_elbow_joint` | -67.163982 | -134.965 | 0.000 | 67.801018 | 67.163982 | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE |
| `right_wrist_yaw_joint` | 0.379021 | -146.448 | 146.448 | 146.827021 | 146.068979 | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE |
| `right_wrist_pitch_joint` | -0.710340 | -31.971 | 31.971 | 31.260660 | 32.681340 | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE |
| `right_wrist_roll_joint` | 1.693887 | -41.482 | 90.012 | 43.175887 | 88.318113 | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE | PASS_GEOMETRIC_RESERVE |

## Candidate ordering

Adaptive symmetric amplitude is `min(requested=2.0°, minimum distance - reserve)`. A joint is skipped when the result is below 1.0°. It is never shrunk into the reserve zone.

Ranking score = information value × 10 + low whole-body-impact score × 5 + capped clearance/10. It is an explicit prioritization heuristic, not a safety certificate.

| Rank | Joint | Min current margin (°) | Selected symmetric amplitude (°) | Info | Low impact | Score | Decision / rationale |
|---:|---|---:|---:|---:|---:|---:|---|
| 1 | `left_wrist_roll_joint` | 45.864304 | 2.000000 | 3 | 3 | 49.586 | SELECT: min(requested=2.000°, available=40.864304°); J7 mirrored FIELD_TEST_EVIDENCE; distal, high sign information |
| 2 | `right_wrist_roll_joint` | 43.175887 | 2.000000 | 3 | 3 | 49.318 | SELECT: min(requested=2.000°, available=38.175887°); J7 mirrored FIELD_TEST_EVIDENCE; distal, high sign information |
| 3 | `right_wrist_yaw_joint` | 146.068979 | 2.000000 | 2 | 3 | 40.000 | SELECT: min(requested=2.000°, available=141.068979°); distal joint; low expected whole-body influence |
| 4 | `left_wrist_yaw_joint` | 145.750392 | 2.000000 | 2 | 3 | 40.000 | SELECT: min(requested=2.000°, available=140.750392°); distal joint; low expected whole-body influence |
| 5 | `left_wrist_pitch_joint` | 31.960072 | 2.000000 | 2 | 3 | 38.196 | SELECT: min(requested=2.000°, available=26.960072°); distal joint; low expected whole-body influence |
| 6 | `right_wrist_pitch_joint` | 31.260660 | 2.000000 | 2 | 3 | 38.126 | SELECT: min(requested=2.000°, available=26.260660°); distal joint; low expected whole-body influence |
| 7 | `left_elbow_joint` | 67.251873 | 2.000000 | 2 | 2 | 35.000 | SELECT: min(requested=2.000°, available=62.251873°); moderate distal motion and broad current margin |
| 8 | `right_elbow_joint` | 67.163982 | 2.000000 | 2 | 2 | 35.000 | SELECT: min(requested=2.000°, available=62.163982°); moderate distal motion and broad current margin |
| 9 | `right_shoulder_yaw_joint` | 146.057996 | 2.000000 | 2 | 1 | 30.000 | SELECT: min(requested=2.000°, available=141.057996°); proximal joint; collision envelope more posture-dependent |
| 10 | `left_shoulder_yaw_joint` | 146.025047 | 2.000000 | 2 | 1 | 30.000 | SELECT: min(requested=2.000°, available=141.025047°); proximal joint; collision envelope more posture-dependent |
| 11 | `left_shoulder_pitch_joint` | 94.663493 | 2.000000 | 2 | 1 | 30.000 | SELECT: min(requested=2.000°, available=89.663493°); proximal shoulder pitch; larger whole-body/collision influence |
| 12 | `right_shoulder_pitch_joint` | 94.224011 | 2.000000 | 2 | 1 | 30.000 | SELECT: min(requested=2.000°, available=89.224011°); proximal shoulder pitch; larger whole-body/collision influence |
| 13 | `left_shoulder_roll_joint` | 2.555685 | SKIP | 3 | 1 | 35.256 | SKIP: symmetric available amplitude -2.444315° after 5.000° reserve is below 1.000°; J2 mirrored FIELD_TEST_EVIDENCE; proximal and posture-sensitive |
| 14 | `right_shoulder_roll_joint` | 2.401869 | SKIP | 3 | 1 | 35.240 | SKIP: symmetric available amplitude -2.598131° after 5.000° reserve is below 1.000°; J2 mirrored FIELD_TEST_EVIDENCE; proximal and posture-sensitive |

## Recommendation

J7 wrist roll is the best first *candidate* because it combines distal motion, ample current margin, exact live name matching, and existing mirrored-coordinate FIELD_TEST_EVIDENCE. Wrist yaw and wrist pitch follow. Both J2 shoulder-roll joints are skipped at the current pose under the 5° screening reserve.

Every candidate remains NO-GO for real motion until the operator checklist, control ownership, numeric velocity/acceleration/effort limits, abort behavior, and communications-loss behavior are confirmed.
