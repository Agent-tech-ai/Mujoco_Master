# Phase 3B-C collision-topology audit

| Check | Evidence-backed result |
|---|---|
| bodies | `left_wrist_roll_link` ↔ `right_wrist_roll_link` |
| parent-child | `FALSE` |
| grandparent-child | `FALSE` |
| kinematic-tree adjacent | `FALSE` |
| lowest common ancestor | `torso_link` |
| edges from bodies to common ancestor | `7` / `7` |
| geom types | `mjGEOM_MESH` / `mjGEOM_MESH` |
| geom masks | contype/conaffinity `1/1` and `1/1` |
| collision enabled by masks | `TRUE` |
| model explicit excludes | `0` |
| pair explicitly excluded | `FALSE` |
| source collision representation | full mesh `left_wrist_roll_link` / `right_wrist_roll_link` |

Body 1 chain: `left_wrist_roll_link -> left_wrist_pitch_link -> left_wrist_yaw_link -> left_elbow_link -> left_shoulder_yaw_link -> left_shoulder_roll_link -> left_shoulder_pitch_link -> torso_link -> waist_pitch_link -> waist_yaw_link -> pelvis -> world`

Body 2 chain: `right_wrist_roll_link -> right_wrist_pitch_link -> right_wrist_yaw_link -> right_elbow_link -> right_shoulder_yaw_link -> right_shoulder_roll_link -> right_shoulder_pitch_link -> torso_link -> waist_pitch_link -> waist_yaw_link -> pelvis -> world`

The pair is a left/right end-effector pair on separate branches. Such surfaces are normally expected to remain collision-enabled; automatically excluding them would hide physically possible hand-to-hand or hand-to-object contact. The contact therefore does **not** demonstrate a parent/adjacent-link filter error.

`MJCF_COLLISION_FILTER_REVIEW_REQUIRED = NO`

This is an audit conclusion only. No mask, geometry, contype/conaffinity, or MJCF file was changed.
