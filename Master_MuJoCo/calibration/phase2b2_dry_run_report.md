# Phase 2B-2 dry-run report

Status: **NO-GO / DRY-RUN ONLY**

- Joint: `left_wrist_roll_joint`
- Saved snapshot: `2026-08-12T16:35:06.797402172-04:00`
- Current: -4.382304°
- Field limits: -90.012° to 41.482°
- Selected symmetric amplitude: 2.0°
- Hypothetical + target: -2.382304012993541°
- Hypothetical - target: -6.382304012993541°
- Saved command publisher count: 1 (['mc_ros2_node2263'])
- Saved MC action: `STAND_DEFAULT`
- Saved input source: `` (empty does not prove ownership)

Dry-run invariants: ROS was not imported; no node or publisher was created; no service was called; no command was sent.

Blockers:

- saved evidence has 1 existing arm-command publisher(s)
- saved snapshot is stale by definition and cannot authorize motion
