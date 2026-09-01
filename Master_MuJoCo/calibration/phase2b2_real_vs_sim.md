# Phase 2B-2 real versus simulation report

Status: **NOT RUN**

The real-robot safety gate returned NO-GO before motion. Therefore there is no real `+2° / 0 / -2° / 0` sequence to replay and no valid overlay to generate.

An isolated MuJoCo motion would not satisfy the requested same-command comparison and could misleadingly imply that a physical test occurred. Simulation replay and overlay are deferred until a safe real command sequence has been approved and captured.

No MJCF actuator, mass, inertia, friction, Kp/Kd, joint limit, or other dynamics parameter was modified.

