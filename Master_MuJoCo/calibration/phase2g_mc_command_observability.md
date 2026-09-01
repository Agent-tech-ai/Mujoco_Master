# Phase 2G MC command observability

Final status: **MC_INTERNAL_COMMAND = UNOBSERVABLE** with current evidence.

- Readable and captured: `/aima/mc/common/state`, including `input_source=app_proxy`, `STAND_DEFAULT`, FSM/body/status fields. It did not expose per-joint q(t) during heart.
- Readable by the validated wrapper when task ID is known: `GetMcPresetMotionState`. It exposes preset task state, not the internal joint trajectory.
- Present in the Phase 2D graph but not captured/decoded as command targets: `/aima/mc/body_pose`, `/aima/mc/manipulation`, `/aima/mc/rl/debug`. Topic names and types are insufficient to assert they contain heart joint targets.
- HAL `/aima/hal/joint/*/command` topics exist, but no evidence proves a passive subscription is an authoritative MC internal target, nor distinguishes competing publishers/arbitration. They were not used as Phase 2D command truth.
- No debug mode was enabled and no state-changing service/action was called.

Preset execution state is partially observable; the MC per-joint reference is not. Later work may be called `OUTPUT_RESPONSE_CALIBRATION` if it uses known preset/output measurements. It must not be called `ACTUATOR_SYSTEM_IDENTIFICATION` without an observable, time-aligned actuator input.
