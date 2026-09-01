# Interactive demo limitations

1. **No locomotion or turning.** The repository has an older actuator-driven single-step experiment in `demo.py`, but it is not a validated continuous gait inside the frozen Phase 3A-X/Y stack. `W/S/A/D/Q/E` therefore report `LOCOMOTION_NOT_AVAILABLE` and create no control command.
2. **Speed levels are simulation-only labels.** `SIMULATION_ONLY_LOW/MEDIUM/HIGH` are reserved for a future gait and do not retime the measured Heart/Wave/Clap references. They are not hardware-calibrated speeds.
3. **Actions are measured output-response replays.** MC internal commands are unobservable; the demo replays processed measured arm position trajectories. This is not actuator system identification.
4. **Controller values remain simulation-only.** Phase 3A-X/Y controller gains, offsets, schedules, and safety mechanisms must not be called real X2 parameters.
5. **No physical-model update is applied.** The cross-motion lower-limb mass sensitivity direction is evidence, not an identified parameter. The original model and all physical values remain frozen.
6. **Clap exception is narrow.** Only the bilateral wrist-roll-link pair during the three documented closure windows is expected task contact. It does not suppress any other self-collision.
7. **Foot slip is observed, not hard-gated by a newly invented threshold.** Phase 3A-X provides the proxy but no frozen interactive hard threshold. The demo records maxima and does not redefine one.
8. **Terminal status replaces HUD.** The existing passive-viewer path does not expose a project-proven lightweight HUD API, so state changes are printed once in the terminal.
9. **`R` is not physical recovery.** It is an explicit simulation Demo reset and intentionally creates a discontinuity through `mj_resetData`. Normal action completion, SPACE stop, and safety stabilization remain physics-continuous.
10. **New demo code does not inject state during motion.** It writes initial joint/root placement only during construction or explicit Demo reset, then uses controller targets, actuator controls, and `mj_step`. The unchanged Phase 3A-X safety shell internally performs restored finite-difference `qpos` probes around `mj_forward` for contact gradients; no `mj_step` occurs in those temporary probes.
11. **Hardware blockers remain.** Physical joint sign/zero are unknown, effort semantics are unknown, IMU transform is partial, and MC internal command is unobservable. `DYNAMICS_CALIBRATION_READY` remains `NO`.

This demo readiness must not be interpreted as a calibrated MJCF, hardware mass identification, real Kp/Kd, or real walking validation.
