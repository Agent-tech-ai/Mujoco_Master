# Phase 2D capture summary

## Result

- Robot: `agi` at `192.168.4.114 10.0.1.41 10.11.1.1 10.0.200.41 100.94.200.26 fd13:49ec:df4e:1:d70c:4493:a8d2:dd2e fd13:49ec:df4e:1:baf8:987b:5318:ea88 `.
- Robot serial: `X240026C3Z0008` is confirmed by the operator's SSH banner; the non-interactive `AGIBOT_SN` environment value was empty.
- Capture window: 180.0 s requested; 31.866 s actually represented by received samples; termination: `MANUAL_OR_CONNECTION_END_AFTER_POST_ROLL`.
- Required-topic ready marker: `True`.
- Robot-control calls made by recorder: **none**. Startup evidence declares subscription-only/no publish/no service-action; the normal footer was absent because the stream ended manually or disconnected after post-roll.
- Detected heart motion: `14.590 s` to `20.250 s` relative to recorder start, using |joint velocity| > 0.0500 rad/s.
- All-required-stream pre-roll: 13.78 s. All-required-stream post-roll: 11.60 s.
- Observed MC input source values: `app_proxy`; observed MC action descriptions: `STAND_DEFAULT`.
- `PHASE2D_REPLAY_READY = YES`.

## Files

Raw source rows and source/receive timestamps are retained in `C:\Users\xinga\Documents\Codex\2026-08-11\master-mujoco-agibot-x2-ultra-mujoco\Master_MuJoCo\calibration\logs\real\phase2d_heart_001`. Reports are analysis-only: no MJCF, joint mapping, dynamics, controller, or robot configuration was modified.
