# Real ↔ MuJoCo comparison report

- Real log: `C:\Users\xinga\Documents\Codex\2026-08-11\master-mujoco-agibot-x2-ultra-mujoco\Master_MuJoCo\calibration\logs\real\test.csv`
- Simulation log: `C:\Users\xinga\Documents\Codex\2026-08-11\master-mujoco-agibot-x2-ultra-mujoco\Master_MuJoCo\calibration\logs\sim\test.csv`
- Timestamp mode: `relative`
- This report is diagnostic only. It does not modify mapping or MJCF parameters.

| Joint | corr(real, sim) | offset candidate (rad) | scale candidate | real delay (s) | sim delay (s) | flags |
|---|---:|---:|---:|---:|---:|---|
| head_yaw_joint | -0.9746 | -0.011988 | -0.97146 | 0.070234 | 0.020067 | POSSIBLE_SIGN_MISMATCH: strong negative position correlation; verify hardware sign<br>POSSIBLE_RESPONSE_DELAY: real and simulation command-response delays differ by +0.0502 s |
| left_knee_joint | 0.9869 | +0.24607 | 1.2641 | 0.070234 | 0.020067 | POSSIBLE_ZERO_OFFSET_MISMATCH: median real-sim offset +0.2461 rad<br>POSSIBLE_POSITION_SCALE_MISMATCH: fitted real/sim scale 1.264<br>POSSIBLE_RESPONSE_DELAY: real and simulation command-response delays differ by +0.0502 s |
| right_shoulder_roll_joint | 0.958 | +0.0050918 | 0.94322 | 0.070234 | 0.020067 | POSSIBLE_RESPONSE_DELAY: real and simulation command-response delays differ by +0.0502 s |

## Interpretation guardrails

Flags are candidates based on signal correlation, offsets, spans, and cross-correlation delay. They are not calibrated values. Confirm them against robot zeroing, encoder conventions, clock provenance, controller mode, and repeated experiments before editing MuJoCo.
