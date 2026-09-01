# Phase 2D replay reference report

- `PHASE2D_REPLAY_READY = YES`.
- Basis: all six required streams present; no clean footer; all required streams still cover full motion and >=5 s post-roll; heart motion detected from arm velocity; pre-roll >=5 s; post-roll >=5 s; required source timestamps monotonic; required receive gaps <0.5 s.
- `phase2d_heart_position_reference.csv` retains measured arm position/velocity/effort with source and receive timestamps from 5 s before through 5 s after detected motion.
- `phase2d_heart_normalized.csv` expresses measured position relative to the pre-motion baseline and normalized motion phase from 0 to 1.
- `command_position` is empty/`UNKNOWN` because the MC-internal preset trajectory is not published in the captured state interface.
- This is a measured replay reference only. It must not be interpreted as dynamics calibration or as evidence for unknown hardware/MuJoCo sign and zero fields.
