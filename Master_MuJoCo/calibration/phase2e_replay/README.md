# Phase 2E/2F offline measured-heart replay

This directory is derived from the accepted read-only capture
`calibration/logs/real/phase2d_heart_001`. The raw capture is never overwritten.

The replay is local simulation only. The scripts do not import ROS, open SSH, or
access robot interfaces. They do not modify MJCF, dynamics, controller gains,
friction, mass, inertia, hardware mapping, sign, zero, or encoder offsets.

Run from the `Master_MuJoCo` directory:

```powershell
python calibration\phase2e_replay\extract_phase2e.py
python calibration\phase2e_replay\run_phase2f_replays.py
python calibration\phase2e_replay\compare_phase2f.py
```

Coordinate warning: all name-matched replay joints use
`q_mujoco = q_real` as an explicit candidate assumption. This is sufficient for
a baseline test but does not confirm physical axis sign or zero.

The source-file hashes and extraction configuration are recorded in
`source_sha256_manifest.csv` and `source_data_lock.json`.
