# X2 MuJoCo standing-stability cleanup

Status: `SIMULATION_STABILITY_CANDIDATE` — not dynamics calibration and not hardware-matched.

Reproduce the full local experiment matrix from the `Master_MuJoCo` directory:

```powershell
python calibration/standing_stability/run_standing_cleanup.py
```

Refresh derived reports and audit tables without rerunning simulation:

```powershell
python calibration/standing_stability/run_standing_cleanup.py --reports-only
```

Run the accepted free-base candidate:

```powershell
python run_simulator.py --free-base --headless --duration 10
```

Reproduce the old deterministic fall:

```powershell
python run_simulator.py --free-base --legacy-controller --headless --duration 10
```

The accepted change is in `master_sim/controller.py`: a separate simulation-only pelvis roll/pitch feedback layer plus smooth compensation for the MJCF's existing frictionloss deadband. No MJCF, joint mapping, mass, inertia, friction, hardware sign/zero, or encoder offset was changed.

Raw per-timestep baseline and sensitivity data is under `data/`; full contact position/normal/force data is in the corresponding `*_contacts.csv` files. Cleanup rehearsal CSVs and plots are under `rehearsal_after/`.

This workflow contains no SSH, ROS, or robot-access code.
