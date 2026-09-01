# AgiBot X2 Ultra calibration environment — Phase 2B-2 offline preparation

This directory is deliberately read-only with respect to the robot and report-only with respect to model calibration. It can inspect MJCF, validate a shared data schema, align logs, generate plots, and flag candidate discrepancies. It does not publish ROS topics, call motion-mode setters, or edit MuJoCo parameters.

## Commands

Inspect all four requested MJCF files using compiled/effective values:

```powershell
python calibration/inspect_model.py
```

Save text or machine-readable output:

```powershell
python calibration/inspect_model.py --output calibration/plots/model_inspection.txt
python calibration/inspect_model.py --format json --output calibration/plots/model_inspection.json
```

Compare logs and generate one four-panel plot per common joint plus IMU orientation/gyro/acceleration plots:

```powershell
python calibration/compare_real_sim.py `
  --real calibration/logs/real/test.csv `
  --sim calibration/logs/sim/test.csv
```

`test.csv` is synthetic smoke-test data, not robot data. Real and simulation clocks default to relative-time alignment; use `--time-mode absolute` only when both logs share a verified clock.

Static and single-joint analyses:

```powershell
python calibration/analyze_static.py --log calibration/logs/real/test.csv
python calibration/analyze_single_joint.py `
  --log calibration/logs/real/test.csv `
  --sim calibration/logs/sim/test.csv `
  --joint left_knee
```

The first passive real-robot capture is available as:

```powershell
python calibration/analyze_static.py --log calibration/logs/real/static_001.csv
```

Phase 2B-1 source-evidence processing is reproducible with:

```powershell
python calibration/prepare_phase2b_static_evidence.py
python calibration/ingest_phase2b_capture.py `
  --evidence ..\work\x2_phase2b_static_capture.txt `
  --output-log calibration/logs/real/static_001.csv
```

Export a fixed-base MuJoCo run directly into the same schema:

```powershell
python calibration/export_sim_log.py --duration 2 --rate 100 --output calibration/logs/sim/mujoco.csv
```

The simulation `measured_torque` field is the direct-drive MuJoCo `actuator_force`. Equivalence to the physical robot's `JointState.effort` remains unverified and must not be assumed.

Phase 2B-2 offline preparation and simulation rehearsal:

```powershell
python calibration/prepare_phase2b2_offline.py
python calibration/active_tests/phase2b2_active_test.py --dry-run
python calibration/phase2b2_sim_rehearsal.py --resume
```

The dry-run command consumes saved evidence by default. It does not import ROS, create a ROS node or publisher, or send a command. Physical execution remains gated behind both `--enable-motion` and a complete machine-readable operator gate; the supplied gate is deliberately `NO-GO`.

Standing-stability cleanup is reproduced locally with:

```powershell
python calibration/standing_stability/run_standing_cleanup.py
```

It runs no robot interface. The accepted cleanup is controller-side and marked `SIMULATION_STABILITY_CANDIDATE`; no MJCF, mapping, mass, inertia, or friction parameter is changed.

Phase 2C's offline-only single-joint plan printer is run with:

```powershell
python calibration/phase2c_dry_run.py --dry-run --joint left_wrist_roll_joint --delta-deg 1
```

It has no execution mode and creates no ROS node, publisher, client, SDK
session, or SSH connection. Discovery confirmed that standing `heart()` uses a
native MC preset, but no approved MC-native arbitrary single-joint interface
was found. The output therefore remains a geometric offline plan, not a GO
authorization.

## Mapping rules

`joint_mapping.csv` has 31 X2 positions: 14 arm, 12 leg, 3 waist, and 2 head slots. Phase 2B-1 confirmed every `hardware_group_index` directly from a stable, populated live `JointState.name`; it is not a motor ID. Thirty live names exactly equal current MuJoCo joint names. The reserved `head_pitch_joint` is live but has no joint in the X2-limit MJCF.

Official limits, separately supplied field-test limits, and current MJCF limits remain distinct evidence columns. Existing mirrored/reversed MJCF ranges and exact string-name matches do not establish hardware↔MuJoCo physical `sign`; all hardware IDs, zero positions, signs, and encoder offsets remain `UNKNOWN`.

The real CSV keeps the shared nine-column schema. Joint rows contain raw `JointState` position, velocity, and effort. Reserved `__imu_chest__` and `__imu_torso__` rows contain IMU samples. AimDK documents effort as torque in N·m, but measured-versus-estimated physical origin remains unconfirmed.

## Safety boundary

Before any robot motion is considered, verify on the actual robot and firmware:

- physical emergency stop and recovery procedure;
- current control mode and safe mode-transition procedure;
- exact command topic/service/message and arbitration ownership;
- joint position limits in the robot coordinate convention;
- per-joint velocity and torque limits;
- support/fixture state, exclusion zone, operator, and low-energy test plan.

Until all are confirmed, only state subscription and offline log conversion are permitted.
