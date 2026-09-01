# Phase 2F real-vs-sim baseline report

## Most important comparison: arm input → balance response

Real MC and the simulation controller receive comparable measured arm-position shapes, but the simulation's gesture-induced autonomous leg/waist response differs most at `left_ankle_pitch_joint` by delta RMSE 0.027501 rad. Absolute q error is instead dominated by the pre-gesture simulated `left_knee_joint` equilibrium offset (RMSE 0.117996 rad). Ranking and detailed response amplitudes are in the Replay 1 report and `phase2f_replay1_joint_metrics.csv`.

## Relative IMU comparison

| real IMU | quantity | real peak | sim peak | sim-real peak | shape corr | lag candidate s |
| --- | --- | --- | --- | --- | --- | --- |
| chest | relative_roll | 0.00631 | 0.00415 | -0.00215 | 0.457 | -0.920 |
| chest | relative_pitch | 0.01364 | 0.03664 | 0.02300 | 0.892 | 0.460 |
| chest | gyro_norm | 0.10321 | 0.15342 | 0.05021 | 0.495 | 0.500 |
| chest | accel_norm | 10.02624 | 9.87670 | -0.14954 | 0.071 | 0.440 |
| torso | relative_roll | 0.00325 | 0.00415 | 0.00090 | 0.222 | 1.000 |
| torso | relative_pitch | 0.02648 | 0.03664 | 0.01016 | 0.676 | 0.340 |
| torso | gyro_norm | 0.11168 | 0.15342 | 0.04173 | 0.642 | 0.360 |
| torso | accel_norm | 10.05202 | 9.87670 | -0.17532 | 0.097 | 1.000 |

IMU mounting/frame remains `UNKNOWN`. Roll/pitch values are relative to each stream's own pre-motion baseline; gyro/acceleration use norms. Absolute quaternion components are intentionally not compared.

## Reported effort versus simulated actuator force

- Status: `NOT_TORQUE_CALIBRATION_READY`.
- Same-sign shape candidates: 20; opposite-sign shape candidates: 0; remaining joints have weak or insufficient shape relation.
- AimDK `reported_effort` and MuJoCo actuator force are not treated as numerically equivalent. No torque fitting is performed.

## Interpretation

- Controller mismatch candidates: Replay 1 leg/waist amplitude, phase and recovery differences after applying the same measured arm reference.
- Physical-model mismatch candidates: base/IMU magnitude, contact penetration/slip proxy, and actuator-load-shape differences. These remain confounded with controller and frame/mapping uncertainty.
- Mapping/kinematic checks: no reference range conflict, target clipping, limit contact, collision, or fall occurred under the identity-name candidate mapping; physical sign and zero remain unconfirmed.
- Dynamics-calibration gate: **NO-GO**. Missing prerequisites include verified hardware↔MuJoCo sign/zero, verified IMU transforms, a defined torque source, MC internal command trajectory, and multiple controlled excitations. This single preset baseline is insufficient for mass/inertia/friction/Kp/Kd fitting.
