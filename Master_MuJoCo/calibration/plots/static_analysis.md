# Static log analysis

- Log: `C:\Users\xinga\Documents\Codex\2026-08-11\master-mujoco-agibot-x2-ultra-mujoco\Master_MuJoCo\calibration\logs\real\test.csv`
- Report-only analysis; no mapping or MuJoCo parameter is changed.

| Joint | samples | position mean (rad) | position std (rad) | drift (rad) | velocity RMS (rad/s) | torque mean (N·m) | torque RMS (N·m) |
|---|---:|---:|---:|---:|---:|---:|---:|
| head_yaw_joint | 301 | -0.000677867 | 0.107029 | -0.0488859 | 0.494315 | 0.0145742 | 0.662268 |
| left_knee_joint | 301 | 0.854953 | 0.292881 | 0.0947963 | 0.967237 | -0.702296 | 0.727295 |
| right_shoulder_roll_joint | 301 | -0.476778 | 0.165183 | -0.183903 | 0.995344 | -0.00892735 | 0.0656617 |

## IMU stability

| Signal | component mean | component std |
|---|---|---|
| imu_quaternion | `[0.9999627893061788, 0.0, 0.0, 9.883782649525975e-05]` | `[2.8080904913209594e-05, 0.0, 0.0, 0.00862609097760356]` |
| imu_gyro | `[0.0, 0.0, 0.0027446638495056993]` | `[0.0, 0.0, 0.0396910598395705]` |
| imu_accel | `[5.944941741400095e-19, 0.0, 9.806649999999992]` | `[0.01411862416005034, 0.0, 7.105427357601002e-15]` |

Maximum quaternion norm error: `1.11022e-16`

## Diagnostic warnings

- head_yaw_joint: position std 0.107 rad; log may not be static
- head_yaw_joint: velocity RMS 0.4943 rad/s; log may not be static
- left_knee_joint: position std 0.2929 rad; log may not be static
- left_knee_joint: velocity RMS 0.9672 rad/s; log may not be static
- right_shoulder_roll_joint: position std 0.1652 rad; log may not be static
- right_shoulder_roll_joint: velocity RMS 0.9953 rad/s; log may not be static
