# Phase 2E/2F final baseline conclusions

## Provenance and restrictions

- Input: accepted Phase 2D capture `phase2d_heart_001`.
- Raw inputs are locked by `source_sha256_manifest.csv`; derived work is contained in `calibration/phase2e_replay/`.
- Unified timeline: 50 Hz, piecewise-linear joint interpolation, quaternion Slerp for IMU orientation, `t=0` at detected heart motion start.
- Replay input is `MEASURED_REAL_TRAJECTORY`, not the MC internal command.
- Replay coordinate transform is an explicit `q_mujoco=q_real` name-match candidate; physical sign and zero remain unverified.
- No MJCF, dynamics, gain, friction, mass, inertia, gear, torque limit, hardware sign, zero, or encoder-offset value was changed.

## 1. Primary gesture joints

The primary joints on both arms are:

- J2 shoulder roll
- J3 shoulder yaw
- J5 wrist yaw
- J7 wrist roll

Secondary gesture joints are J1 shoulder pitch and J4 elbow on both sides. J6 wrist pitch is effectively static in this heart capture.

## 2. Measured balance-compensation candidates

Dynamic magnitude, onset/recovery timing, and lagged correlation with dominant arm activity identify:

- left/right ankle pitch
- left/right hip pitch
- left/right knee
- waist pitch
- waist roll

These are `BALANCE_COMPENSATION_CANDIDATE`, not a recovered MC law. Ankle roll, hip roll/yaw, waist yaw, and head joints remained below the classification threshold for this capture.

## 3. J2/J7 mirror confirmation

Yes, for the real **measured coordinate** in this full dynamic capture:

- J2 shoulder roll: mirrored correlation `0.999996`; excursions left/right `2.20534/2.20281 rad`.
- J7 wrist roll: mirrored correlation `0.999961`; excursions left/right `1.07830/1.09102 rad`.

Shoulder yaw and wrist yaw are also mirrored. Shoulder pitch and elbow are same-sign. Wrist pitch has insufficient excursion. None of these results updates the hardware↔MuJoCo sign field.

## 4. Head-pitch mismatch

The real `head_pitch_joint` had exactly `0 rad` excursion and `0 rad/s` peak velocity. The model still has a structural DOF difference, but it is `NO_MATERIAL_DOF_MISMATCH_FOR_THIS_MOTION`; retaining the fixed treatment is valid for this replay.

## 5. Replay 1 largest MC-vs-simulation response difference

The largest **dynamic delta-response** error is left ankle pitch:

- real excursion: `0.04621 rad`
- simulated autonomous excursion: `0.07313 rad`
- delta-coordinate RMSE: `0.02750 rad`

Right hip pitch is nearly equal in delta RMSE (`0.02729 rad`). Both simulated knees substantially over-respond in excursion, while simulated waist roll under-responds (`0.00041 rad` versus real `0.01022 rad`).

For absolute q comparison, the largest mismatch is the simulated equilibrium offset at left/right knee (RMSE `0.1180/0.1137 rad`). Replay 1 holds those reference targets at their measured initial values, but the current free-base controller/model settles away from them before the gesture. This is a standing-equilibrium controller/physical-model mismatch candidate distinct from the gesture-induced delta mismatch.

Relative IMU response is also stronger/slower in simulation: simulated relative pitch peaks at `0.03664 rad`, versus chest `0.01364 rad` and torso `0.02648 rad`; candidate pitch lag is approximately `0.34–0.46 s`. IMU frame remains unknown, so this is a relative-shape comparison only.

## 6. Replay 2 sign/range/kinematic checks

- No measured reference exceeded the current model range.
- No target clipping or runtime joint-limit contact occurred.
- No self-collision or non-foot ground contact occurred.
- No q-tracking sign conflict appeared under the identity-coordinate assumption.
- Fast wrist-yaw and shoulder-roll trajectories have the largest tracking errors. Wrist-yaw RMSE is about `0.341 rad` with a `0.38 s` lag candidate; shoulder-roll RMSE is about `0.300 rad` with a `0.24 s` lag candidate.

Because position targets were applied using an unverified identity transform, this does not physically confirm hardware↔MuJoCo sign or zero.

## 7. Stability, limits, and collisions

Both replays remained upright without a fall. Neither replay contacted a joint limit, self-collided, or produced non-foot ground contact. Both feet were simultaneously in contact for about `99.87%` of samples. The foot-body displacement proxy reached approximately `10.0 mm` left and `6.5 mm` right; maximum contact penetration was `1.51 mm`. Those contact values remain physical-model/contact candidates rather than calibrated truth.

## 8. Controller mismatch candidates

- Replay 1 leg/waist response amplitude and phase differences, especially ankle pitch, hip pitch, knees, and waist roll.
- Replay 2 wrist/shoulder tracking lag and associated large fast-motion errors.
- Relative pitch and gyro response timing differences.

These results describe the current simulation stability/tracking controller only; they are not X2 hardware-gain estimates.

## 9. Physical-model mismatch candidates

- Foot-slip proxy and penetration behavior.
- Base/IMU response magnitude differences.
- Qualitative actuator-force versus reported-effort load-shape differences.

These remain confounded with controller, mapping, contact, and IMU-frame uncertainty. Real `reported_effort` versus MuJoCo actuator force is explicitly `NOT_TORQUE_CALIBRATION_READY`.

## 10. Dynamics-calibration gate

`NO-GO` for dynamics calibration. Required evidence still missing:

- physically verified hardware↔MuJoCo sign and zero
- verified IMU mounting transforms and conventions
- defined provenance of AimDK `reported_effort`
- MC internal commanded trajectory or independently controlled command reference
- multiple controlled motions with varied speeds, amplitudes, poses, and loads

This Phase 2E/2F run establishes a reproducible baseline only. It must not be used to fit mass, inertia, friction, Kp/Kd, gear, or torque limits.
