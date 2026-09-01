# Phase 2G calibration readiness

| item | status | basis |
| --- | --- | --- |
| joint name mapping | READY | live JointState.name decoded; exact-name candidate exists |
| joint physical sign | PARTIAL | mirror FIELD_TEST_EVIDENCE; MuJoCo physical sign unconfirmed |
| joint zero / encoder offset | BLOCKED | no documented known physical pose plus measurement |
| IMU transform | PARTIAL | frame_id and upstream URDF known; deployed TF/convention unconfirmed |
| real position | READY | decoded 47 Hz Phase 2D capture |
| real velocity | READY | decoded 47 Hz Phase 2D capture |
| reported effort semantics | BLOCKED | N·m label only; HAL assignment source absent |
| MC internal command | BLOCKED | preset state visible; joint reference UNOBSERVABLE |
| sim controller alignment | PARTIAL | cause separation improved; candidates not adopted |
| contact baseline | PARTIAL | sim contact stable; no real foot contact/force baseline |
| balance baseline | PARTIAL | real/sim relative response exists; IMU axes not transformed |

## Gate

**DYNAMICS_CALIBRATION_READY = NO**

## Minimum conditions for Phase 3

1. Confirm a common IMU comparison frame from deployed TF/driver documentation or a validated static transform.
2. Confirm physical sign and zero for every joint selected for fitting; do not infer these from replay error.
3. Select and validate a simulation controller timing/alignment policy so artificial tracking lag is either materially reduced or explicitly modeled.
4. Treat standing target/balance equilibrium separately from encoder zero; adopt no global target compensation until ankle/knee trade-offs are resolved.
5. If effort is used for torque calibration, confirm its HAL assignment semantics and sign. Otherwise exclude torque fitting.
6. If MC internal q-command remains unavailable, limit Phase 3 to `OUTPUT_RESPONSE_CALIBRATION`; do not claim `ACTUATOR_SYSTEM_IDENTIFICATION`.

All Phase 2G candidates are `SIM_CONTROLLER_ALIGNMENT_CANDIDATE` and **NOT HARDWARE CALIBRATION**. No calibrated MJCF was created.
