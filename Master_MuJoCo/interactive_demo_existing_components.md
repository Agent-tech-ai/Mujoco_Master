# Interactive demo existing-component audit

Scope: local MuJoCo only. The audit did not connect to X2, modify MJCF, or change hardware/calibration evidence.

| Component | Status | Interactive-demo disposition |
|---|---|---|
| `master_sim/model.py::load_model` | FOUND / REUSED | Loads the existing free-base `scene_x2_free.xml`; `validate_model` is run before the viewer starts. |
| `run_simulator.py` / `master_sim/app.py` | FOUND / REUSED PATTERN | The separate entry point preserves its official passive-viewer, `mj_step`, real-time sync, and explicit-reset pattern. Existing files are unchanged. |
| `demo.py` | FOUND / PARTIALLY REUSED | Its Windows/viewer keyboard pattern and explicit demo-reset design were adapted. Its older contact-step locomotion controller was not connected because it is not integrated with the frozen Phase 3A-X/Y controller stack. |
| `master_sim/controller.py` | FOUND / REUSED | `SimulationStabilityController` is the unchanged base used by Phase 3A-X/Y. All motion reaches MuJoCo through actuator control. |
| Phase 3A-X safety shell | FOUND / REUSED | `ConstraintAwareBalanceController`, standing offsets, contact/limit/rate/saturation handling, safety state, and existing thresholds remain unchanged. |
| Phase 3A-Y controller | FOUND / REUSED | `MotionConditionedBalanceController` and the frozen candidate JSON are instantiated without editing either source. Classification remains simulation-only and not hardware calibration. |
| Stable standing | FOUND / REUSED | Heart pre-roll at `t=-5 s` plus frozen Phase 3A-X standing-reference offsets initializes the free-base controller. |
| Heart replay reference | FOUND / REUSED | `calibration/phase2e_replay/phase2e_heart_measured_reference.csv`, motion duration `5.659416987 s`. |
| Wave replay reference | FOUND / REUSED | `calibration/phase3av_validation/phase3av_measured_reference.csv`, motion duration `4.349152726 s`. |
| Clap replay reference | FOUND / REUSED | `calibration/phase3bv_physical_direction_validation/phase3bv_measured_reference.csv`, motion duration `5.443540770 s`. |
| Trajectory interpolation | FOUND / REUSED | Existing Phase 3A `Reference` class, linear interpolation at the validated 50 Hz source rate. Only `t`, name, position, and velocity are loaded; `reported_effort` is never loaded. |
| Joint mapping | FOUND / READ-ONLY | `calibration/joint_mapping.csv` remains unchanged. The demo uses existing same-name MuJoCo trajectory channels; it does not promote unknown physical sign/zero evidence. |
| Safety monitor | FOUND / REUSED + THIN OBSERVER | Phase 3A-X safety state and hard gates are reused. The thin demo observer classifies global self/ground contact and the narrowly scoped expected-Clap exception; it does not alter control output or thresholds. |
| Reset logic | FOUND / REUSED PATTERN | Explicit `R` uses `mj_resetData`, validated standing initialization, `mj_forward`, controller reconstruction, then `STANDING`. This is a Demo reset, not physical recovery. |
| Mature locomotion in frozen stack | NOT_AVAILABLE | `demo.py` contains an older simulation-only single-step controller, but no safe integration with the frozen Phase 3A-X/Y architecture or continuous key-hold contract is validated. It is intentionally not spliced into this demo. |
| Viewer HUD overlay | NOT_AVAILABLE | The installed passive viewer path has no small stable overlay API already used by this project. Concise terminal state/event output is used instead; no GUI framework was added. |

## Frozen-source check

The post-implementation hashes remain the Phase 3D frozen values:

| File | SHA-256 |
|---|---|
| `ff_master_ultra.xml` | `89619295fcc372c57473224130865b2fe4f22e0741f72925fac243805f4353db` |
| `ff_master_ultra_x2_limits.xml` | `6d5940490d93f89929af8983a0de900c9e6c0351839163463ae0881d1b9399dd` |
| `scene_x2_fixed.xml` | `2dc116ce47d09a5105d01372a8456356b6e9881dee4c4947bc7c876757529a08` |
| `scene_x2_free.xml` | `88483553e15173d09d69f4fca32a466bb022d6dbb805f074ffa89447fc876d0b` |
| `master_sim/controller.py` | `eae1b320d4ace99fe79bd123d70398ff6ac1446b2c33191ae8a68f5a31691c6e` |
| `master_sim/model.py` | `eb723f10257a3e91901d452f881647822ebb9930204035c311a3535001c51b16` |
| `calibration/joint_mapping.csv` | `3975d90f7f9405f3d98f1e19c873fbd5688f02b68d1d239f11a7d12a4fb5ff04` |

No calibrated MJCF was created.
