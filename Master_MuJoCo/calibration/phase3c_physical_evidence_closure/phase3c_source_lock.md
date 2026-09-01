# Phase 3C Source Lock

## Scope

Phase 3C is an evidence audit only. No robot connection, robot command, simulation rollout, parameter sweep, controller change, MJCF edit, or calibrated model creation was performed.

The following states remain frozen:

- Phase 3A/3A-Y simulation controller architecture and accepted controller baseline.
- Phase 3B-S physical sensitivity experiment design and results.
- Phase 3B-V Heart/Wave/Clap validation results.
- `bs_mass_lower_plus08` as `CROSS_MOTION_VALIDATED_PHYSICAL_SENSITIVITY_DIRECTION` only.
- Phase 3B-C2 classification of the clap wrist-to-wrist pair as a motion-specific `EXPECTED_TASK_CONTACT`.

## Locked primary inputs

| SHA-256 | File / evidence |
|---|---|
| `89619295fcc372c57473224130865b2fe4f22e0741f72925fac243805f4353db` | `assets/Master/ff_master_ultra.xml` |
| `6d5940490d93f89929af8983a0de900c9e6c0351839163463ae0881d1b9399dd` | `assets/Master/ff_master_ultra_x2_limits.xml` |
| `ccea5e27f8e5fd4f13381b2af7c8f61c0da7cf75e4d11a07c2af2bfc1ea4783d` | `assets/Master/ff_master_ultra.urdf` |
| `3975d90f7f9405f3d98f1e19c873fbd5688f02b68d1d239f11a7d12a4fb5ff04` | `calibration/joint_mapping.csv` at Phase 3C start |
| `09a21d7a40b3d9a531ebb59a32f5fbcb2977bb9bdc335b01587af697f18204a2` | `phase3ay_core.py` |
| `7a1aef562fcf40ca376a8a79e111ca3bb4688a6a95773d7834a5fa85d1b37d00` | `phase3ax_core.py` |
| `288233bcdba015bda24e86991ab89eb7742cebeca39a64181a3c240ce654594d` | `phase3bs_core.py` |
| `5f6279e0d6001d34f9ef52e9f40e7578c67a82ee3b02afec132be4bd9df12795` | frozen Phase 3B-V final gate |
| `4ffce397b4de65255ec970b4ba18a644670a50b13f63c975d821bd1dedd0737d` | Phase 3B-C2 real clap contact evidence |
| `5bc592306494e6c38375e6fe71000faae2846054125424f1f1a9b6821b4cd406` | Phase 3B-C2 gate reinterpretation |
| `39811fa9057c5b369782619055895edd6a1dd828cb1775ab66abf209df96a18e` | Phase 2H sign/zero evidence |
| `297274d420593a6e4c2125fd9a398b80d8fad141e785f8c282e2f963d671e982` | Phase 2H IMU transform closure |
| `f3e89c86e03be493aa16cad1bbe3faf769bd4218072586e62784a15972c6fe1e` | Phase 2H effort semantics closure |
| `2fae16b7869fd37d0c81a6ec835cb083f900d41e7a8d6d67e90a289b3976adfe` | Phase 2G MC command observability |
| `8fe7a087490e4788fa42dc313b3d3af939503da5a4a7531396b004233e5bbb5d` | Phase 2H read-only soft-engineer evidence capture |

The captured AimDK SDK artifact `.../AimDK_X2_SDK_v1.0.0/.../model_info/x2.xml` has SHA-256 `3ff43f05beb57412a804ba9fe05cd9adcdfce78e9ce73a95a71ac58ad20d91a3`. Only excerpts already present in the locked Phase 2H evidence are used; the remote source was not accessed in Phase 3C.

## Phase 3C reproducible audit artifacts

| SHA-256 | Artifact |
|---|---|
| `5e0605cc4b2f57b9c0c471b4612aec9ed4734edbe94f440d42a3b040a4a44718` | `build_phase3c_inertial_audit.py` |
| `a79041949cf2b4dc6bde8f83ed3618e248899c7c684447c0ce7e6e4b8020b730` | `phase3c_current_inertial_provenance.csv` |
| `c0df5a2a9ce440656fb453e5ffdc213b948f70176279740e1e69da6bb5a699b1` | `phase3c_inertial_source_comparison.csv` |

The audit script compiles MJCF solely to read the resolved body inertials. It does not step the simulation.

## Evidence hierarchy

- Level A: official manufacturer data/source.
- Level B: deployed robot description or driver source.
- Level C: direct, read-only hardware observation with an identifiable physical meaning.
- Level D: simulation/replay inference.
- Level E: assumption, same-family model reuse, or unverified conversion lineage.

Only A/B/C evidence with a complete semantic chain can close a hardware parameter. Manufacturer simulator source proves provenance, but is not automatically physical metrology.

## Source search result

Local and previously captured evidence contains the Robothon Master MJCF/URDF family and an AimDK X2 SDK MuJoCo artifact. No independent X2 CAD mass-properties export, load-cell/scale measurement, per-link mass table, inertial YAML, complete deployed Xacro/URDF with documented metrology provenance, encoder datum specification, deployed IMU mounting transform, HAL effort assignment source, or observable MC joint-reference stream was found.

