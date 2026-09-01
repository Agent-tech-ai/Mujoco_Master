# Phase 3D Final Freeze Manifest

Freeze date: 2026-08-31  
Scope: Phase 1 through Phase 3C evidence consolidation  
Status: `PROJECT_FROZEN_PENDING_NEW_PHYSICAL_EVIDENCE`

## Freeze assertions

- No simulation replay, optimization, parameter sweep or controller tuning was run in Phase 3D.
- No robot connection, command, publish, service/action call or configuration change was made.
- No MJCF, scene, controller source, hardware mapping or prior-phase evidence file was modified.
- No calibrated MJCF was created.
- Phase 3D consists only of read-only evidence consolidation and these five reports.

## Required final gates

- `POSITION_SPACE_PHYSICAL_DIRECTION_VALIDATED = YES`
- `EVIDENCE_BACKED_PHYSICAL_MODEL_UPDATE_READY = NO`
- `DYNAMICS_CALIBRATION_READY = NO`
- `CALIBRATED_MJCF_CREATION_ALLOWED = NO`

## Phase 3D package files

| SHA-256 | Bytes | File |
|---|---:|---|
| `c4bc378c1cd95ac8e85743b6a2b1dd498fcacb20737183bdb4cb3565cd099550` | 9356 | `phase3d_master_status.md` |
| `31e13522e9ee4dabb17b256fb2c1b33198b1cf1a292ac58f0b0c1d427806da4c` | 8137 | `phase3d_evidence_matrix.csv` |
| `520a9f54e153adb9bc14858f1ff70df2cb3b4cd64694a7a1db1a9a0f3c0d8abb` | 4770 | `phase3d_open_blockers.md` |
| `dc17a20674bc8a03359267acae609fabd507ed60c9e012b61e8af7d0dcc46b45` | 3782 | `phase3d_recommended_next_actions.md` |

This manifest intentionally does not embed its own hash because that would be self-referential. Its SHA-256 should be recorded externally when the package is transferred.

## Immutable model and mapping baseline

| SHA-256 | Bytes | File |
|---|---:|---|
| `89619295fcc372c57473224130865b2fe4f22e0741f72925fac243805f4353db` | 39960 | `assets/Master/ff_master_ultra.xml` |
| `6d5940490d93f89929af8983a0de900c9e6c0351839163463ae0881d1b9399dd` | 39842 | `assets/Master/ff_master_ultra_x2_limits.xml` |
| `2dc116ce47d09a5105d01372a8456356b6e9881dee4c4947bc7c876757529a08` | 1101 | `assets/Master/scene_x2_fixed.xml` |
| `88483553e15173d09d69f4fca32a466bb022d6dbb805f074ffa89447fc876d0b` | 1025 | `assets/Master/scene_x2_free.xml` |
| `eae1b320d4ace99fe79bd123d70398ff6ac1446b2c33191ae8a68f5a31691c6e` | 8964 | `master_sim/controller.py` |
| `eb723f10257a3e91901d452f881647822ebb9930204035c311a3535001c51b16` | 5852 | `master_sim/model.py` |
| `3975d90f7f9405f3d98f1e19c873fbd5688f02b68d1d239f11a7d12a4fb5ff04` | 33590 | `calibration/joint_mapping.csv` |

## Frozen real-motion evidence

| SHA-256 | Bytes | Evidence |
|---|---:|---|
| `a6757b13753966ec95cc053bd5fcf5b807419d9ba94a5e57c517f3aa72c539d9` | 12571942 | Heart raw serialized evidence |
| `bdb883357fab5c15948dc15404477b5d70f844371f0abdfa14d7f77514a3ce1b` | 3803578 | Heart measured replay reference |
| `185433f6c21e7744cf77f0792ea691d3ad0e9128d993ac4a1aae8a4994472436` | 10156669 | right-Wave raw serialized evidence |
| `bd10471f360fd1eed64c63b8ddd5ee44b5e1ff1bd277af15431b628f8d3899b7` | 3653152 | right-Wave measured replay reference |
| `37a56c53c0d9c769eb90e2ef495269826a49c8614f768769cd6198f53b19f513` | 8597267 | Clap raw serialized evidence |
| `97a617884b6f64e9de704b7c4e739306cec733fa3e83c23d1d650e9a3a686495` | 3470463 | Clap measured replay reference |

Raw capture roles:

- Heart: MC preset 1007 / area 3, accepted output-response baseline.
- Wave: MC preset 1002 / area 2, independent right-arm validation.
- Clap: MC preset 3017 / area 11, independent blind validation with physically confirmed task contact.

The recorders were subscription-only. Motion execution was external operator action through the existing MC-compatible path.

## Frozen simulation controller evidence

| SHA-256 | Bytes | File / role |
|---|---:|---|
| `c351dea8131971e8c05e0a72bff350c8142fe913135d9d87b416080fb0dcd483` | 2297 | Phase 3A simulation controller alignment candidate |
| `7a1aef562fcf40ca376a8a79e111ca3bb4688a6a95773d7834a5fa85d1b37d00` | 41362 | Phase 3A-X constraint-aware implementation |
| `2c3f9b2dbb7150c1557c500d4b02bfcc679b327d7df4e4d54c26ca4837b92b40` | 2422 | Phase 3A-X controller candidate |
| `09a21d7a40b3d9a531ebb59a32f5fbcb2977bb9bdc335b01587af697f18204a2` | 13434 | Phase 3A-Y motion-conditioned implementation |
| `53a693e8a02a34034c1a4124544ab87085c822e8835edbfb9998edd9e728f9ad` | 3614 | Phase 3A-Y controller candidate |
| `288233bcdba015bda24e86991ab89eb7742cebeca39a64181a3c240ce654594d` | 11746 | Phase 3B-S sensitivity implementation |

All are `SIMULATION_ONLY`. The combined controller is frozen for reproducibility, but `VALIDATED_SIM_CONTROLLER_BASELINE = NO` because balance amplitude/timing did not generalize across the required motions.

## Frozen Phase 3C physical-evidence audit

| SHA-256 | Bytes | File |
|---|---:|---|
| `1181aefeb155261b6148bb049a42111950a4c16318b2ba43ccf7b8a74d1d6248` | 4105 | `phase3c_physical_evidence_closure/phase3c_source_lock.md` |
| `a79041949cf2b4dc6bde8f83ed3618e248899c7c684447c0ce7e6e4b8020b730` | 27410 | `phase3c_physical_evidence_closure/phase3c_current_inertial_provenance.csv` |
| `c0df5a2a9ce440656fb453e5ffdc213b948f70176279740e1e69da6bb5a699b1` | 107614 | `phase3c_physical_evidence_closure/phase3c_inertial_source_comparison.csv` |
| `fefce01ca8b2fb5aacf3d65bfa697283864f101e94febe3d8a7b65afc07df8be` | 5376 | `phase3c_physical_evidence_closure/phase3c_mass_evidence.md` |
| `48bfab97b527ff303865939212ec3e3a5c1c963684a67d6d9e1d1b10a8a955f8` | 2685 | `phase3c_physical_evidence_closure/phase3c_evidence_matrix.csv` |

## Decision-report chain

| SHA-256 | File |
|---|---|
| `1de7c6d5bbeca311709d734b7506bbb0c0a814caddc71e4b83df79d9e2222498` | `calibration/phase1_report.md` |
| `9f906ce16e884797a333e96227d190174dbbd1a91928ea2fa6d119cd26eabef4` | `calibration/phase2h_dynamics_gate.md` |
| `e47c56046ce00b85d4d7817ce5f3d7a19c9bef12985e6db821f84d82bcd02609` | `phase3a_position_only/phase3a_before_after_report.md` |
| `88e5e69cfb2dd0fd0130890c6ae6cffd5e3ad78156dff4b5738973e9268273be` | `phase3av_validation/phase3av_final_gate.md` |
| `1514633f3a72769ddd480373670488d1712febb1ff32c2a29fcdcc10c4f554c8` | `phase3ax_constraint_balance/phase3ax_final_gate.md` |
| `7cca7e4f98cd4bb62a68d7234d5a749d5f431cc19117ac14556e7019bf7a2ee3` | `phase3ay_motion_conditioned_balance/phase3ay_final_gate.md` |
| `2dee0320d08a568117ed19952c4f21bfda945c6bbc599bdb14d53628c599bccc` | `phase3bs_physical_sensitivity/phase3bs_final_gate.md` |
| `5f6279e0d6001d34f9ef52e9f40e7578c67a82ee3b02afec132be4bd9df12795` | `phase3bv_physical_direction_validation/phase3bv_final_gate.md` |
| `5bc592306494e6c38375e6fe71000faae2846054125424f1f1a9b6821b4cd406` | `phase3bc_clap_collision_root_cause/phase3bc2_phase3bv_gate_reinterpretation.md` |
| `006a19ff7fb933649119e961c3859e98a478b9df59bd5730ff75ea32e5731632` | `phase3c_physical_evidence_closure/phase3c_final_gate.md` |

## Frozen classifications

Allowed:

- `POSITION_RESPONSE_BASELINE_READY = YES` within its restricted position/controller scope.
- `ARM_TRACKING_GENERALIZES = YES`.
- `CROSS_MOTION_VALIDATED_PHYSICAL_SENSITIVITY_DIRECTION`.
- `POSITION_SPACE_PHYSICAL_DIRECTION_VALIDATED = YES`.
- `MODEL_CONVERSION_PROVENANCE_DISCREPANCY` for the pelvis inertial.

Not allowed:

- `REAL_X2_LOWER_LIMB_MASS = +8%`.
- `HARDWARE_MASS_IDENTIFIED`, `REAL_MASS_CALIBRATION` or `CALIBRATED_MJCF`.
- `REAL_KP`, `REAL_KD`, real damping/friction/armature/gear/torque limit.
- `MEASURED_TORQUE` or any other resolved effort semantic.
- `ACTUATOR_SYSTEM_IDENTIFICATION` or full dynamics calibration.
- physical sign, zero, encoder offset or absolute IMU transform confirmation.

## Reopen rule

The project may be reopened only when new evidence is explicitly linked to a blocker and has a verifiable source, configuration, units, coordinate convention and uncertainty. Sensitivity or better simulation fit alone is not sufficient. Before any derived physical-model edit, an evidence review must change `EVIDENCE_BACKED_PHYSICAL_MODEL_UPDATE_READY` from `NO` to `YES` for the exact affected parameters.
