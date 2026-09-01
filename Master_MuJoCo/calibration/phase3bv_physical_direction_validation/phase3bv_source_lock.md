# Phase 3B-V source lock

- Controller architecture: frozen Phase 3A arm tracking + Phase 3A-X safety shell + Phase 3A-Y motion-conditioned response.
- Physical comparison: original baseline versus `bs_mass_lower_plus08` only.
- Candidate label: **SHARED_PHYSICAL_SENSITIVITY_DIRECTION — NOT IDENTIFIED HARDWARE PARAMETER**.
- Source MJCF/scenes are immutable. No calibrated MJCF is created.
- Inertia, damping, friction, armature, gear, limits, controller parameters, hardware mapping, and standing references remain frozen.
- `reported_effort` is excluded. IMU use is relative roll/pitch and gyro only.
- Recorder is subscription-only and cannot invoke a preset. Codex analysis/replay made no robot connection; the real capture was produced by the user-run read-only recorder.


## Third-motion capture lock

- `THIRD_MOTION_CAPTURE = CLAP`
- `CAPTURE_SOURCE = READ_ONLY_REAL_ROBOT_STATE`
- `PRESET_EXECUTION = EXTERNAL_OPERATOR / EXISTING_MC_COMPATIBLE_PATH`
- `RECORDER_SENT_COMMAND = NO`
- absolute path: `C:\Users\xinga\Documents\Codex\2026-08-11\master-mujoco-agibot-x2-ultra-mujoco\Master_MuJoCo\calibration\phase3bv_physical_direction_validation\capture\phase3bv_clap_001\raw_serialized_evidence.txt`
- bytes: `8597267`
- last modified: `2026-08-27T14:51:54.907727-07:00`
- SHA-256: `37a56c53c0d9c769eb90e2ef495269826a49c8614f768769cd6198f53b19f513`

## Blind comparison lock

- original MJCF SHA-256: `89619295fcc372c57473224130865b2fe4f22e0741f72925fac243805f4353db`
- frozen Phase 3A-Y controller SHA-256: `09a21d7a40b3d9a531ebb59a32f5fbcb2977bb9bdc335b01587af697f18204a2`
- frozen +8% candidate/replay implementation SHA-256: `ca3765d62a8faf93c795bb7dd34bcdf1def786e0cba4193c5ec1545231f011ec`
- measured replay input SHA-256: `97a617884b6f64e9de704b7c4e739306cec733fa3e83c23d1d650e9a3a686495`
- baseline/candidate controller config: `IDENTICAL` (enforced before replay)


## SHA-256 manifest

| path | bytes | sha256 |
|---|---|---|
| Master_MuJoCo\assets\Master\ff_master_ultra.xml | 39960 | 89619295fcc372c57473224130865b2fe4f22e0741f72925fac243805f4353db |
| Master_MuJoCo\assets\Master\ff_master_ultra_x2_limits.xml | 39842 | 6d5940490d93f89929af8983a0de900c9e6c0351839163463ae0881d1b9399dd |
| Master_MuJoCo\assets\Master\scene_x2_fixed.xml | 1101 | 2dc116ce47d09a5105d01372a8456356b6e9881dee4c4947bc7c876757529a08 |
| Master_MuJoCo\assets\Master\scene_x2_free.xml | 1025 | 88483553e15173d09d69f4fca32a466bb022d6dbb805f074ffa89447fc876d0b |
| Master_MuJoCo\master_sim\controller.py | 8964 | eae1b320d4ace99fe79bd123d70398ff6ac1446b2c33191ae8a68f5a31691c6e |
| Master_MuJoCo\master_sim\model.py | 5852 | eb723f10257a3e91901d452f881647822ebb9930204035c311a3535001c51b16 |
| Master_MuJoCo\calibration\phase3a_position_only\run_phase3a_experiments.py | 50230 | 5ef5c215f8dbc73101183f43ae89aec1edceebe83142c21a9b5464b3d422d52a |
| Master_MuJoCo\calibration\phase3ar_controller_redesign\phase3ar_core.py | 29122 | e3f126c8be4d34c1e0c1b78af90ca600cf04fe08aa8432b0456ea4b183efa435 |
| Master_MuJoCo\calibration\phase3ax_constraint_balance\phase3ax_core.py | 41362 | 7a1aef562fcf40ca376a8a79e111ca3bb4688a6a95773d7834a5fa85d1b37d00 |
| Master_MuJoCo\calibration\phase3ay_motion_conditioned_balance\phase3ay_core.py | 13434 | 09a21d7a40b3d9a531ebb59a32f5fbcb2977bb9bdc335b01587af697f18204a2 |
| Master_MuJoCo\calibration\phase3ay_motion_conditioned_balance\simulation_motion_conditioned_balance_candidate.json | 3614 | 53a693e8a02a34034c1a4124544ab87085c822e8835edbfb9998edd9e728f9ad |
| Master_MuJoCo\calibration\phase3bs_physical_sensitivity\phase3bs_core.py | 11746 | 288233bcdba015bda24e86991ab89eb7742cebeca39a64181a3c240ce654594d |
| Master_MuJoCo\calibration\phase3bs_physical_sensitivity\phase3bs_analysis_summary.json | 2931 | 6040926cd8ca3ad9e569370cd20f871a79feb509e7e50c05106af0b8ac2680f1 |
| Master_MuJoCo\calibration\phase3bs_physical_sensitivity\phase3bs_sensitivity_matrix.csv | 4153 | d4fef9c57119cfd645a8df6a9ae9e45c715bce11ba27e1af47541ff13a8b411d |
| Master_MuJoCo\calibration\phase3bs_physical_sensitivity\phase3bs_position_comparison_metrics.csv | 70322 | b4114a4c8bc422829c44e4c94423383c56a7a97fe23c73d8c67d3f989ea09022 |
| Master_MuJoCo\calibration\phase3bs_physical_sensitivity\phase3bs_shared_direction_safety_validation.json | 13245 | 725e2fbe5f5be7e61a90e02822999050d0e2c4dd589a7c64a45f2f795ba194ed |
| Master_MuJoCo\calibration\phase3bv_physical_direction_validation\process_phase3bv_capture.py | 13506 | ed95c0c1eb937a8635ebf4f022de23234f5a6518b65eed86e46494debd5c6af9 |
| Master_MuJoCo\calibration\phase3bv_physical_direction_validation\run_phase3bv_replays.py | 6086 | ca3765d62a8faf93c795bb7dd34bcdf1def786e0cba4193c5ec1545231f011ec |
| Master_MuJoCo\calibration\phase3bv_physical_direction_validation\analyze_phase3bv.py | 23745 | d1da5f33e1122868a9d6252256982ef6cef65b246b30d1987b2396ff968ae3e3 |
| Master_MuJoCo\calibration\phase3bv_physical_direction_validation\finalize_phase3bv_blind_validation.py | 24975 | 9ecb63ffb52c48ce18a1fde14e2bcbf06254b280b9b20f89904cb828465976a9 |
| Master_MuJoCo\calibration\phase3bv_physical_direction_validation\README.md | 2135 | 954a37309cc05641ebbba12fecd146cec4edda3b8c50331456d74110284dd906 |
| Master_MuJoCo\calibration\phase3bv_physical_direction_validation\history\phase3bv_offline_preparation_gate_20260827.md | 719 | c649f721e70caf383152a4e21a5c07310339f91afcb668062e690c531ed569ab |
| Master_MuJoCo\calibration\phase3bv_physical_direction_validation\capture\phase3bv_clap_001\raw_serialized_evidence.txt | 8597267 | 37a56c53c0d9c769eb90e2ef495269826a49c8614f768769cd6198f53b19f513 |
| Master_MuJoCo\calibration\phase3bv_physical_direction_validation\capture\phase3bv_clap_001\recorder_status.txt | 91 | 1879dd5d82d8dbf11618cfe9c2e65f5c678222d1bd78a6739539aceceff70fe9 |
| Master_MuJoCo\calibration\phase3bv_physical_direction_validation\capture\phase3bv_clap_001\operator_safety_confirmation.txt | 175 | 932f145694b6104589ec1cfcdcf87fc7a65972ad9342bdfbf406fc8c4fdbb82d |
| Master_MuJoCo\calibration\phase3bv_physical_direction_validation\phase3bv_capture_metadata.json | 3861 | 937c0eb295d1a7aa20b94f583b041b9c5819a71315bef9dc61b0ef4306c7fed4 |
| Master_MuJoCo\calibration\phase3bv_physical_direction_validation\phase3bv_independence.json | 1455 | ea32ccc5fb07a34a600fe788b2758d2f65fe09c364e018f2883cada2d07295c3 |
| Master_MuJoCo\calibration\phase3bv_physical_direction_validation\phase3bv_joint_metrics.csv | 8147 | ae43158105293c2da532dc297cd11d44344670c731966bb0d9daf4d6a87c2f2c |
| Master_MuJoCo\calibration\phase3bv_physical_direction_validation\phase3bv_measured_reference.csv | 3470463 | 97a617884b6f64e9de704b7c4e739306cec733fa3e83c23d1d650e9a3a686495 |
| Master_MuJoCo\calibration\phase3bv_physical_direction_validation\phase3bv_aligned_joint_data.csv | 2080575 | 6e3f622065704156944b6f7bf6d98967df68c3de0792a7a64e94c709b0c5d99f |
| Master_MuJoCo\calibration\phase3bv_physical_direction_validation\phase3bv_aligned_imu_data.csv | 365176 | c5db3a17000b8ebc0db6a5c25656211b4dc944cc568849b6ce862175236d9db3 |
| Master_MuJoCo\calibration\phase3bv_physical_direction_validation\phase3bv_replay_execution.json | 1855 | 4bb0c6cdfa87c33d67b89f9c291b7854446924bb1a8877456d67ab0257e57b42 |
| work\run_x2_phase3bv_clap_capture_readonly.ps1 | 6151 | 09e0eea2723120710c40a0461e633af2123e7d77c3a1efcbcf0cd2dca198d306 |
| work\x2_phase2d_heart_capture_readonly.sh | 9392 | fc3d6947628cf0b9a6dcee38d65cc6197199a543caf6d30ca927b854de437a1d |
| work\phase2c_agentech01_code_discovery_readonly.txt | 20492828 | 15867c3feefec5f7ad061940c584c865d3f0e96b8c834f9e9b0626d37eaf967f |

`DYNAMICS_CALIBRATION_READY = NO`
