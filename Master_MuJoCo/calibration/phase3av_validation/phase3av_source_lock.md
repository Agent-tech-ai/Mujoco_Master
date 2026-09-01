# Phase 3A-V frozen source lock

Lock date: 2026-08-18  
Purpose: **BLIND VALIDATION — NO RETUNING**

## Frozen Phase 3A candidate

- candidate JSON SHA-256: `c351dea8131971e8c05e0a72bff350c8142fe913135d9d87b416080fb0dcd483`
- shoulder simulation gain scale: `8.0`
- wrist simulation gain scale: `8.0`
- balance gain scale: `0.7`
- standing-reference scale: `1.0`
- reference interpolation/rate: `linear / 50 Hz`
- controller update rate: `1000 Hz`
- simulation timestep: `0.001 s`
- global reference advance: **none**

## Required hashes

| Component | SHA-256 |
| --- | --- |
| `master_sim/controller.py` | `eae1b320d4ace99fe79bd123d70398ff6ac1446b2c33191ae8a68f5a31691c6e` |
| `master_sim/model.py` | `eb723f10257a3e91901d452f881647822ebb9930204035c311a3535001c51b16` |
| `ff_master_ultra.xml` | `89619295fcc372c57473224130865b2fe4f22e0741f72925fac243805f4353db` |
| `ff_master_ultra_x2_limits.xml` | `6d5940490d93f89929af8983a0de900c9e6c0351839163463ae0881d1b9399dd` |
| `scene_x2_fixed.xml` | `2dc116ce47d09a5105d01372a8456356b6e9881dee4c4947bc7c876757529a08` |
| `scene_x2_free.xml` | `88483553e15173d09d69f4fca32a466bb022d6dbb805f074ffa89447fc876d0b` |
| Phase 3A replay implementation | `5ef5c215f8dbc73101183f43ae89aec1edceebe83142c21a9b5464b3d422d52a` |
| Phase 3A-V replay entrypoint | `2543a0c92af428c8a0a3081f5261732fd3a5de937fe1d1385c9e004a8f2df092` |

The machine-readable lock is `phase3av_frozen_source_manifest.csv`. The recorder aborts before SSH if any locked local source hash differs. The replay entrypoint verifies the same manifest again before simulation.

## Prohibited changes during Phase 3A-V

No changes to bandwidth, balance gains, standing reference, interpolation, update rate, timing, controller gains, MJCF dynamics, mapping sign/zero/offset, or any physical parameter are permitted. Poor results remain validation evidence and are not optimized in this phase.

`NOT HARDWARE CALIBRATION`  
`DYNAMICS_CALIBRATION_READY = NO`
