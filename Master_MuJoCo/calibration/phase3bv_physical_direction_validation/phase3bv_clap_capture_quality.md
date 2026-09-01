# Phase 3B-V Clap capture quality gate

`CLAP_CAPTURE_VALID = YES`

- source: `READ_ONLY_REAL_ROBOT_STATE`; recorder sent command: `NO`
- preset execution: `EXTERNAL_OPERATOR / EXISTING_MC_COMPATIBLE_PATH`
- automatic motion window: `10.096456` to `15.539997` capture seconds
- duration: `5.443541 s`; pre/post: `9.849181 / 6.048464 s`
- completion marker missing; accepted only because the detected motion has >5 s intact pre/post data on all six streams
- MC: `{'status': 'EXPECTED_STANDING_MODE_CONFIRMED', 'messages_during_motion_window': 75, 'action_desc_values': ['STAND_DEFAULT'], 'input_source_values': ['app_proxy'], 'motion_status_values': ['', '3017'], 'player_state_values': ['0', '1', '2']}`
- independence: `SUFFICIENTLY_INDEPENDENT_FROM_HEART_AND_WAVE`

## Existing processor evidence

# Phase 3B-V capture quality

`PHASE3BV_VALIDATION_DATA_READY = YES`

- capture: `phase3bv_clap_001`
- selected candidate: `clap`, native MC preset 3017 / area 11
- recorder complete/data-window accepted: `False` / `True`
- detected motion: `10.096 s -> 15.540 s`; threshold `0.0500 rad/s`
- pre-roll/post-roll: `9.849 / 6.048 s`
- stable joint names: `True`; issues: `none`
- source timestamps monotonic: `True`
- max receive gaps below 0.5 s: `True`
- MC expected standing mode: `{'status': 'EXPECTED_STANDING_MODE_CONFIRMED', 'messages_during_motion_window': 75, 'action_desc_values': ['STAND_DEFAULT'], 'input_source_values': ['app_proxy'], 'motion_status_values': ['', '3017'], 'player_state_values': ['0', '1', '2']}`
- motion invocation: operator only; recorder sent no command
- reported effort: excluded from all validation references and metrics

| topic | samples | mean Hz | max gap s | source reversals | frame IDs |
| --- | --- | --- | --- | --- | --- |
| /aima/hal/joint/arm/state | 1018 | 47.51 | 0.0327 | 0 | {'x2_arm': 1018} |
| /aima/hal/joint/head/state | 984 | 45.92 | 0.0363 | 0 | {'x2_head': 984} |
| /aima/hal/joint/leg/state | 1018 | 47.48 | 0.0305 | 0 | {'x2_leg': 1018} |
| /aima/hal/joint/waist/state | 1018 | 47.48 | 0.0310 | 0 | {'x2_waist': 1018} |
| /aima/hal/imu/chest/state | 1015 | 47.42 | 0.0326 | 0 | {'base_link': 1015} |
| /aima/hal/imu/torso/state | 1012 | 47.37 | 0.0337 | 0 | {'base_link': 1012} |
