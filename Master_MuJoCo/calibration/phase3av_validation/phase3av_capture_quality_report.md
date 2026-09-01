# Phase 3A-V capture quality report

`PHASE3AV_VALIDATION_DATA_READY = YES`

- capture: `phase3av_wave_right_001`
- selected preset: `wave(right)`, native MC preset 1002 / area 2
- complete recorder footer: `False`
- acceptable complete data window/footer: `True`
- detected motion: `13.240 s -> 17.589 s`; velocity threshold `0.0500 rad/s`
- verified pre-roll/post-roll: `13.067 / 8.017 s`
- stable joint names: `True`; issues: `none`
- required source timestamps monotonic: `True`
- all required max receive gaps below 0.5 s: `True`
- MC expected standing mode: `{'status': 'EXPECTED_STANDING_MODE_CONFIRMED', 'messages_during_motion_window': 63, 'action_desc_values': ['STAND_DEFAULT'], 'input_source_values': ['app_proxy'], 'motion_status_values': ['', '1002'], 'player_state_values': ['0', '1', '2']}`
- motion invocation: operator/soft-engineer only; recorder sent no command

| topic | samples | mean Hz | max gap s | source reversals | frame IDs |
| --- | --- | --- | --- | --- | --- |
| /aima/hal/joint/arm/state | 1203 | 47.23 | 0.0510 | 0 | {'x2_arm': 1203} |
| /aima/hal/joint/head/state | 1172 | 46.00 | 0.0424 | 0 | {'x2_head': 1172} |
| /aima/hal/joint/leg/state | 1201 | 47.14 | 0.0506 | 0 | {'x2_leg': 1201} |
| /aima/hal/joint/waist/state | 1203 | 47.22 | 0.0509 | 0 | {'x2_waist': 1203} |
| /aima/hal/imu/chest/state | 1203 | 47.25 | 0.0364 | 0 | {'base_link': 1203} |
| /aima/hal/imu/torso/state | 1203 | 47.26 | 0.0416 | 0 | {'base_link': 1203} |
