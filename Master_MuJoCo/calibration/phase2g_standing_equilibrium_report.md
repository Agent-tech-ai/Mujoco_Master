# Phase 2G standing equilibrium mismatch

The knee offsets are classified as **CONTROLLER_EQUILIBRIUM_MISMATCH**, not hardware zero. In the baseline, each controller target is equal (within interpolation noise) to the real pre-roll reference, while the settled simulated joint differs from that target.

| joint | real initial | sim initial | controller target | sim settled | settled-target | settled-real | class |
| --- | --- | --- | --- | --- | --- | --- | --- |
| left_knee_joint | 0.50362 | 0.50364 | 0.50362 | 0.62410 | 0.12049 | 0.12049 | CONTROLLER_EQUILIBRIUM_MISMATCH |
| right_knee_joint | 0.48034 | 0.48046 | 0.48042 | 0.60045 | 0.12004 | 0.12012 | CONTROLLER_EQUILIBRIUM_MISMATCH |
| right_ankle_pitch_joint | -0.29500 | -0.29497 | -0.29500 | -0.35460 | -0.05960 | -0.05960 | CONTROLLER_EQUILIBRIUM_MISMATCH |
| left_ankle_pitch_joint | -0.27276 | -0.27276 | -0.27276 | -0.30942 | -0.03666 | -0.03666 | CONTROLLER_EQUILIBRIUM_MISMATCH |
| left_hip_pitch_joint | -0.30075 | -0.30079 | -0.30075 | -0.33525 | -0.03449 | -0.03449 | CONTROLLER_EQUILIBRIUM_MISMATCH |
| left_ankle_roll_joint | -0.02982 | -0.02982 | -0.02982 | -0.01334 | 0.01648 | 0.01648 | CONTROLLER_EQUILIBRIUM_MISMATCH |
| right_hip_pitch_joint | -0.25684 | -0.25708 | -0.25703 | -0.26581 | -0.00877 | -0.00896 | WITHIN_0P01_RAD |
| right_ankle_roll_joint | 0.01563 | 0.01561 | 0.01563 | 0.02032 | 0.00469 | 0.00469 | WITHIN_0P01_RAD |
| right_hip_roll_joint | -0.01774 | -0.01773 | -0.01774 | -0.02126 | -0.00353 | -0.00353 | WITHIN_0P01_RAD |
| left_hip_roll_joint | 0.01371 | 0.01372 | 0.01371 | 0.01609 | 0.00238 | 0.00238 | WITHIN_0P01_RAD |
| left_hip_yaw_joint | -0.00029 | -0.00029 | -0.00029 | -0.00264 | -0.00235 | -0.00235 | WITHIN_0P01_RAD |
| right_hip_yaw_joint | -0.01352 | -0.01352 | -0.01352 | -0.01469 | -0.00118 | -0.00118 | WITHIN_0P01_RAD |

For left/right knee, baseline settled-minus-real is 0.12049 / 0.12012 rad. A simulation-only one-shot standing-target compensation reduced the absolute knee residuals to -0.01467 / -0.00999 rad, but did not eliminate settled-minus-target behavior and worsened some ankle equilibria. It is therefore only `SIM_CONTROLLER_ALIGNMENT_CANDIDATE` and is not adopted as a calibrated state.

`POSSIBLE_ZERO_MISMATCH` remains unresolved for every joint. Real encoder zero cannot be inferred from agreement or disagreement with an unverified identity replay mapping.
