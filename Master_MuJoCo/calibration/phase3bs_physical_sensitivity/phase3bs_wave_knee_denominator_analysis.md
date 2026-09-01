# Wave right-knee denominator analysis

## Measured comparison

| experiment_id | dataset | joint_name | real_excursion_rad | sim_excursion_rad | excursion_ratio | absolute_excursion_error_rad | position_rmse_rad | velocity_rmse_rad_s | onset_delta_s | peak_timing_delta_s | xcorr_lag_s | recovery_delta_s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| bs_baseline | wave | right_knee_joint | 0.0101536 | 0.082092 | 8.08499 | 0.0719384 | 0.0447729 | 0.0632348 | 0.46 | 0.28 | -1 |  |

- Real excursion denominator: **0.01015364 rad**.
- Simulation excursion: **0.08209202 rad**.
- Absolute excursion difference: **0.07193838 rad**.
- Observed ratio: **8.084986×**.
- Median real Wave pitch-chain excursion: **0.01706553 rad**.
- Small-denominator amplification relative to that median: **1.681×**.
- Replacing the knee denominator with that median would leave a **4.810×** response, so the mismatch is not only a ratio artifact.
- The denominator accounts for **46.2%** of the excess-over-one ratio under this explicit median-response counterfactual; in log-ratio terms it accounts for **24.8%**. These are sensitivity summaries, not causal parameter identification.

`RATIO_AMPLIFICATION_BY_SMALL_REAL_EXCURSION = YES`.

The cross-correlation lag reaches the -1.0 s search boundary and is therefore boundary-censored; it is reported, but not interpreted as a precise lag estimate.
