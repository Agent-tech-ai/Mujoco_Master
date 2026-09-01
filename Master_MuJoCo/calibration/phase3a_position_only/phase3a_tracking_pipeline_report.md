# Phase 3A Tracking Pipeline Decomposition

Status: **COMPLETE**  
Scope: position/velocity response only. **NOT HARDWARE CALIBRATION.**

No global time advance was used. The Phase 2G 0.30 s advance remains diagnostic-only and is absent from every Phase 3A candidate.

## One-factor fixed-base experiments

| experiment | single changed factor | mean RMSE rad | mean MAE rad | median lag s | max ctrl fraction | persistent saturation | classification |
| --- | --- | --- | --- | --- | --- | --- | --- |
| fixed_baseline | none | 0.27055 | 0.18958 | 0.310 | 0.273 | 0.000 | DIAGNOSTIC_ONLY |
| fixed_interp_zoh | reference interpolation | 0.27815 | 0.19489 | 0.330 | 0.251 | 0.000 | DIAGNOSTIC_ONLY |
| fixed_interp_pchip | reference interpolation | 0.27055 | 0.18958 | 0.310 | 0.274 | 0.000 | DIAGNOSTIC_ONLY |
| fixed_reference_rate_25hz | reference sampling rate | 0.27045 | 0.18957 | 0.310 | 0.274 | 0.000 | DIAGNOSTIC_ONLY |
| fixed_reference_rate_100hz | reference sampling rate | 0.27055 | 0.18958 | 0.310 | 0.273 | 0.000 | DIAGNOSTIC_ONLY |
| fixed_controller_rate_500hz | controller update rate | 0.27054 | 0.18958 | 0.310 | 0.274 | 0.000 | DIAGNOSTIC_ONLY |
| fixed_controller_rate_200hz | controller update rate | 0.27053 | 0.18957 | 0.310 | 0.274 | 0.000 | DIAGNOSTIC_ONLY |
| fixed_controller_rate_100hz | controller update rate | 0.31507 | 0.23435 | 0.310 | 1.000 | 0.221 | DIAGNOSTIC_ONLY |
| fixed_timestep_0005 | simulation timestep | 0.27054 | 0.18951 | 0.310 | 0.274 | 0.000 | DIAGNOSTIC_ONLY |
| fixed_velocity_limit_5 | simulation target velocity limit | 0.27055 | 0.18958 | 0.310 | 0.273 | 0.000 | DIAGNOSTIC_ONLY |
| fixed_velocity_limit_2 | simulation target velocity limit | 0.29182 | 0.20411 | 0.350 | 0.273 | 0.000 | DIAGNOSTIC_ONLY |
| fixed_arm_pd_candidate | per-joint simulation arm PD bandwidth | 0.11101 | 0.06925 | 0.130 | 0.293 | 0.000 | ACCEPTED_SIM_CONTROLLER_ALIGNMENT |

## Findings by pipeline factor

- **A — interpolation:** 50 Hz linear and PCHIP are effectively equivalent. ZOH adds error and increases median lag from 0.31 s to 0.33 s.
- **B — reference rate:** 25, 50, and 100 Hz are nearly identical at the baseline bandwidth. Reference sampling is not the dominant 0.24/0.38 s delay source.
- **C — controller update rate:** 500 and 200 Hz match 1000 Hz closely. At 100 Hz the controller develops persistent saturation (~22.1% of samples) and RMSE regresses, so it is rejected.
- **D — timestep:** 0.5 ms and 1.0 ms are effectively equivalent. The integrator timestep is not the dominant delay source.
- **E — PD/controller bandwidth:** independent shoulder and wrist scans monotonically reduce lag/RMSE through the tested 8x boundary. The 8x/8x combination is accepted as a bounded simulation candidate, but the boundary optimum is a reason not to interpret it as a unique physical parameter estimate.
- **F — actuator saturation:** no actuator limit was changed. Baseline peak command fraction is observationally below 0.28 with zero persistent saturation; saturation does not explain the original lag. The 100 Hz controller-rate failure is the counterexample and was rejected.
- **G — velocity limiting:** a 5 rad/s target limiter is inactive/equivalent to baseline; 2 rad/s increases RMSE and lag and is rejected.

Conclusion: the dominant removable delay is the **simulation arm controller bandwidth**, not free-base coupling, reference sample rate, timestep, or an admissible global clock shift.
