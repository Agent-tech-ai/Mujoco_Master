# Solver, timestep and sensitivity report

Baseline: timestep 0.001 s, Euler integrator enum 0, Newton solver enum 2, 100 iterations, tolerance 1e-8.

Reducing timestep to 0.0005 s produced essentially the same forward fall. Friction scaling 0.5×/1.0×/1.5×, position-gain scaling 0.5×/1.0×/1.5×, and damping scaling 0.5×/1.0×/1.5× all failed to keep the original controller standing. The instability is therefore not primarily timestep, solver, friction magnitude, or modest joint-PD tuning.

Experiment metrics are recorded in `standing_stability_experiments.csv`.
