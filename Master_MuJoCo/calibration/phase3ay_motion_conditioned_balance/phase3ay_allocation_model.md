# Phase 3A-Y Allocation Model

```text
measured arm/base state
  -> filtered motion-activity estimator
  -> instantaneous left/right asymmetry schedule
  -> independent total pitch / total roll request
  -> signed-normalized desired joint distribution
  -> frozen Phase 3A-X priority + constraint redistribution
  -> contact / joint-limit / saturation / slew / hard target envelope
  -> final simulation joint target/torque addition
```

No branch checks `heart`, `wave`, preset ID, or dataset name. At zero motion activity, the exact Phase 3A-X allocation is recovered. Every timestep is auditable in the redistribution CSV.
