# Phase 3A-Y Controller Architecture

```text
real-time measurable state
  arm q / qdot / acceleration proxy
  left-right asymmetry + sagittal/lateral proxies
  pelvis pitch/roll + angular rates
  current joint/contact/actuator margins
        |
        v
disturbance estimator (0.10 rad/s activity deadband, 0.12 s filter)
        |
        +--> PITCH_RESPONSE_MODEL --> total pitch request + normalized allocation
        |
        +--> ROLL_RESPONSE_MODEL  --> total roll request  + normalized allocation
        |
        v
Phase 3A-X frozen constraint-aware safety layer
  channel priority -> constraint redistribution -> contact/limit/saturation/rate scaling
        |
        v
final target composition -> frozen arm/standing controller -> MuJoCo
```

The response model cannot bypass the safety layer. It contains no motion name, preset ID, physical parameter fit, effort input, or hardware calibration parameter.
