# Phase 2G JointState effort semantics

Final classification: **UNKNOWN**.

| Evidence layer | Finding | What it does not prove |
|---|---|---|
| `aimdk_msgs/msg/JointState` schema | `effort` is `double` | origin or estimator |
| AimDK documentation already captured | labeled torque, unit N·m | measured vs estimated vs commanded vs current-derived |
| live graph | state publisher is EtherCAT HAL (`/hal_ethercat_x21436`) | assignment source |
| FF SDK/application source | consumes the value as reported/measured effort and applies safety thresholds | HAL meaning; downstream naming is not provenance |
| Phase 2D response | nonzero, time-varying output correlated to motion for some joints | torque source or sign convention |

No inspected official documentation or publisher/HAL source shows the assignment to `JointState.effort`. Therefore it cannot be classified as `MEASURED_TORQUE`, `ESTIMATED_TORQUE`, `COMMANDED_TORQUE`, `CURRENT_DERIVED_TORQUE`, or `OTHER`. It remains excluded from torque calibration; `phase2f_effort_qualitative.csv` is response-shape evidence only.
