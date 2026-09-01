# BalanceSafetyState definition

| field | simulation source |
| --- | --- |
| joint lower/upper margin | q versus unchanged MJCF range |
| velocity / tracking error | qvel and filtered target minus q |
| actuator margin | current ctrl fraction of unchanged ctrlrange |
| pelvis/hip distance | `mj_geomDistance` on collision geoms |
| foot state/slip | floor contacts and sole-body XY displacement |
| base roll/pitch | pelvis rotation matrix |
| CoM/support margin | subtree CoM versus sole-center support proxy |

The vector is computed read-only every simulation control timestep. Logs separately
record reference, standing offset, pitch/roll additions, contact avoidance,
limit/contact/saturation/rate scaling, allocation and final equivalent target.
