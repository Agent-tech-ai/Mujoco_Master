# Foot collision report

Each ankle-roll link uses one non-colliding visual mesh plus 12 tiny collision spheres; 11 are sole/edge candidates and one is above the sole. Sphere radius is 0.005 m. Default friction is `(1.0, 0.005, 0.0001)` inherited from MuJoCo defaults, while the plane declares `(1.0, 0.01, 0.001)`.

The left/right layouts are mirrored. The combined center envelope is approximately X (-0.08441, 0.11959) m and Y (-0.19715009, 0.19715009) m. At initialization, the lowest sphere surfaces are 5.05 mm above the plane. This creates a small landing transient and discrete contact chatter.

A single-factor box-foot experiment and an initial-height experiment both still fell forward. Therefore foot geometry is a **SECONDARY_CONTRIBUTOR**, not the primary cause. No foot geometry or friction was changed in the accepted cleanup.
