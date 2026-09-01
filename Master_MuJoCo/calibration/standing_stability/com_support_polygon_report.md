# CoM and support-polygon report

- Total modeled mass: **43.474797 kg**.
- Whole-body CoM XYZ: **[0.005621692662336757, 0.00025176609100720427, 0.7124937672722609] m**.
- Left support polygon XY: `[[-0.08441000000000001, 0.11215008900000001], [-0.06941, 0.087150089], [-0.03941, 0.077150089], [0.05059, 0.077150089], [0.09059, 0.087150089], [0.11959, 0.137150089], [0.09059, 0.18715008900000002], [0.05059, 0.197150089], [-0.03941, 0.197150089], [-0.06941, 0.18715008900000002], [-0.08441000000000001, 0.162150089]]` m.
- Right support polygon XY: `[[-0.08441000000000001, -0.162150089], [-0.06941, -0.18715008900000002], [-0.03941, -0.197150089], [0.05059, -0.197150089], [0.09059, -0.18715008900000002], [0.11959, -0.137150089], [0.09059, -0.087150089], [0.05059, -0.077150089], [-0.03941, -0.077150089], [-0.06941, -0.087150089], [-0.08441000000000001, -0.11215008900000001]]` m.
- Combined support polygon XY: `[[-0.08441000000000001, -0.162150089], [-0.06941, -0.18715008900000002], [-0.03941, -0.197150089], [0.05059, -0.197150089], [0.09059, -0.18715008900000002], [0.11959, -0.137150089], [0.11959, 0.137150089], [0.09059, 0.18715008900000002], [0.05059, 0.197150089], [-0.03941, 0.197150089], [-0.06941, 0.18715008900000002], [-0.08441000000000001, 0.162150089]]` m.
- CoM projection is inside: **True**.
- Boundary margins: rear X 0.090032 m, front X 0.113968 m, right Y 0.197402 m, left Y 0.196898 m.
- Minimum combined-polygon boundary distance: **0.090032 m**.

The CoM starts inside the combined convex hull with substantial margin. Initial CoM location is ruled out as the primary cause. Method: convex hull of foot collision-sphere centers whose surfaces are within 3 mm of the lowest sole surface. This is a collision-center approximation, not a pressure/contact-patch measurement.
