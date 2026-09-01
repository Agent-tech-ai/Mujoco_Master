# Initial pose report

- Base qpos: `[0.0, 0.0, 0.68, 1.0, 0.0, 0.0, 0.0]`; qvel is identically zero.
- Initial maximum target-minus-qpos error: **0°**.
- Lowest foot contact spheres are **5.05 mm above** the plane at initialization; both feet are symmetric within numeric precision.
- Initial contact count is 0, so the robot first drops approximately 5 mm before loading the feet.
- Pelvis orientation is identity. Knee is 0°, at its lower documented limit; ankles are 0° and not near their limits.
- Initial left/right lowest collision heights: 0.005050 / 0.005050 m.

Changing initial base height alone did not prevent the deterministic forward fall. It is a model-initialization issue, but not the primary instability cause.
