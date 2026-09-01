# Fixed-base versus free-base decomposition

1. Fixed base + original controller: stable for 10 s, max joint error 0.012° and peak saturation 0.066.
2. Free base + identical target/controller: deterministic forward fall at 1.606 s.
3. Foot/friction/timestep single-factor changes do not remove the fall.
4. Free base + explicit simulated pelvis-attitude feedback: stands 10 s.

Decision: the primary standing failure is **missing base-attitude/whole-body balance control**. Foot initialization/contact discretization is secondary. CoM-outside-support, gross mass asymmetry and timestep instability are ruled out as primary causes.
