# Phase 3C Mass Evidence

## Decision

`CURRENT_MJCF_LOWER_LIMB_MASS_UNDERESTIMATED = INSUFFICIENT_EVIDENCE`

`bs_mass_lower_plus08` remains a `CROSS_MOTION_VALIDATED_PHYSICAL_SENSITIVITY_DIRECTION`. It is not an identified X2 mass distribution and does not establish a physical `+8%` correction.

## Current model inventory

The resolved `ff_master_ultra.xml` contains 32 inertial bodies:

| Metric | Current compiled MJCF |
|---|---:|
| Total mass | 43.4747966593 kg |
| Lower-limb mass | 16.646758 kg |
| Non-lower mass | 26.8280386593 kg |
| Lower/non-lower ratio | 0.620498509467 |
| Left named-body mass | 12.268144 kg |
| Right named-body mass | 12.250455 kg |
| Left-right difference | 0.017689 kg |

The exact per-body mass, CoM and full inertia tensor are in `phase3c_current_inertial_provenance.csv`.

## Confirmed source-lineage facts

1. Thirty-one bodies have explicit MJCF inertials that numerically match the same-bundle `ff_master_ultra.urdf` within conversion precision. The representation changed from URDF full tensors to MJCF principal inertia plus quaternion, but the values are otherwise consistent.
2. The current MJCF pelvis has no explicit `<inertial>`. MuJoCo therefore derives its inertial from the pelvis collision mesh and default geometry density at compile time.
3. Compiled pelvis mass is 5.03181065927 kg; the same-bundle URDF pelvis mass is 3.523487 kg. The model-conversion delta is +1.50832365927 kg, or +42.8077% relative to the URDF value.
4. This pelvis discrepancy raises the current non-lower mass without changing the lower-limb mass. Consequently, the current MJCF lower/non-lower ratio is 0.620499, versus 0.657462 in the same-bundle Ultra URDF.
5. `ff_master_ultra_x2_limits.xml` resolves to the same mass aggregates as the current MJCF and is not an independent physical source.
6. Fist and hand URDF variants retain the same lower-limb mass and change upper mass through end-effector variants. They are members of the same asset family, not independent X2 measurements.

## AimDK X2 SDK evidence

The locked Phase 2H read-only capture includes excerpts from AimDK X2 SDK v1.0.0 `x2_rl_deploy_mujoco/.../model_info/x2.xml` (SHA-256 `3ff43f...d91a3`). The excerpts show:

- the same pelvis mesh-based auto-inertia structure, with no explicit pelvis inertial in the captured root-body section;
- matching inertial values for captured sample bodies including left hip pitch, waist pitch, torso, left shoulder pitch and left shoulder roll;
- the same X2 link/mesh naming lineage.

This is Level A manufacturer-SDK source-lineage evidence. It confirms that the baseline inertials are not an arbitrary local invention. It does not provide metrology provenance, uncertainty, payload configuration, or an independent real-robot mass measurement. It therefore cannot prove that the real X2 lower limbs are under-massed, nor can it validate the `+8%` magnitude.

## Source comparison

| Source | Total kg | Lower kg | Non-lower kg | Lower/non-lower | Independence for real-X2 mass closure |
|---|---:|---:|---:|---:|---|
| Current compiled MJCF | 43.474797 | 16.646758 | 26.828039 | 0.620499 | Baseline only |
| X2-limits MJCF | 43.474797 | 16.646758 | 26.828039 | 0.620499 | Derived limit variant |
| Ultra URDF | 41.966521 | 16.646766 | 25.319755 | 0.657462 | Same asset family |
| Simple-collision Ultra URDF | 41.966521 | 16.646766 | 25.319755 | 0.657462 | Same asset family |
| Fist URDF | 40.965056 | 16.646766 | 24.318290 | 0.684537 | Same asset family/end-effector variant |
| Hand URDF | 40.472410 | 16.646766 | 23.825644 | 0.698691 | Same asset family/end-effector variant |

## Interpretation of `bs_mass_lower_plus08`

The Phase 3B-S perturbation preserves total mass while applying a relative lower-limb change:

| Metric | Baseline | `bs_mass_lower_plus08` |
|---|---:|---:|
| Lower mass | 16.646758 kg | 17.978499 kg |
| Non-lower mass | 26.828039 kg | 25.496298 kg |
| Lower/non-lower ratio | 0.620499 | 0.705142 |
| Non-lower scaling | 1.000000 | 0.950360 |

This perturbation is not equivalent to a scale measurement, CAD update, or manufacturer-specified per-link correction. It simultaneously redistributes mass between broad body groups. The observed response improvement can support only the direction of a position-space sensitivity.

## Mass, CoM and inertia are separate blockers

- **Mass:** no independent A/B/C per-link or grouped mass measurement closes the magnitude.
- **CoM:** no independent A/B/C X2 link CoM source was found. The sensitivity experiment did not identify CoM.
- **Inertia:** no independent A/B/C X2 inertia tensor source was found. The experiment intentionally did not vary inertia.
- **Pelvis conversion discrepancy:** actionable as a model-provenance defect candidate, but not sufficient evidence that either 3.523487 kg or 5.031811 kg is the true physical pelvis mass.

## Evidence required to close

At least one independently traceable source is needed: manufacturer/CAD per-link mass properties for the exact X2 Ultra configuration, a deployed robot description whose inertials are documented as physical values, or a controlled physical measurement protocol that separates assemblies and payloads. The source must specify configuration, coordinate frame, units, CoM, inertia convention and uncertainty.

`MASS_PARAMETER_EVIDENCE_READY = NO`

`INERTIA_PARAMETER_EVIDENCE_READY = NO`

