# Phase 2G hardware sign / zero verification plan

Status: **PLAN ONLY — NO MAPPING UPGRADE**.

## Existing evidence

- Live `JointState.name` values provide a name-level mapping.
- Historical heart motion provides dynamic left/right mirror evidence for J2/J7 and other moving arm joints, but it does not define the MuJoCo physical positive axis or encoder zero.
- `STAND_DEFAULT` is checked before/after preset 1007. Neither the inspected wrapper nor the captured MC state exposes a numeric joint specification for this pose.
- Heart endpoints are measured/API-preset outcomes, not documented mechanical calibration targets.

## Passive evidence sequence

1. Obtain manufacturer or deployed MC source specifying numeric joint targets for a named `home`, `neutral`, `calibration`, or `STAND_DEFAULT` pose, including coordinate convention and firmware applicability.
2. Hash and record that source; capture JointState while an operator has independently placed/confirmed the robot in exactly that already-supported pose. Codex sends no motion.
3. For each fitted joint, compare specified physical angle to measured position. A single nonzero known pose can provide an offset candidate; at least two distinct known poses or a physical direction observation are required to confirm sign and scale.
4. Confirm left/right convention separately; symmetry alone is not a global-axis definition.
5. Promote sign/zero only per joint with source plus physical-pose evidence. Keep encoder offset separate from an MC balance/posture bias.

## Required physical verification

An operator must identify the physical landmark/fixture for zero, confirm pose tolerance, E-stop and passive observation conditions, and verify the deployed firmware uses the cited coordinate definition. Until then, sign/zero remain UNKNOWN or FIELD_TEST_EVIDENCE only.
