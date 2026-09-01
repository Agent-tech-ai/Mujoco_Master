# Arm control coordinate evidence

## Evidence scope

The values below were supplied by the operator as on-site soft-engineer data. They are recorded as `FIELD_TEST_EVIDENCE`; they do not confirm a MuJoCo axis, hardware zero, encoder offset, or hardware-to-MuJoCo sign.

- Source: `FIELD_TEST_EVIDENCE: operator-supplied soft-engineer arm limits and Agentech.heart() endpoint observations, 2026-08-11`
- No robot movement was requested or performed by this calibration workflow.

| Joint | Side | field minimum (deg) | field maximum (deg) | observed endpoint (deg) | relation |
|---|---|---:|---:|---:|---|
| shoulder_pitch | left | -176.471 | 116.883 | — | NOT_ESTABLISHED |
| shoulder_pitch | right | -176.471 | 116.883 | — | NOT_ESTABLISHED |
| shoulder_roll | left | -3.495 | 171.486 | 126.042 | LEFT_RIGHT_MIRRORED |
| shoulder_roll | right | -171.486 | 3.495 | -126.042 | LEFT_RIGHT_MIRRORED |
| shoulder_yaw | left | -146.448 | 146.448 | — | NOT_ESTABLISHED |
| shoulder_yaw | right | -146.448 | 146.448 | — | NOT_ESTABLISHED |
| elbow | left | -134.965 | 0 | — | NOT_ESTABLISHED |
| elbow | right | -134.965 | 0 | — | NOT_ESTABLISHED |
| wrist_yaw | left | -146.448 | 146.448 | — | NOT_ESTABLISHED |
| wrist_yaw | right | -146.448 | 146.448 | — | NOT_ESTABLISHED |
| wrist_pitch | left | -31.971 | 31.971 | — | NOT_ESTABLISHED |
| wrist_pitch | right | -31.971 | 31.971 | — | NOT_ESTABLISHED |
| wrist_roll | left | -90.012 | 41.482 | -63.021 | LEFT_RIGHT_MIRRORED |
| wrist_roll | right | -41.482 | 90.012 | 63.021 | LEFT_RIGHT_MIRRORED |

## FIELD_TEST_EVIDENCE

- J2 shoulder-roll endpoint: left `+126.042°`, right `-126.042°`.
- J7 wrist-roll endpoint: left `-63.021°`, right `+63.021°`.
- Therefore the left/right hardware control-coordinate signs are mirrored for J2 and J7.

## UNKNOWN

- Whether each hardware coordinate has sign `+1` or `-1` relative to its MuJoCo joint.
- Hardware zero, encoder offset, and the physical pose at coordinate zero.
- Whether the field limits are firmware-enforced, soft limits, or a UI/control-layer limit.

## NEEDS_PHYSICAL_VERIFICATION

- Correlate passive/operational logs with an independently observed physical joint direction in a later authorized phase.
- Verify exact limit behavior without commanding a limit approach during Phase 2B-1.
