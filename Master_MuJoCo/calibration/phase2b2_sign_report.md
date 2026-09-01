# Phase 2B-2 sign report

Status: **NOT TESTED — INSUFFICIENT_EVIDENCE**

| Joint | Active motion performed | Phase 2B-2 classification | Mapping change |
|---|---|---|---|
| left J2 shoulder roll | No | INSUFFICIENT_EVIDENCE | None; sign remains UNKNOWN |
| right J2 shoulder roll | No | INSUFFICIENT_EVIDENCE | None; sign remains UNKNOWN |
| left J7 wrist roll | No | INSUFFICIENT_EVIDENCE | None; sign remains UNKNOWN |
| right J7 wrist roll | No | INSUFFICIENT_EVIDENCE | None; sign remains UNKNOWN |

The earlier operator-supplied J2/J7 left/right mirrored-coordinate observation remains `FIELD_TEST_EVIDENCE`. It is not promoted to a confirmed hardware↔MuJoCo sign because no controlled physical move was performed in this phase.

Reason for stopping: native MC was actively publishing the full arm HAL command; the supported ownership procedure, numeric safety thresholds, and physical operator confirmations were incomplete. The requested symmetric J2 ±2° sequence also approached the supplied field limits too closely at the observed pose.

