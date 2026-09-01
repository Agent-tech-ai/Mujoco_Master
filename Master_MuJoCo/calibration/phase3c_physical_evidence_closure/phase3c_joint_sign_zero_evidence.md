# Phase 3C Joint Sign / Zero Evidence

## Decision

`PHYSICAL_SIGN_READY = NO`

`PHYSICAL_ZERO_READY = NO`

No `joint_mapping.csv` sign, hardware zero, MuJoCo zero, or encoder offset was changed.

## What is confirmed

- Live `JointStateArray` group/index/name associations were passively observed for arm, leg, waist and head channels.
- Official joint names and limit intervals are available.
- Heart preset evidence shows mirrored left/right control-coordinate endpoints for shoulder roll and wrist roll.
- Current MJCF contains mirrored-side axis/range choices for selected joints.

These facts establish names, indices, ranges and candidate relationships. They do not establish the physical positive rotation of an isolated joint or the encoder datum corresponding to a defined physical zero pose.

## Target-joint audit

| Joint(s) | Physical sign | Physical zero | Best evidence | Why not closed |
|---|---|---|---|---|
| left/right shoulder roll | `UNKNOWN` | `UNKNOWN` | Official limits + live names + mirrored multi-joint Heart endpoint (`FIELD_TEST_EVIDENCE`) | Heart is a coupled preset; no isolated physical-axis observation or documented frame convention ties measured sign to the link motion. |
| left/right wrist yaw | `UNKNOWN` | `UNKNOWN` | Official symmetric limits + live names + replay trajectories | Symmetric limits and trajectory correlation cannot identify physical sign or zero datum. |
| left/right wrist roll | `UNKNOWN` | `UNKNOWN` | Official limits + live names + mirrored multi-joint Heart endpoint (`FIELD_TEST_EVIDENCE`) | Mirroring is a control-coordinate relation, not a physical-axis proof; no isolated link observation. |
| left/right hip pitch | `UNKNOWN` | `UNKNOWN` | Official symmetric limits + live names + standing/preset response | Symmetric range and balance motion cannot disambiguate sign; controller equilibrium is not hardware zero. |
| left/right knee | `UNKNOWN` | `UNKNOWN` | Official one-sided range + live names + standing flexion response | A `0..138°` range suggests a convention but does not prove encoder polarity, physical datum, or MuJoCo mapping sign. |
| left/right ankle pitch | `UNKNOWN` | `UNKNOWN` | Official asymmetric limits + live names; MJCF interval is reversed in magnitude | This is a `SIGN_MISMATCH_CANDIDATE`, not proof. No isolated physical-axis observation or manufacturer frame mapping was found. |
| waist pitch | `UNKNOWN` | `UNKNOWN` | Official symmetric limits + live name + balance response | Symmetric range and coupled balance motion do not identify sign or zero. |

No target is upgraded to `LIKELY` because Phase 3C found no new independent A/B/C evidence beyond the already-audited Phase 2H material. Re-labeling an existing simulation convention as likely hardware truth would overstate the evidence.

## Evidence needed

For each fitted joint, close sign using either an official/deployed axis-frame convention or a read-only observation of a known, isolated physical movement with an independently labeled direction. Close zero using a manufacturer-defined home/calibration fixture pose or an encoder-datum procedure that provides the measured position and offset for that physical pose. Multi-joint preset endpoints alone are insufficient.

