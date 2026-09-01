# Phase 2C — recommended future test interface

Status: **NO APPROVED MC-NATIVE SINGLE-JOINT INTERFACE / NO-GO**

## RECOMMENDED_INTERFACE

None is currently approved for arbitrary J2/J7 deltas.

The required next interface should be a soft-engineer/vendor-qualified
**MC-native high-level arm action** exposed through coBridge/AimDK, with:

- native MC remaining the command owner;
- explicit joint name plus bounded relative delta;
- current-position readback before target calculation;
- full limit, velocity, acceleration, effort and timeout validation;
- controllable duration and exact return target;
- task ID, cancel/abort semantics and completion status;
- no project-created publisher on `/aima/hal/joint/arm/command`.

This interface was not found in the production source and must not be invented
from method names.

## INFERRED_CANDIDATE

An unmerged worktree named `standing-right-arm-teaching` contains a candidate
`teach_standing_action()` surface labelled `native_owner="native-mc"` and an
`adjust_elbow()` operation. However:

- it is outside the primary source tree;
- it targets teaching/right elbow rather than J2/J7;
- its replay method explicitly says live standing replay is not connected;
- no physical qualification for this calibration use was established.

It is therefore research evidence only, not a callable recommendation.

## ALTERNATIVE_INTERFACE

`SetMcPresetMotion(area=3, motion=1007)` is acceptable only for replaying the
already-defined full standing-heart preset after a separate GO review. It can
provide passive observations of known multi-joint motion, but it cannot isolate
one joint or request ±1°/±2°. It is not suitable for Phase 2B-2 sign/zero tests.

## DO_NOT_USE

- `/aima/hal/joint/arm/command` direct publisher;
- `seated_controller.py` or `arm_teach_runtime.py` for a standing MC-preserving
  test (both are direct-HAL ownership designs);
- `SetMcInputSource` as a substitute for direct-HAL ownership;
- `ff_sdk.motion.joint_stream`/`pose_arm` for X2: inspected SDK 0.1.0a2 exposes
  abstract capability surfaces but no verified X2 implementation for this use;
- the native heart preset to infer individual commanded joint angles.

## Required human confirmations before any future active test

E-stop tested; operator beside robot; clear workspace; stable posture; approved
MC-native interface; unique command source; numeric joint velocity and
acceleration limits; position reserve; effort protection; abort command;
communications-loss behavior; and verified post-test ownership/state. Any
UNKNOWN keeps the decision at NO-GO.
