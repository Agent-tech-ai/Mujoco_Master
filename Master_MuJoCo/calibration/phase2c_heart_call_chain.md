# Phase 2C — `Agentech.heart()` call chain

Status: **STATIC CODE TRACE CONFIRMED**

## Public dispatch

`Agentech.use("master")` constructs `Master(**options)`. The metaclass then
forwards `Agentech.heart(...)` to that `Master` instance. `Master` defaults to
`dry_run=True`; live behavior requires an explicit `dry_run=False` selection.

With the normal `Agentech.heart(both)` call, posture defaults to `STAND`:

1. `Master.heart()` resolves `Hand.BOTH` and `Posture.STAND`.
2. It calls `_gesture("heart", Hand.BOTH)`.
3. The catalog resolves heart to `motion_id=1007`, `area=3`.
4. `_execute()` reads `GetMcAction` and refuses unless it is `STAND_DEFAULT`.
5. It sends `SetMcPresetMotion(area=3, motion=1007, interrupt=False)`.
6. It blocks/polls the returned task ID through `GetMcPresetMotionState`.
7. It reads `GetMcAction` again and requires `STAND_DEFAULT` after completion.

The wrapper uses a non-blocking local mutex, so a second gesture through the
same `Master` instance is rejected while one is active.

## Standing-heart motion details

| Item | Result |
|---|---|
| Command layer | Native AimDK/MC preset |
| API command | Area 3, motion 1007, `interrupt=False` |
| Joints involved | **UNKNOWN** to wrapper; defined inside preset 1007 |
| Joint targets | **UNKNOWN** to wrapper |
| Interpolation | **UNKNOWN** to wrapper |
| Duration | **UNKNOWN**; `motion_timeout=20 s` is only a client timeout |
| Velocity/acceleration limits | **UNKNOWN** to wrapper |
| Safety clamp | Pre/post `STAND_DEFAULT`, one local gesture mutex, task-state checks |
| Return-to-neutral | Preset-internal **UNKNOWN**; wrapper only confirms return to `STAND_DEFAULT` |
| Current-pose dependence | Preset-internal **UNKNOWN** |
| MC-mode dependence | Requires `STAND_DEFAULT` before sending |

## J2/J7 endpoint interpretation

The earlier field values were:

- left J2 `+126.042°`, right J2 `-126.042°`;
- left J7 `-63.021°`, right J7 `+63.021°`.

For the default standing call, these are **not transmitted API targets**: the
wrapper sends only area 3 and motion ID 1007. Therefore they may be treated as
FIELD_TEST_EVIDENCE/measured endpoint observations, not as proof of the
standing preset's source trajectory.

The current source also contains exactly these numbers in the *separate*
`SEATED_HEART_ENDPOINT_DEGREES` table. That table defines all 14 arm targets for
the direct-HAL seated controller, with 10 s outbound, 2 s hold, and 10 s return.
It must not be used to relabel standing-heart observations as commanded joint
targets unless logs prove the seated route was explicitly selected.

## Source evidence

- Primary source:
  `/mnt/c/Users/wesle/OneDrive/Documents/Agentech/agentech_sdk/agentech/robots/master/api.py`
- Catalog evidence from `actions/standing.py`: heart motion 1007, both-arm
  physical-test marker.
- Raw local capture:
  `../../work/phase2c_agentech01_code_discovery_readonly.txt`.
