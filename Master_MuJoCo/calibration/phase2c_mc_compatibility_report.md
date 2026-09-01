# Phase 2C — MC compatibility and ownership

Status: **STANDING PRESET COMPATIBLE; ARBITRARY SINGLE-JOINT PATH NOT FOUND**

## Standing `heart()`

| Question | Evidence-backed answer |
|---|---|
| Does native MC stay running? | **YES at code/process-ownership level.** The standing route contains no MC stop/restart. |
| Is the action accepted by MC? | **YES.** It calls `SetMcPresetMotion`. |
| Is balance active throughout? | MC owns the motion and `STAND_DEFAULT` is checked before/after. Exact internal in-flight balance-loop state is **UNKNOWN** from static code. |
| Direct HAL write? | **NO** in the standing route. |
| `SetMcInputSource` used? | **NO** in the standing route. |
| Mode switch used? | **NO** in the standing route. |
| Physically exercised? | Catalog marks heart/both as physically tested; test date/hardware revision is **UNKNOWN**. |

Primary classification: **A. HIGH_LEVEL_MOTION**.

## Competition risks

- A per-instance `_motion_lock` prevents two Agentech gestures from the same
  process.
- `interrupt=False` asks MC not to interrupt an existing preset.
- The coBridge contract permits one logged-in client; the wrapper releases its
  submission connection and reconnects only to poll the exact task.
- The wrapper does not acquire an MC input source and does not prove global
  exclusivity against other applications. Cross-process preset arbitration and
  another high-level client remain **UNKNOWN**.
- It does not compete with `mc_ros2_node` on the direct arm HAL topic because it
  creates no HAL publisher.

## Direct-HAL route found

The powered seated controller:

1. subscribes to all joint state groups and chest/torso IMUs;
2. creates publishers for all joint command groups;
3. captures measured state and arms a persistent safety owner;
4. explicitly stops native MC;
5. publishes direct HAL commands at a 5 ms period.

Classification: **D. DIRECT_HAL**. It necessarily changes ownership and is
**DO_NOT_USE** for a Phase 2B-2 test whose requirement is to keep native MC
running.

## Decision

- Native standing heart can run without stopping MC.
- No reviewed production interface permits arbitrary J2/J7 ±1°/±2° motion
  while preserving native MC ownership. That proposed test remains **NO-GO**.
