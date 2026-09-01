# Phase 2C control-path discovery

Status: **READ-ONLY COMPLETE / NO MOTION EXECUTED**

## CONFIRMED environment

- SSH alias: `agentech01`; host name `agentech01`; user `wesle`.
- Login working directory: `/mnt/c/Users/wesle` inside Ubuntu 22.04.2 WSL2.
- ROS 2 and AimDK are not in the default WSL shell environment. No live ROS
  graph was queried from this host.
- Primary source tree found at
  `/mnt/c/Users/wesle/OneDrive/Documents/Agentech/agentech_sdk`.
- Staged source snapshots also exist below `/mnt/c/Users/wesle/.agentech/staging`.
  The primary-tree and `seated-heart-150-v106` Master API/controller hashes are
  different, so the exact revision deployed at the time of an earlier physical
  `heart()` test is **UNKNOWN**.

Evidence: `../../work/phase2c_agentech01_readonly.txt` and
`../../work/phase2c_agentech01_code_discovery_readonly.txt`. The second capture
ended with exit code 0, a completion marker, and zero remote writes, imports,
ROS calls, process changes, mode changes, or motion.

## CONFIRMED standing-heart route

```text
Agentech.use("master")
  -> Master(..., dry_run=True by default)
Agentech.heart(both)
  -> Master.heart(posture defaults to STAND)
  -> Master._gesture("heart", Hand.BOTH)
  -> STANDING_GESTURES["heart"]
       area=3, motion_id=1007, physically_tested=(BOTH,)
  -> Master._execute()
  -> GetMcAction; require STAND_DEFAULT
  -> CoBridgeClient.start_preset(area=3, motion=1007, interrupt=False)
  -> AimDK SetMcPresetMotion
  -> poll GetMcPresetMotionState(task_id)
  -> GetMcAction; require return to STAND_DEFAULT
```

The default local transport is `ws://127.0.0.1:4173/sdk` when the relay token
exists; the token file existed during discovery. With `relay_url=None`, the API
uses robot host `192.168.4.66`, port `21274`. The relay's internal forwarding
implementation was not fully traced, so that intermediate hop remains
**UNKNOWN** beyond the wrapper's `CoBridgeClient` contract.

## Classification

- Standing `Agentech.heart()`: **A. HIGH_LEVEL_MOTION**, specifically a native
  MC preset request through `SetMcPresetMotion`.
- Direct HAL: **NO** for the standing route.
- `SetMcInputSource`: **not called** by the standing route.
- Control-mode switch: **not called** by the standing route.
- Native MC stop/restart: **not called** by the standing route.

## Separate controller found — do not conflate

`seated_controller.py` is a different execution route. It creates
`JointCommandArray` publishers on `/aima/hal/joint/{arm,waist,head,leg}/command`
and explicitly calls `EmClient(...).stop_app('mc')`. It is therefore
**D. DIRECT_HAL** and is not evidence that standing `heart()` writes HAL.

This Phase 2C work did not invoke either route and did not modify any remote
source or configuration.

## UNKNOWN

- Exact deployed source revision for the earlier field observation.
- Relay implementation after `ws://127.0.0.1:4173/sdk`.
- MC's internal balance-loop state at every instant of preset 1007; code proves
  MC stays running and validates `STAND_DEFAULT` before/after, but static source
  cannot expose the internal in-flight controller state.
- Any production MC-native arbitrary single-joint position interface.
