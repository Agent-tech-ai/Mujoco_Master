# Phase 2B-2 control ownership plan

Status: **PLAN ONLY / NO-GO — NOTHING IN THIS DOCUMENT WAS EXECUTED**

## Confirmed current architecture

- Live preflight on 2026-08-12 found `mc_ros2_node2263` as the sole publisher to `/aima/hal/joint/arm/command`, with `hal_ethercat_x21455` as the subscriber.
- The published message is the full 14-entry arm `JointCommandArray`, ordered left J1–J7 then right J1–J7.
- `GetMcAction` and `/aima/mc/common/state` reported `STAND_DEFAULT`.
- AimDK defines `STAND_DEFAULT` as stable standing with active force/dynamic balance behavior. The native MC command stream must therefore be treated as safety-critical whole-body control, not as an idle publisher.
- `GetCurrentInputSource` returned an empty source name. Official documentation says this can occur before MC has received an effective input; it does not imply ownership of the HAL joint-command topic.

## Two different ownership mechanisms

### MC-level commands

For commands consumed *by MC* (for example locomotion velocity), AimDK v0.7+ provides `SetMcInputSource`:

- ADD `1001`
- MODIFY `1002`
- DELETE `1003`
- ENABLE `2001`
- DISABLE `2002`

MC selects the highest-priority active registered source. The official example registers a custom source with a name, priority 40, and 1000 ms timeout. This is an MC arbitration mechanism.

### Direct HAL joint commands

The arm test considered here publishes directly to `/aima/hal/joint/arm/command`. Official joint-control examples and the FAQ explicitly require native MC to be stopped first with `aima em stop-app mc` so the direct controller can obtain control.

No official evidence found in the reviewed documentation says that registering an MC input source grants exclusive ownership of the direct HAL joint-command topic. The two mechanisms must not be conflated.

## Proposed future procedure, with hazard labels

| Step | Operation | Classification | Current status |
|---:|---|---|---|
| 1 | Physically test E-stop; confirm operator and swept volume | READ_ONLY / PHYSICAL_CHECK | UNKNOWN; mandatory |
| 2 | Query mode, input source, arm state, alerts, and command graph | READ_ONLY | Supported by existing preflight tooling |
| 3 | Confirm numeric position reserve, velocity/acceleration limits, effort thresholds, abort behavior, and communications-loss behavior | READ_ONLY / PHYSICAL_CHECK | UNKNOWN; mandatory |
| 4 | Approve a robot-support procedure for handing direct HAL ownership away from native MC | STATE_CHANGING / MOTION_CAPABLE | UNKNOWN; mandatory |
| 5 | If and only if explicitly approved, stop native MC using the official direct-HAL prerequisite | STATE_CHANGING / MOTION_CAPABLE | **Not authorized; not performed by project scripts** |
| 6 | Verify `mc_ros2_node*` is absent and arm command publisher count is exactly zero | READ_ONLY | Required immediately before publisher construction |
| 7 | Re-read all 14 arm states, recompute adaptive delta, verify limit reserve, temperature/voltage/error state | READ_ONLY | Implemented as guarded live preflight in the prepared script |
| 8 | Obtain two operator confirmations for exactly one joint and one round | PHYSICAL_CHECK | Required by prepared script |
| 9 | Create one publisher and execute current/+delta/return/-delta/return while monitoring state | MOTION_CAPABLE | Prepared but locked by the NO-GO machine-readable gate |
| 10 | On any threshold or publisher-count violation, hold latest measured positions for the approved interval and stop publishing | MOTION_CAPABLE | Strategy remains unapproved; gate is UNKNOWN |
| 11 | Restore native MC using a vendor-confirmed process and verify a single MC publisher | STATE_CHANGING / MOTION_CAPABLE | **Exact official restart procedure UNKNOWN** |
| 12 | Query MC action and follow the official mode transition route to `JOINT_DEFAULT`, then `STAND_DEFAULT` only when physically appropriate | STATE_CHANGING / MOTION_CAPABLE | Requires restored MC, feet firmly grounded, operator approval |
| 13 | Verify mode, input source, command publisher uniqueness, alerts, and stable state | READ_ONLY / PHYSICAL_CHECK | Mandatory after restoration |

## Acquire/release findings

- **MC-level acquire:** documented through `SetMcInputSource ADD`; selection is still priority/timeout arbitration, not an unconditional lock.
- **MC-level release:** DELETE/DISABLE exist in the service schema, but the reviewed examples do not demonstrate a complete acquire-use-release lifecycle. Exact project policy and error recovery remain to be approved.
- **Direct HAL acquire:** reviewed official material says native MC must be stopped.
- **Direct HAL release/MC restart:** no authoritative `start-app mc` recovery procedure was found in the reviewed X2 documentation. It would be unsafe to infer one by symmetry with `stop-app`. This remains `UNKNOWN` pending AgiBot support or installed-firmware documentation.
- **Restoring `STAND_DEFAULT`:** official mode documentation supports `JOINT_DEFAULT` followed by `STAND_DEFAULT` along the transition diagram and warns that the feet must be on the ground. This does not replace the missing MC restart and post-direct-control recovery procedure.

## Decision

The prepared test must remain NO-GO until AgiBot confirms the direct-HAL ownership handoff, abort/loss-of-command behavior, and exact native-MC restoration procedure for the installed firmware. Project scripts never stop/restart MC or switch modes automatically.

## Sources

- https://x2-aimdk.agibot.com/zh-cn/latest/Interface/control_mod/MC_control.html
- https://x2-aimdk.agibot.com/zh-cn/latest/example/Python.html#id15
- https://x2-aimdk.agibot.com/en/latest/faq/index.html
- https://x2-aimdk.agibot.com/en/dev/Interface/control_mod/modeswitch.html
- Local live evidence: `../../work/x2_phase2b2_preflight.txt`

