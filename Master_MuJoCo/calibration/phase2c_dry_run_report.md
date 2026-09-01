# Phase 2C dry-run wrapper report

Status: **OFFLINE ONLY / NO-GO**

## Implementation

`calibration/phase2c_dry_run.py` is intentionally incapable of motion. It:

- imports Python standard-library modules only;
- reads the saved local Phase 2B-2 arm snapshot;
- performs a hardware control-coordinate limit/reserve calculation;
- prints the selected joint, current-position source, delta, target, proposed
  interface label, duration, safety limits, and return target;
- creates no ROS node, publisher, service/action client, SDK session, or SSH
  connection;
- has no motion execution mode; `--dry-run` is mandatory.

The interface defaults to `NO_APPROVED_MC_SINGLE_JOINT_INTERFACE`. Supplying an
interface string changes only printed text; it never imports or calls it.

## Verified sample

Command:

```powershell
python calibration/phase2c_dry_run.py --dry-run --joint left_wrist_roll_joint --delta-deg 1 --duration-s 3
```

Key result from the saved 2026-08-12 snapshot:

```text
DRY_RUN_ONLY=true
MOTION_CAPABILITY_PRESENT=false
selected_joint=left_wrist_roll_joint
current_position_deg=-4.382304013
target_delta_deg=+1.000000000
target_position_deg=-3.382304013
command_interface=NO_APPROVED_MC_SINGLE_JOINT_INTERFACE
safety_limit_deg=[-90.012000,41.482000] FIELD_TEST_EVIDENCE
required_limit_reserve_deg=5.000000
return_target_deg=-4.382304013
publisher_count_check=NOT_PERFORMED_OFFLINE
control_input_source_check=NOT_PERFORMED_OFFLINE
STATUS=PLAN_GEOMETRIC_CHECK_PASS
```

`PLAN_GEOMETRIC_CHECK_PASS` means only that this stale offline target lies
inside the supplied hardware limit with the requested reserve. It is not a GO
decision and is not evidence that the command interface, sign, ownership,
velocity/acceleration limits, or abort behavior is safe.

## Remaining gate

The standing `heart()` source trace ends in native MC preset 1007, but that
interface cannot express an arbitrary single-joint delta. No approved
MC-preserving J2/J7 interface was found, so the wrapper deliberately prints the
NO-GO interface label. No control-layer conclusion is inferred from the
installed SDK's abstract capability comments.
