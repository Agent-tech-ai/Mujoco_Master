# Interactive demo keyboard controls

Run from `Master_MuJoCo`:

```powershell
python run_interactive_demo.py
```

The viewer starts in `STANDING`. Click/focus the MuJoCo window before using keys.

| Key | Behavior |
|---|---|
| `H` | Play the frozen measured Heart arm trajectory once. |
| `V` | Play the frozen measured right-hand Wave arm trajectory once. |
| `C` | Play the frozen measured Clap arm trajectory once. |
| `SPACE` | Smoothly interrupt the active action and return through `STOPPING` to `STANDING`; while idle, hold standing. |
| `R` | Explicit Demo reset: stop action, `mj_resetData`, restore validated standing initialization/controller, then `STANDING`. |
| `1` | Select `SIMULATION_ONLY_LOW`. Reserved for future locomotion; measured actions remain at validated 1.0x timing. |
| `2` | Select `SIMULATION_ONLY_MEDIUM` (default). Reserved for future locomotion. |
| `3` | Select `SIMULATION_ONLY_HIGH`. Reserved for future locomotion. |
| `W` / `S` | Prints `LOCOMOTION_NOT_AVAILABLE`; no command is sent. |
| `A` / `D` | Prints `LOCOMOTION_NOT_AVAILABLE`; no command is sent. |
| `Q` / `E` | Prints `LOCOMOTION_NOT_AVAILABLE`; no command is sent. |
| `ESC` | Close the viewer and exit. |

Action and mode keys are edge-debounced. `H`, `V`, or `C` pressed during an action is ignored as busy; actions cannot overlap. Key release is not mapped to a joint target because locomotion is unavailable.

## State machine

The implemented states are `STANDING`, `LOCOMOTION`, `ACTION_PLAYBACK`, `STOPPING`, `RESETTING`, and `SAFETY_HOLD`. `LOCOMOTION` is defined but unreachable until a continuous actuator-driven gait is validated inside the frozen safety/controller stack.

Normal action flow:

```text
STANDING -> ACTION_PLAYBACK -> smooth recovery -> STANDING
                       SPACE -> STOPPING -> STANDING
any state                  R -> RESETTING -> STANDING
safety violation             -> SAFETY_HOLD
```

`SAFETY_HOLD` rejects new actions and uses the existing controller with a smooth standing recovery target. It never teleports the model. Use `R` only when an explicit Demo reset is wanted.

Terminal messages are event-based, not per simulation step: `[STATE]`, `[ACTION]`, `[COMMAND]`, `[SPEED]`, and `[SAFETY]`.
