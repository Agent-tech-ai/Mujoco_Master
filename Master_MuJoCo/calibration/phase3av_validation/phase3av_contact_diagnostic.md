# Phase 3A-V contact diagnostic

This is an offline, observation-only rerun of the frozen candidate arm-only
replay. Candidate parameters, controller code, MJCF, hardware mapping, and real
data were not modified. Frozen-source verification remained `13/13`.

## Confirmed contact pair

| body 1 | body 2 | sampled contact times | first / last sim time | maximum penetration |
| --- | --- | ---: | --- | ---: |
| `left_hip_roll_link` | `pelvis` | 690 | `0.560 / 14.341 s` | `0.001289 m` |

No other self-contact pair was observed. The contact starts during the initial
standing portion and persists through the replay, so it is **not an arm/torso
collision caused by the wave gesture**. It remains a valid acceptance blocker.

Classification: `PHYSICAL_MODEL_MISMATCH_CANDIDATE`. The present evidence does
not distinguish collision-geometry mismatch, standing-reference interaction,
or an unconfirmed physical joint zero. No physical model or mapping value was
changed.

## Safety interpretation

- Candidate arm-only: stable/no fall, no non-foot ground contact, no target
  clipping, positive minimum joint-limit margin (`0.04606 rad`), and no
  persistent actuator saturation; **fails** the no-collision rule above.
- Candidate whole-body measured-reference replay: falls at `8.037 s`, reaches
  a negative joint-limit margin (`-0.04501 rad`), and has persistent saturation
  fraction `0.11708`. This replay is an infrastructure/mapping check and is not
  used to assess autonomous balance prediction.

Raw diagnostic evidence is in
`diagnostics/phase3av_candidate_arm_only_contact_pairs.csv`.
