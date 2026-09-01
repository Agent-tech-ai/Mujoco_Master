# Phase 2B-2 active-test data status

No **real-robot** active-test CSV exists because the Phase 2B-2 safety gate returned **NO-GO** before any physical motion command was sent.

The following requested files are intentionally absent rather than empty or synthetic:

- `j2_left.csv`
- `j2_right.csv`
- `j7_left.csv`
- `j7_right.csv`

They may be created only from a physically executed, operator-authorized test after every item in `operator_preflight_confirmation.md` is confirmed. Static/passive snapshots must not be relabeled as active-test evidence.

Offline MuJoCo infrastructure rehearsal data is stored separately under `sim/`, with plots under `../plots/phase2b2_sim/`. Those artifacts validate target generation and logging only; they are not real-robot sign, zero, encoder-offset, or dynamics evidence.
