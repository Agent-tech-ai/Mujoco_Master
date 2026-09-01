# Calibration logs

- `real/`: read-only captures from the physical X2, converted without hidden sign/offset changes.
- `sim/`: MuJoCo exports using the same schema.
- `real/test.csv` and `sim/test.csv` are deterministic **synthetic smoke-test data**, not robot measurements. Regenerate them with `python calibration/generate_example_logs.py`.

Never place a motion-command utility in this directory. Phase 1 is state capture and offline analysis only.

