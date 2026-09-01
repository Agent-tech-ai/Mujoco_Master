# Phase 3A-V frozen candidate replay

Configuration is loaded without modification from the locked Phase 3A candidate.

- stable/no fall: `True`
- collision/non-foot contact: `690/0`
- target clips: `0`
- persistent saturation fraction: `0.00000`
- minimum joint-limit margin: `0.04606 rad`
- mean active-arm RMSE/|lag|: `0.12443 rad / 0.147 s`
- source lock verification: `13/13`

All 690 collision-positive samples are the single persistent pair
`pelvis <-> left_hip_roll_link` (maximum penetration `1.289 mm`), beginning in
the initial standing window. This is not a wave arm/torso contact, but it still
fails the acceptance rule. See `phase3av_contact_diagnostic.md`.

**NOT HARDWARE CALIBRATION.**
