# Phase 3A-X controller candidate

Classification: `SAFETY_ARCHITECTURE_CANDIDATE_NOT_VALIDATED_RESPONSE_BASELINE`.

It combines frozen arm tracking, pitch/roll separation, direction-aware limit
envelope, pre-contact gradient retreat, channel slew, actuator-authority scaling,
eligibility-safe allocation, whole-body tracking arbitration, and a +0.025 rad
left hip-roll simulation standing offset. It solves the tested hard-safety chain
and local perturbations, but heart/wave balance response still fails the declared
similarity band. It is not hardware or dynamics calibration.
