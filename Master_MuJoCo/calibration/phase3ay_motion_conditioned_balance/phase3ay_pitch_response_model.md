# Phase 3A-Y Pitch Response Model

Model class: **continuous, explainable activity/asymmetry gain scheduling**.

Inputs are live arm position/velocity-derived energy, instantaneous left/right asymmetry, sagittal-motion fraction, pelvis pitch and pitch rate, plus inherited joint/contact/saturation safety state. No motion name or preset ID is available.

At rest the model exactly returns the frozen Phase 3A-X distribution. During symmetric motion it moves normalized authority toward ankle pitch and reduces direct knee share. During unilateral motion it continuously returns toward the frozen distribution. Total pitch request and allocation weights are computed separately; absolute weights are normalized before safety redistribution.

This is a simulation controller response model, **not MC gain identification and not hardware calibration**.
