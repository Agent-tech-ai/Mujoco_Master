# Phase 3A-V workflow

1. The onsite operator confirms E-stop, clear workspace, stable standing, MC/balance normal, and approval for the already-verified `wave(right)` preset.
2. Run the subscription-only recorder from Windows PowerShell:

   ```powershell
   powershell.exe -NoProfile -ExecutionPolicy Bypass -File "C:\Users\xinga\Documents\Codex\2026-08-11\master-mujoco-agibot-x2-ultra-mujoco\work\run_x2_phase3av_wave_capture_readonly.ps1" -Target "run@192.168.4.114" -CaptureSeconds 120 -OperatorSafetyConfirmed
   ```

3. Wait for `PHASE 3A-V RECORDING STARTED`, then wait again for `PRE-ROLL COMPLETE`.
4. Only the onsite operator executes `Agentech.wave(right)` through the existing MC-compatible pipeline. Keep at least five seconds of post-roll.
5. After capture, run the offline stages:

   ```powershell
   C:\Python3.12\python.exe Master_MuJoCo\calibration\phase3av_validation\process_phase3av_capture.py
   C:\Python3.12\python.exe Master_MuJoCo\calibration\phase3av_validation\run_phase3av_replays.py
   C:\Python3.12\python.exe Master_MuJoCo\calibration\phase3av_validation\analyze_phase3av_validation.py
   ```

The quality processor stops before replay when any data gate fails. The replay verifies the frozen hashes and loads only joint position/velocity, never `reported_effort`.
