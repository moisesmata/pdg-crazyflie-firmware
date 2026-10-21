# Development validation — 2026-10-20

Local environment: macOS Apple Silicon, Python 3.12.13, xPack Arm GCC 14.2.1-1.1. These are development checks, not flight qualification.

- Both `make firmware PLATFORM=cf2` and `PLATFORM=cf21bl` completed. The build verifies the selected platform, enabled OOT controller and required arming, and linked `controllerOutOfTree` / `pdgLimitCollective` symbols. Platform-specific ELF exports were compared against their source build artifacts; CF2 and Brushless hashes differ. No device was flashed.
- `make test`: 14 tests passed. Coverage includes native collective-limit edge cases, platform-manifest mismatch rejection, nonlinear integration/derivatives, SCvx convergence and independent rollout, reference message units, and simulated mission sequencing/abort behavior.
- `scripts/simulation_sweep.py` retains four nominal descent cases in `output/smoke-study/`: vertical (0.5 m), east/west (1 m), and diagonal (1.5 m). All four converged and passed the independent rollout checks (1e-3 numerical tolerance per reported residual). Results are numerical smoke checks, not randomized robustness evidence. The 1.8 s lower time bound is part of the experiment.
- `pdg mission --height 1.0` converged in 13 iterations and passed validation. This is a dry-run planning/validation command; it does not connect to ROS or issue flight commands.

Reproduce with `make test`, `.venv/bin/python scripts/simulation_sweep.py`, and `.venv/bin/pdg mission --height 1.0`. Generated JSON reports include iteration histories and all rollout validation residuals. Firmware manifests record upstream revision, source hashes, configuration hash, compiler and ELF hash.

Outstanding: Linux ROS runtime integration, Crazyswarm2 closed-loop simulation, vehicle identification, frame/marker-origin verification, and physical tracking/touchdown tests. The optimized-force rollout is not a simulation of the onboard Mellinger feedback loop. Dense sampling and numerical tolerances do not certify continuous-time constraint satisfaction. No fault recovery guarantee, calibrated failure injection, real-time replanning, or hardware experiment is claimed.
