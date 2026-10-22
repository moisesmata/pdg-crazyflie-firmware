# Rocket-emulation validation — 2026-10-21

Development checks on macOS, using a temporary Python environment. No firmware
was flashed and no hardware mission was executed.

- Default `pdg plan`: 34 SCvx iterations; independent target replay passed.
  Target burn time 6.6990 normalized time units; terminal position residual
  0.000168 target length units; normalized maximum node replay defect 8.95e-05.
- Mapping: 2 m rise above the pad body-origin plane and about 1 m lateral travel;
  13.398 s lab descent, peak speed 0.2320 m/s, peak drone tilt 0.3953 degrees,
  specific thrust 9.7493–9.9962 m/s². All declared mapped nominal limits passed
  across 2,699 grid samples. Motor feasibility uses assumed parameters.
- These are nonlinear target-replay and reference-feasibility checks, not a
  closed-loop tracking experiment, formal certificate, or real-time benchmark.
- Saved-target mission dry run passed; an intentionally over-tight mapped speed
  limit was rejected before the ROS path.
- Regression suite: 132 tests passed, including 10 new rocket model/scaling/
  provenance/mission tests. The full
  suite retains a pre-existing failure in `test_heading_packets.py` because this
  checkout lacks the pinned ROS workspace's `crazyflie_cpp/crtp.h`. That test also
  failed before this change. Firmware source/submodule and archived flights were
  not changed.

See [rocket-emulation.md](rocket-emulation.md) for commands, scaling and scope.
The following historical checks concern the **legacy quadrotor backend**. Use
an explicit legacy config to reproduce them now that the CLI default is rocket PDG.

---

# Development validation — 2026-10-20

Local environment: macOS Apple Silicon, Python 3.12.13, xPack Arm GCC 14.2.1-1.1. These are development checks, not flight qualification.

- Both `make firmware PLATFORM=cf2` and `PLATFORM=cf21bl` completed. The build verifies the selected platform, enabled OOT controller and required arming, and linked `controllerOutOfTree` / `pdgLimitCollective` symbols. Platform-specific ELF exports were compared against their source build artifacts; CF2 and Brushless hashes differ. No device was flashed.
- `make test`: 14 tests passed. Coverage includes native collective-limit edge cases, platform-manifest mismatch rejection, nonlinear integration/derivatives, SCvx convergence and independent rollout, reference message units, and simulated mission sequencing/abort behavior.
- `scripts/simulation_sweep.py` retains four nominal descent cases in `output/smoke-study/`: vertical (0.5 m), east/west (1 m), and diagonal (1.5 m). All four converged and passed the independent rollout checks (1e-3 numerical tolerance per reported residual). Results are numerical smoke checks, not randomized robustness evidence. The 1.8 s lower time bound is part of the experiment.
- `pdg mission --height 1.0` converged in 13 iterations and passed validation. This is a dry-run planning/validation command; it does not connect to ROS or issue flight commands.

Reproduce with `make test`, `.venv/bin/python scripts/simulation_sweep.py`, and `.venv/bin/pdg mission --height 1.0`. Generated JSON reports include iteration histories and all rollout validation residuals. Firmware manifests record upstream revision, source hashes, configuration hash, compiler and ELF hash.

Outstanding: Linux ROS runtime integration, Crazyswarm2 closed-loop simulation, vehicle identification, frame/marker-origin verification, and physical tracking/touchdown tests. The optimized-force rollout is not a simulation of the onboard Mellinger feedback loop. Dense sampling and numerical tolerances do not certify continuous-time constraint satisfaction. No fault recovery guarantee, calibrated failure injection, real-time replanning, or hardware experiment is claimed.
