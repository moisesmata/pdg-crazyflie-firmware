# Successful 2 m repeat — 2026-10-21 20:29:46 UTC

Source run: `output/pdg-2m-controller-repeat-20261021T202946Z`.
The mission exited successfully, sent all 351 descent samples, passed normal
landing checks, and confirmed disarming and four zero motor commands.
Average / maximum tracking errors: 5.89 / 10.16 cm. Final ground horizontal
error: 6.34 cm. These are telemetry results; no separate operator observation
was recorded. Earlier failures show that repeatability remains unresolved.

`mission.jsonl` is complete raw telemetry. `execution.json` records runtime
settings; `config.yaml` and `plan.json` preserve the configuration and trajectory.
`result.json` contains measured results; `manifest.json` records SHA-256 checksums
and provenance limits. Source hashes were captured at archive time, not during flight.

Sequence: climb from world XY [-1,0] to 2 m above the landing plane in 8 s,
settle 2 s, track a 7 s PDG descent to the origin, then request an 8 s landing.
Mellinger effective mass 44 g; planning mass 48 g. Kalman + IMU + Vicon full pose;
Flow deck removed. Explicit controller-trial profile, unchanged baseline settings.
