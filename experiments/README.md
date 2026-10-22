# Versioned hardware experiments and validated plans

These archives are included by `git add .`; general `output/` remains ignored.
Outcomes below distinguish completed missions from a fully streamed reference
whose landing check failed. A completed run does not establish repeatability.
Directory timestamps are UTC.

| Archive | Outcome | Measured evidence |
| --- | --- | --- |
| [pdg-2m-direct-baseline](pdg-2m-direct-baseline/) | Completed legacy quadrotor PDG | Mean/max tracking 3.17/4.93 cm; final XY error 2.52 cm; operator confirmation |
| [pdg-2m-repeat-20261021T202946Z](pdg-2m-repeat-20261021T202946Z/) | Completed legacy quadrotor PDG repeat | Mean/max tracking 5.89/10.16 cm; final XY error 6.34 cm; landing/disarm telemetry |
| [rocket-2p75m-20261022T015434Z](rocket-2p75m-20261022T015434Z/) | **Completed 2.75 m rocket-model descent and landing** | All 671 references; mean/max tracking 2.45/4.60 cm; confirmed disarm and zero motors; operator confirmation |
| [rocket-retarget-20261022T022742Z](rocket-retarget-20261022T022742Z/) | Full two-stage reference flown; **mission touchdown check failed** | Target switch near measured 1.487 m; all 1012 references; final XY error 17.84 cm; final status unarmed and motors zero |
| [rocket-attempts-20261022](rocket-attempts-20261022/) | Diagnostic failures | Complete logs for voltage, telemetry, start-state, tracking and crash investigations |
| [rocket-workstation-setup](rocket-workstation-setup/) | Historical settings and setup evidence | Recorded tracker parameters and original diagnostic outcomes; not fresh flight readiness |

Each main flight archive contains exact execution settings, model/trajectory
artifacts, validation, results, complete raw telemetry and checksum manifests.
New telemetry is stored as `mission.jsonl.gz` with lossless gzip compression.
The original legacy archives retain plain JSONL. Rejected solver candidates,
intermediate builds, caches and duplicate plots remain outside the archives.

The root source, tests, configuration and documentation must be committed with
these records. Archive-time source hashes do not establish flight-time source
identity unless separately documented. No archive is a substitute for current
hardware, frame, tracking and ground-state checks.

The launch helpers fall back to these saved plans and versioned tracker settings
when the local ignored copies are absent. See [commit/reproduction notes](../docs/versioned-experiments.md).
