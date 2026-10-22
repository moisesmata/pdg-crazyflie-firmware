# 2.75 m rocket test

The operator selected full-path scaling and confirmed room clearance and ground
placement for this test. The active configuration is `config/rocket-lab.yaml`,
with a named copy at `config/rocket-2p75m.yaml`. The prior 2 m configuration is
preserved at `config/rocket-2m-voltage-disabled.yaml`.

The rocket target remains the normalized 14-state mass-depleting, gimbaled
6-DOF model. Length scaling is 2.75/5.33 m per target length unit; time scaling
remains 2 s per target time unit. This maps the start to world ENU
[-1.375, 0, 2.7643266757950187] m: 2.75 m above the original pad plane.
The aircraft takes off vertically from that XY and descends to the original
landing origin. Vicon must keep its existing world origin.

The declared planning envelope is world Z <= 3.1 m and |X|, |Y| <= 1.625 m.
These values are software limits, not measurements of the room. Existing speed,
tilt, motor reserve, freshness, start-state and touchdown checks remain. The
host tracking-error abort is explicitly disabled (`max_tracking_error: null`)
at the operator’s request. Normal landing is shortened to 3 seconds; recovery
landing remains 8 seconds.
Both host voltage thresholds remain disabled by the operator's earlier request;
firmware protections and valid/fresh status telemetry remain required.

The fresh rocket solve is saved to `output/rocket-2p75m-prepared`. Its target is
independently replayed and its mapped drone reference checked before flight.
The launcher now selects this plan:

```bash
cd /home/gravitas/workspaces/moises-experiments/pdg/pdg-crazyflie-firmware
bash scripts/rocket.sh review           # Offline validation only
bash scripts/rocket.sh fly --execute    # Arms, takes off, descends and lands
```

The radio/Vicon server must remain running throughout flight and touchdown.
Controller settings must be reapplied after a restart/power cycle using the
existing grounded-state helper. Flight output has a separate UTC-stamped
`output/rocket-flight-*` directory, including exact execution settings and logs.

An early landing can be requested with `bash scripts/rocket.sh land`; this is
not an independent motor cut or touchdown confirmation.
