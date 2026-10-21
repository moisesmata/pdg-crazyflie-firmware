# Successful 2 m direct-start PDG baseline

Hardware run: `output/pdg-2m-headingfree-20261021T182906Z`.
The operator confirmed the aircraft grounded with propellers stopped and reported:
“this looked really good”.

`config.yaml` and `plan.json` freeze the actual 2 m direct-start setup;
`result.json` records quantitative tracking and landing results; `manifest.json`
records source hashes and the runtime configuration. No gains were changed after
this successful run. Complete raw telemetry is archived here in `mission.jsonl`, with runtime
`execution.json`. `flight-plan.json` preserves the original flight output; its
trajectory matches `plan.json`, with different validation diagnostics.
Original output files remain in the ignored source run directory.

The vehicle takes off directly beneath world XY approximately [-1,0], climbs
to 2 m above the landing plane in 8 s, settles for 2 s, executes the saved 7 s
PDG descent to the original landing origin, then requests an 8 s bounded landing.
Total observed takeoff-to-disarm time: 27.72 s including ground confirmation.

Tracking mean/max errors: 0.0317 / 0.0493 m. Final ground horizontal error:
0.0252 m. The full 351-point stream completed, normal landing checks passed,
supervisor disarming and four zero motor commands were confirmed.
Minimum logged voltage: 3.425 V; launch voltage: 4.053 V.

This is one successful hardware baseline, not a demonstration of repeatability,
all battery conditions, or the earlier 2.5 m mission. Planning mass 48 g and
tracker effective mass 44 g remain model assumptions/calibration, not a scale
measurement. Ground marks, mocap axes and original landing origin must remain
consistent. The heading gate is disabled only for the explicit controller-trial
profile; tilt, fresh data, startup and settled-start checks remain.

Offline review only (does not connect to ROS or send commands):

```bash
pdg mission --config experiments/pdg-2m-direct-baseline/config.yaml \
  --plan-file experiments/pdg-2m-direct-baseline/plan.json \
  --controller-trial --output output/baseline-review
```

Do not append `--execute` without a fresh physical readiness and live telemetry
check. The launcher starts PID; the calibrated Mellinger settings must be
applied while unarmed before the PDG mission.
