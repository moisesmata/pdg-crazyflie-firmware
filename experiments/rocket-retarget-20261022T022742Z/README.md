# Full retarget reference flown; touchdown check failed

The complete 1012-point two-stage reference ran for 20.22 seconds. The first
stage started at world XY [-1.375,0], 2.75 m above the pad, initially targeting
[0,0]. At the precomputed reference crossing of 1.5 m (7.321589 seconds), stage
two targeted world XY [0,-1]. Measured switch height was approximately 1.487 m.
Both stages use the 6-DOF rocket model, with a continuous full-state/thrust join.
The switch is precomputed, not an online measured-state replanning operation.

The aircraft ended near [0.042,-1.173] m, 17.84 cm from the new site. Mean/max
tracking errors were 8.04/35.35 cm. The normal touchdown check and recovery
confirmation did not pass. Final fresh supervisor status reported unarmed and
all four motor commands were zero. This is full-reference flight evidence,
NOT a successfully completed mission or a precision landing demonstration.

Normal landing was 3 seconds. Host launch-voltage, airborne-voltage and tracking-
error abort thresholds were disabled. Other checks remained active. The target
plane includes the tracked body-origin height of 0.014326675795018673 m.

Both target plans, exact stage configurations, handoff manifest, independent
validation and mapped references are included. mission.jsonl.gz preserves all
raw telemetry. The rejected diagnostic solve is deliberately not part of this
archive. See result.json for the measured outcome and archive-manifest.json
for checksums and provenance limits.
