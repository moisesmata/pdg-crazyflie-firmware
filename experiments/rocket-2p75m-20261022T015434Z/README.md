# Completed 2.75 m rocket-model descent

This run streamed all 671 references over 13.40 seconds, passed the normal
landing checks, and confirmed disarming and four zero motor commands. Mean/max
tracking errors were 2.45/4.60 cm. Takeoff-to-disarm lasted 28.64 seconds.
The operator reported that this run was perfect.

Normal landing was 3 seconds. The host airborne-voltage, launch-voltage and
tracking-error thresholds were disabled for this run. Other startup, telemetry,
deadline, supervisor and touchdown checks remained. See execution.json for the
exact configuration; config.yaml freezes the mission configuration.

The reference uses the normalized mass-depleting, gimbaled 6-DOF rocket model,
scaled to start 2.75 m above the original pad at world XY [-1.375,0]. The drone
emulates rocket translation. Its actual mass and motor model remain assumptions.

mission.jsonl.gz contains complete raw telemetry. Decompress with Python gzip
or gzip -dc. manifest.json preserves the original post-flight source hashes;
archive-manifest.json adds file checksums and explicit archive-time provenance.
This single completed run is not a demonstration of repeatability.
