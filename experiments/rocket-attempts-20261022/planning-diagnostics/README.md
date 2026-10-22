# Retarget planner diagnostics

The original continuation candidates failed independent dense angular-rate
validation. The diagnostic log identifies the failure; diagnostic-target.json
is unconverged/rejected evidence and must not be flown. The refined solve used
25 nodes and a tighter 30% nodal margin, passed unchanged independent limits,
and produced the plans in the main retarget archive. The test log records the
143-test regression run before the hardware retarget attempts.
