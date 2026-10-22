# Two-stage rocket landing retarget

Prepared from the validated 2.75 m rocket-model trajectory. The nominal start
is world ENU [-1.375, 0, 2.7643266757950187] m, 2.75 m above the existing pad
plane. The first stage initially targets [0,0] m. At the reference's 1.5 m
height crossing (7.321589 s into descent), the second stage targets world XY
[0,-1] m and the same ground-body height. World z=0 in the operator request
means ground contact; the tracked-body offset remains 0.014326675795018673 m.

Both stages are solved offline with the mass-depleting, gimbaled 6-DOF rocket
model. The switch is scheduled by the reference crossing, not triggered by
measured altitude or solved online. The second solve carries forward rocket
mass, position, velocity, quaternion, angular rates, engine thrust and its
initial time derivative. Only the target origin translates. Position, velocity,
acceleration and mapped attitude are continuous through the abrupt target change.
The virtual rocket's attitude/mass are not physical drone measurements.

The second stage uses 25 nodes and a 30% nodal tilt/rate margin. Independent
DOP853 replay validates each rocket target, and dense mapped-drone validation
checks the combined reference, including the seam. Total descent duration is
20.207661 s. The planned landing is near world ENU [0,-1,0.014326676] m.
Normal landing remains 3 seconds; recovery remains 8 seconds. Host voltage
thresholds and the tracking-error abort remain disabled as previously requested.
Start-state, freshness, timing, supervisor and touchdown checks still apply.

```bash
cd /home/gravitas/workspaces/moises-experiments/pdg/pdg-crazyflie-firmware
source scripts/local_env.sh
python -m pdg.rocket.retarget             # Reload and revalidate saved stages
python -m pdg.rocket.retarget --execute   # Arm, take off, stream both stages, land
```

`output/rocket-retarget-prepared/retarget-plan.json` identifies both target plans
and switch timing. Each flight gets a fresh `output/rocket-retarget-flight-*`
folder containing the plans, exact execution configuration and telemetry.
`rocket_emulation_sample.rocket_phase` identifies stage 1 or 2.
Keep the tracking/radio server running through touchdown. Use the existing
`bash scripts/rocket.sh land` command to request an early landing.
