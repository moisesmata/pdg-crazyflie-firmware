# Workstation settings and historical setup evidence

These are snapshots of local reports, not fresh readiness checks. Reports may
come from different attempts; retain their original timestamps and actual passed
or error fields. The tracker parameters reproduce the recorded effective 44 g
Mellinger settings and thrust scaling, not independently measured aircraft mass
or thrust. Revalidate after hardware, marker or frame changes.

config/pdg-tracker.json is the versioned fallback consumed by the tracker helper
when the local ignored candidate file is absent. Robot/radio, Vicon, body-frame
and tracker-frame configurations are versioned under the root config directory.
ROS revisions are pinned by ros.repos; compiler/ROS/Python build output is not
part of these archives. The hardware and ground calibration must be checked live.
