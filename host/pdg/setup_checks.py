"""Checks for commissioning with an already-installed firmware app."""
from datetime import datetime, timezone


def validate_recent_probe(probe, expected_uri, now=None):
    now = now or datetime.now(timezone.utc)
    captured = datetime.fromisoformat(probe["captured_at_utc"])
    age = (now - captured).total_seconds()
    parameters = probe.get("parameters", {})
    expected = {"system.selftestPassed": "1", "pdg.apiVersion": "1",
                "pdg.controllerId": "6", "pdg.capEnable": "0"}
    if (probe.get("uri") != expected_uri or probe.get("flight_commands_sent") is not False
            or not 0 <= age <= 3600
            or any(str(parameters.get(k)) != v for k, v in expected.items())):
        raise ValueError("Require a successful read-only probe of this aircraft within the last hour; stop the radio server and rerun start_ground_checks.sh")
