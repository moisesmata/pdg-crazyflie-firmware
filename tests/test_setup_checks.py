from datetime import datetime, timezone, timedelta
import pytest
from pdg.setup_checks import validate_recent_probe


def test_existing_firmware_requires_recent_matching_aircraft_probe():
    now = datetime(2026, 10, 22, tzinfo=timezone.utc)
    uri = "radio://0/80/2M/E7E7E7E7E7"
    probe = {"captured_at_utc": now.isoformat(), "uri": uri,
             "flight_commands_sent": False,
             "parameters": {"system.selftestPassed": "1", "pdg.apiVersion": "1",
                            "pdg.controllerId": "6", "pdg.capEnable": "0"}}
    validate_recent_probe(probe, uri, now)
    for key, value in [("uri", "radio://0/81/2M/E7E7E7E7E7"),
                       ("captured_at_utc", (now-timedelta(hours=2)).isoformat()),
                       ("flight_commands_sent", True),
                       ("parameters", {**probe["parameters"], "system.selftestPassed": "0"}),
                       ("parameters", {**probe["parameters"], "pdg.apiVersion": "2"})]:
        with pytest.raises(ValueError):
            validate_recent_probe({**probe, key: value}, uri, now)


@pytest.mark.parametrize('position,accepted', [([-1., 0., .014], True),
                                               ([0., 0., .014], False),
                                               ([-1., 0., .04], False)])
def test_hover_preparation_keeps_rocket_start_and_landing_plane(tmp_path, position, accepted):
    import json
    import shutil
    import subprocess
    import sys
    from pathlib import Path
    import yaml
    root = Path(__file__).resolve().parents[1]
    (tmp_path/'scripts').mkdir()
    (tmp_path/'config').mkdir()
    (tmp_path/'output/setup').mkdir(parents=True)
    (tmp_path/'output/rocket-setup-review').mkdir()
    shutil.copy(root/'scripts/prepare_rocket_hover.py', tmp_path/'scripts')
    mission = {'execution': {'max_start_error': .08, 'ground_body_height': .014,
                            'expected_tracker_parameters': {'ctrlMel.mass': .044}}}
    (tmp_path/'config/rocket-lab.yaml').write_text(yaml.safe_dump(mission))
    (tmp_path/'config/pdg-mellinger-hover.yaml').write_text(yaml.safe_dump({'calibration': {}, 'execution': {}}))
    (tmp_path/'output/setup/ground-calibration.json').write_text(json.dumps({'passed': True, 'position_m': position}))
    (tmp_path/'output/rocket-setup-review/flight-reference.json').write_text(json.dumps({'position_m': [[-1., 0., 2.014]]}))
    result = subprocess.run([sys.executable, str(tmp_path/'scripts/prepare_rocket_hover.py')], capture_output=True, text=True)
    assert (result.returncode == 0) is accepted, result.stdout + result.stderr
    assert (tmp_path/'output/setup/rocket-hover.yaml').exists() is accepted
