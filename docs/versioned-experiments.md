# What a normal git add includes

The source, tests, config files, runbooks and curated experiments are versioned.
The current successful 2.75 m rocket run and the full-reference retarget run are
indexed in `experiments/README.md`. Their exact plans, execution settings,
validation, results and complete compressed telemetry are included. Failed
attempts with useful diagnostic evidence are archived separately and are not
labelled successful. Existing successful legacy PDG experiments remain intact.

`output/` remains ignored because it also contains duplicate reviews, rejected
solutions and temporary logs. The curated archives remove that dependency:

- `scripts/rocket.sh` uses its local 2.75 m plan when available, otherwise the
  archived successful rocket target plan.
- `python -m pdg.rocket.retarget` loads the prepared local two-stage bundle when
  available, otherwise the versioned retarget bundle. It independently validates
  both targets and the combined mapping again. It does not fly without `--execute`.
- `scripts/configure_pdg_tracker.py` previews the local candidate when present,
  otherwise `config/pdg-tracker.json`. `--candidate PATH` selects an explicit
  file. Applying settings still requires `--apply` and live stopped/unarmed checks.

After cloning, install the project environment, initialize the firmware
submodules, and import/build the ROS dependencies pinned by `ros.repos` using
`docs/local-setup.md`. Build/venv/toolchain directories are intentionally ignored.
The local `ros_ws/src` symlink is not portable and is intentionally ignored;
reimport the pinned dependencies on another workstation.

Before hardware use, review the recorded aircraft assumptions and confirm the
actual calibration and frames. The current test profiles retain the operator's
disabled host voltage/tracking-error thresholds and 3-second normal landing;
firmware protections and other checks remain. The effective 44 g mass and thrust
scaling are recorded settings, not independent physical identification.

To inspect compressed logs without changing the archive:

```bash
python -c 'import gzip; print(gzip.open("experiments/rocket-2p75m-20261022T015434Z/mission.jsonl.gz", "rt").readline())'
```

`archive-manifest.json` hashes every archived file. Compression is lossless, and
original output files have not been changed. Source hashes marked at archive
time describe the working tree at that time; preserved original flight
manifests carry their own provenance limits.
