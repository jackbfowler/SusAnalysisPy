# SussyAnal

Suspension kinematics + quasistatic force analysis for Baja SAE vehicles, with
interactive Plotly visualizations. Python port of the MATLAB toolchain in
`SusAnalysis/` (reference only).

## Capabilities

| Command | What it does |
| --- | --- |
| `analyze` | Kinematic sweep (shock × steer): interactive 3-D model with sliders, live-linked envelope plots below, static envelope + component-analysis HTML |
| `forces` | Quasistatic force analysis at a target steer, 3-D force vector plot |
| `optimize` | Hardpoint grid-search optimizer (minimize bump steer, plunge, camber gain, …) |
| `tools/points_to_csv.py` | Convert a mm hardpoint export (plain rows or labelled `Point N:` blocks) into an inch CSV the analyzer can read |

## Install (once)

```bash
python3 -m venv .venv                      # venv is gitignored, lives in the repo
.venv/bin/pip install -e .                 # installs sussyanal + numpy + plotly
.venv/bin/pip install -e ".[dev]"          # optional: pytest
```

> Always use `.venv/bin/python` (or `.venv/bin/sussyanal`) — the system
> `python3` can't see the package. If the repo moves, recreate the venv
> (its launcher scripts embed absolute paths).

## Run — example calls

```bash
# Analyze a front suspension (2-D: shock + steering sweep)
.venv/bin/sussyanal analyze data/2026BajaFront_1-20.csv

# Analyze a rear suspension (1-D: no steering) into a custom output dir
.venv/bin/python -m sussyanal analyze data/2027BajaRear.csv --out-dir outputs

# Add 3-D surface plots to a 2-D analysis
.venv/bin/sussyanal analyze data/2026BajaFront_1-20.csv --surfaces

# Quasistatic forces
.venv/bin/sussyanal forces data/2026BajaFront_1-20.csv

# Optimizer (sweep outer track-rod ball joint Z, minimize bump steer)
.venv/bin/sussyanal optimize data/2026BajaFront_1-20.csv \
    --point OuterTrackRodBallJoint --axes 3 --range 1.0 --steps 50

# Convert mm hardpoints -> analyzer CSV
.venv/bin/python tools/points_to_csv.py pointsimport.txt data/2027BajaRear.csv
```

Outputs are self-contained Plotly HTML files in `outputs/` — open in any
browser, no server or display required.

### What you get from `analyze`

- `suspension3d.html` — interactive 3-D model; shock + steering sliders drive a
  live-linked envelope below (scroll down). A red vertical line marks current
  shock travel; the bold line is the current steering position.
- `kinematics_envelope.html` — static envelope (2-D sets), colorcoded by
  steering, with steering colorbar.
- `kinematics_component.html` — axle plunge, CV angles, arm articulation.
- `kinematics_curves.html` — 1-D sets: the same 3×3 envelope grid (no steering).
- `--surfaces` adds `kinematics_surfaces.html`.

All files are prefixed with the CSV name (e.g. `2027BajaFront_...`).

## Tests

```bash
.venv/bin/python -m pytest -q
```
