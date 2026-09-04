# SuspensionAnalysis

Suspension kinematics + quasistatic force analysis for Baja SAE, with
interactive Plotly visualizations.

## Capabilities

| Command | What it does |
| --- | --- |
| `analyze` | Kinematic sweep (shock × steer): interactive 3-D model with sliders, live-linked envelope plots below, static envelope + component-analysis HTML |
| `forces` | Quasistatic force analysis at a target steer, 3-D force vector plot |
| `optimize` | Hardpoint grid-search optimizer (minimize bump steer, plunge, camber gain, …) |
| `tools/points_to_csv.py` | Import ONE LOTUS mm corner export into the shared `datanew/*.txt` (`--axle front\|rear`, merges) |
| `tools/csv_to_shared.py` | Migrate a legacy front+rear `data/*.csv` pair into one shared `datanew/*.txt` |

## Install (once)

```bash
python3 -m venv .venv                      # venv is gitignored, lives in the repo
.venv/bin/pip install -e .                 # installs sussyanal + numpy + plotly
.venv/bin/pip install -e ".[dev]"          # optional: pytest
```

> use `.venv/bin/python` (or `.venv/bin/sussyanal`) — the system
> `python3` can't see the package. If the repo moves, recreate the venv
> (its launcher scripts embed absolute paths).

## Run — example calls

Shared `datanew/*.txt` files hold front + rear for a year/version; pick the
corner with `--axle`. Legacy per-corner `data/*.csv` files still run but are
deprecated.

```bash
# Analyze the FRONT corner (2-D: shock + steering sweep)
.venv/bin/sussyanal analyze datanew/2026Baja.txt --axle front

# Analyze the REAR corner of the same file (1-D: no steering travel)
.venv/bin/sussyanal analyze datanew/2026Baja.txt --axle rear

# Add 3-D surface plots to a 2-D analysis, into a custom output dir
.venv/bin/sussyanal analyze datanew/2026Baja.txt --axle front --surfaces --out-dir outputs

# Quasistatic forces
.venv/bin/sussyanal forces datanew/2026Baja.txt --axle front

# Optimizer (sweep ex: outer track-rod ball joint Z, minimize bump steer)
.venv/bin/sussyanal optimize datanew/2026Baja.txt --axle front \
    --point OuterTrackRodBallJoint --axes 3 --range 1.0 --steps 50

# Import one LOTUS corner export (mm) into the shared file
.venv/bin/python tools/points_to_csv.py pointsimport.txt datanew/2027Baja.txt --axle rear

# Migrate an existing legacy front+rear CSV pair into the shared format
.venv/bin/python tools/csv_to_shared.py data/2026BajaFront_12-16.csv \
    data/2026BajaRear_3-3.csv -o datanew/2026Baja.txt
```

Outputs are self-contained Plotly HTML files in `outputs/` — open in any
browser, no server or display required.

### `analyze`

- `suspension3d.html` — interactive 3-D model; shock + steering sliders drive a
  live-linked envelope below (scroll down). A red vertical line marks current
  shock travel; the bold line is the current steering position.
- `kinematics_envelope.html` — static envelope (2-D sets), colorcoded by
  steering, with steering colorbar.
- `kinematics_component.html` — axle plunge, CV angles, arm articulation.
- `kinematics_curves.html` — 1-D sets: the same 3×3 envelope grid (no steering).
- `--surfaces` adds `kinematics_surfaces.html`.
