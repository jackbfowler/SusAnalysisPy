# SussyAnal

Python port of the Baja SAE suspension analysis MATLAB tooling — the
`sussy_steer` kinematics analyzer, quasistatic force solving, and interactive
Plotly visualization.

Configuration (`bump`, `droop`, `wheel_size`, `steer_sweep`,
`shock_mount_lca`, `wheelbase`) is specific to each hardpoint set: it is read
from that set's own CSV and bundled with its geometry.

See [ARCHITECTURE.md](ARCHITECTURE.md) for the design and `agents.md` for
development guidance.

## Install

The package is installed into the project virtual environment (`.venv/`),
**not** into the system Python.

```bash
# from the repo root
python3 -m venv .venv                      # once (already done in this sandbox)
.venv/bin/pip install -e .                 # installs sussyanal + numpy + plotly
.venv/bin/pip install -e ".[dev]"          # optional: adds pytest for testing
```

> `python3 -m sussyanal ...` fails with `No module named sussyanal` because the
> system interpreter doesn't know about the venv. Always use the venv's Python
> (below) or activate the venv first.

## Usage

Use the venv's Python (or its `sussyanal` entry-point script) — either works:

```bash
# 1) run directly with the venv interpreter
.venv/bin/python -m sussyanal analyze data/2026BajaFront_1-20.csv --out-dir outputs

# 2) use the installed console script (same thing)
.venv/bin/sussyanal analyze data/2026BajaFront_1-20.csv --out-dir outputs

# 3) activate the venv once per shell, then plain `python` / `sussyanal` work
source .venv/bin/activate
python -m sussyanal analyze data/2026BajaFront_1-20.csv --out-dir outputs
```

### Commands

```bash
# Kinematic sweep (sussy_steer) + interactive 3-D + curve/envelope plots
sussyanal analyze <csv> [--n-shock 100] [--out-dir outputs]

# 2-D sets write envelope plots by default; add --surfaces for surface plots
sussyanal analyze <csv> --surfaces

# Quasistatic force analysis + force visualization
sussyanal forces <csv> [--out-dir outputs]

# Hardpoint grid-search optimizer
sussyanal optimize <csv> [--point OuterTrackRodBallJoint] [--axes 3] \
                       [--range 1.0] [--steps 50] [--objective bump_steer] \
                       [--n-shock 30] [--out-dir outputs]
```

Outputs are self-contained Plotly HTML files, viewable in any browser — no
display server required (works over SSH/VS-Codium).

## Live-linked plots (single page)

For 2-D hardpoint sets, `analyze` writes `suspension3d.html` as **one page**:
the interactive 3-D viewer fills the top (with its shock + steering sliders),
and the envelope plots sit below — scroll down to see them. The page opens
**static** (no auto-play); moving the sliders updates the envelopes in place,
like the MATLAB visualizer:

- **steering slider** — switches which steering line is highlighted (bold),
- **shock slider** — moves the red dot along the highlighted line.

Envelope steering lines are colorcoded by steering position (Plotly
`sunsetdark`, full negative → full positive steer). A static standalone
envelope (`kinematics_envelope.html`, with the steering colorbar) and a static
**component-analysis** plot (`kinematics_component.html` — axle plunge, CV
angles, and LCA/UCA articulation angles, same envelope style) are also written
for 2-D sets. No server needed — open the HTML directly.

## Testing

```bash
.venv/bin/python -m pytest              # run the test suite
.venv/bin/python -m pytest -q           # quiet
```

Quick smoke check that the install is healthy:

```bash
.venv/bin/python -c "import sussyanal; print(sussyanal.__version__)"
```
