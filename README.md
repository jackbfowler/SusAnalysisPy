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

```bash
python -m venv .venv
.venv/bin/pip install -e .
```

## Usage

```bash
# Kinematic sweep (sussy_steer) + interactive 3-D + curve/surface plots
.venv/bin/python -m sussyanal analyze data/2026BajaFront_1-20.csv --out-dir outputs

# Quasistatic force analysis + force visualization
.venv/bin/python -m sussyanal forces data/2026BajaFront_1-20.csv --out-dir outputs

# Hardpoint grid-search optimizer
.venv/bin/python -m sussyanal optimize data/2026BajaFront_1-20.csv --out-dir outputs
```

Outputs are self-contained Plotly HTML files, viewable in any browser — no
display server required (works over SSH/VS-Codium).
