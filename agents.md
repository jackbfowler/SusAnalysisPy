# agents.md

Guidance for AI coding agents working in this repository. See
[ARCHITECTURE.md](ARCHITECTURE.md) for the full design and roadmap.

## Project

SussyAnal is a suspension kinematics and force analysis tool for Baja SAE
vehicles. This repository is a Python port of the MATLAB implementation in
`SusAnalysis/` (reference only, gitignored — never edit or depend on it).

## Repository layout

- `src/sussyanal/io.py` — CSV hardpoint/config parsing.
- `src/sussyanal/geometry.py` — math helpers, dataclasses, `SuspensionModel`.
- `src/sussyanal/kinematics/solver.py` — `solve_sweep()` + `KinematicResults`.
- `src/sussyanal/kinematics/steer.py` / `shock_only.py` — analyzers.
- `src/sussyanal/kinematics/optimize.py` / `tie_on_arm.py` — optimizers (stubs, TODO).
- `src/sussyanal/forces/` — `solve_forces.py` + `run_quasistatic.py`.
- `src/sussyanal/plotting/` — Plotly figures (curves, surfaces, 3-D, forces).
- `data/` — canonical input CSVs. `tests/` — pytest suite. `outputs/` — generated HTML (gitignored).

## MATLAB → Python mapping

| MATLAB source | Python target |
| --- | --- |
| `sussy_steer.m` | `kinematics/steer.py` + `kinematics/solver.py` |
| `sussy_shock_only.m` | `kinematics/shock_only.py` |
| `sussy_optimize.m` | `kinematics/optimize.py` (TODO) |
| `sussy_tie_on_arm.m` | `kinematics/tie_on_arm.py` (TODO) |
| `solve_forces.m` | `forces/solve_forces.py` |
| `run_quasistatic.m` | `forces/run_quasistatic.py` |
| `visualize_forces.m` | `plotting/forces3d.py` |
| `print_axle.m` (parsing) | `io.py` |

## Conventions

- **Units preserved**: inches, pounds (lbf), in-lbs; angles in degrees at the
  results boundary, radians internally.
- **Stack**: NumPy (linear algebra) + Plotly (viz). No pandas/scipy/matplotlib.
- **Data structures**: dataclasses mirror MATLAB structs (`Config`, `Geometry`,
  `Forces`, `KinematicResults`).
- **Array shapes match MATLAB**: scalars `(nSteer, nShock)`, points
  `(3, nSteer, nShock)`.
- **Porting**: one MATLAB function at a time; verify numerically against the
  reference. Preserve sign conventions and behavior 1:1 unless documented.
- **Headless**: figures are written to `outputs/*.html` with
  `include_plotlyjs=True` (self-contained). Never open a GUI window.

## Environment

Python 3.11+ (sandbox runs 3.14); venv at `.venv/` (gitignored). Review is via
VS-Codium over SSH — prefer browser-viewable HTML output.
