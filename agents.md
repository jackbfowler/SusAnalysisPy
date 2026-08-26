# agents.md

Guidance for AI coding agents working in this repository.

## Project

SussyAnal is a suspension kinematics and force analysis tool for Baja SAE
vehicles. This repository is a Python port of the original MATLAB
implementation.

## Goal

Refactor the MATLAB scripts into a clean, tested Python package. The MATLAB
sources are **reference only**: they are gitignored and must not be edited,
and the Python port must not import or depend on them at runtime.

## Repository layout

- `src/sussyanal/` — the Python package. **All new Python code goes here.**
- `src/sussyanal/kinematics/` — suspension geometry / kinematics analysis
  (ports of `SusAnalysis/SussyAnal/*.m`).
- `src/sussyanal/forces/` — quasistatic force analysis
  (ports of `SusAnalysis/SussyForces/*.m`).
- `src/sussyanal/io.py` — shared CSV hardpoint/config parsing.
- `data/` — canonical input CSVs (hardpoint geometry), copied from the
  reference so the project is self-contained.
- `tests/` — pytest suite.
- `SusAnalysis/` — original MATLAB code and data. **Reference only**,
  gitignored. Do not edit, and do not treat as a runtime dependency.

## MATLAB → Python mapping

| MATLAB source | Python target |
| --- | --- |
| `SusAnalysis/SussyAnal/sussy_steer.m` | `src/sussyanal/kinematics/steer.py` |
| `SusAnalysis/SussyAnal/sussy_shock_only.m` | `src/sussyanal/kinematics/shock_only.py` |
| `SusAnalysis/SussyAnal/sussy_optimize.m` | `src/sussyanal/kinematics/optimize.py` |
| `SusAnalysis/SussyAnal/sussy_tie_on_arm.m` | `src/sussyanal/kinematics/tie_on_arm.py` |
| `SusAnalysis/SussyForces/solve_forces.m` | `src/sussyanal/forces/solve_forces.py` |
| `SusAnalysis/SussyForces/run_quasistatic.m` | `src/sussyanal/forces/run_quasistatic.py` |
| `SusAnalysis/SussyForces/visualize_forces.m` | `src/sussyanal/forces/visualize_forces.py` |
| `SusAnalysis/print_axle.m` (parsing logic) | `src/sussyanal/io.py` |

## Conventions

- **Units are preserved from MATLAB**: lengths in inches, forces in pounds
  (lbf), moments in in-lbs. Do not convert.
- **Stack**: NumPy for linear algebra, SciPy for numerical routines,
  pandas for CSV I/O, Matplotlib for plotting.
- **Data structures**: MATLAB structs map to `dataclasses`
  (e.g. `Geometry`, `Config`, `Loads`, `Forces`).
- **Sign conventions**: keep the documented wheel-frame and global-frame
  conventions exactly as in the MATLAB sources.
- **Packaging**: `src` layout, installed via `pip install -e ".[dev]"`.
  Python 3.11+ (environment currently runs 3.14).
- **Porting process**: port one MATLAB function at a time; verify numerical
  outputs against the reference before moving on. Preserve 1:1 behavior unless
  a deviation is explicitly justified and documented.
- **Naming**: snake_case module/function names; keep a 1:1 correspondence to
  the MATLAB source so code review (VS-Codium over SSH) can compare easily.

## Environment

- Virtualenv at `.venv/` (gitignored). Dependencies are declared in
  `pyproject.toml` and installed on demand.
- Review is done through VS-Codium over an SSH connection into the sandbox;
  avoid heavy interactive GUIs unless explicitly requested. Prefer headless
  (save figures to file) or `matplotlib` Agg backend.
