# agents.md

Guidance for AI coding agents working in this repository. See
[ARCHITECTURE.md](ARCHITECTURE.md) for the full design and roadmap.

## Project

SussyAnal is a suspension kinematics and force analysis tool for Baja SAE
vehicles. This repository is a Python port of the core MATLAB implementation in
`SusAnalysis/` (reference only, gitignored — never edit or depend on it).

**Scope**: the `sussy_steer` kinematics analyzer and the quasistatic force
solver. `sussy_shock_only` is deprecated and `sussy_tie_on_arm` is out of
scope — neither is ported.

## Repository layout

- `src/sussyanal/io.py` — CSV hardpoint/config parsing → `SuspensionData`.
- `src/sussyanal/geometry.py` — math helpers, dataclasses, `SuspensionModel`.
- `src/sussyanal/kinematics/solver.py` — `solve_sweep()` + `KinematicResults`.
- `src/sussyanal/kinematics/steer.py` — the `sussy_steer` analyzer.
- `src/sussyanal/kinematics/optimize.py` — hardpoint grid-search optimizer.
- `src/sussyanal/forces/` — `solve_forces.py` + `run_quasistatic.py`.
- `src/sussyanal/plotting/` — Plotly figures (envelope, surfaces, 3-D, forces).
- `tools/points_to_csv.py` — converts mm hardpoint exports (plain rows or
  `(N) Point N: Name` blocks) into `io.py`-compatible inch CSVs.
- `data/` — canonical input CSVs. `tests/` — pytest suite. `outputs/` — generated HTML (gitignored).

## MATLAB → Python mapping

| MATLAB source | Python target |
| --- | --- |
| `sussy_steer.m` | `kinematics/steer.py` + `kinematics/solver.py` |
| `sussy_optimize.m` | `kinematics/optimize.py` (grid method) |
| `solve_forces.m` | `forces/solve_forces.py` |
| `run_quasistatic.m` | `forces/run_quasistatic.py` |
| `visualize_forces.m` | `plotting/forces3d.py` |
| `print_axle.m` (parsing) | `io.py` |

## Configuration is per hardpoint set

`bump`, `droop`, `wheel_size`, `steer_sweep`, `shock_mount_lca`, `wheelbase`
are parsed from each set's own CSV and bundled with its geometry via
`SuspensionData(geometry, config)`. **Never** introduce a shared/global config;
always build models with `SuspensionModel.from_data(data)`. Fields the CSV
omits keep documented fallback defaults for that set only, tracked in
`Config.missing`.

**A note on `XXXX` placeholders**: newly generated CSVs may ship config values
as `XXXX` (from `tools/points_to_csv.py`). `parse_csv` will raise
`ValueError: could not convert string to float: 'XXXX'` until real numbers are
filled in — that is expected for "fill in later" placeholders.

## Conventions

- **Units preserved**: inches, pounds (lbf), in-lbs; angles in degrees at the
  results boundary, radians internally.
- **Stack**: NumPy (linear algebra) + Plotly (viz). No pandas/scipy/matplotlib.
- **Data structures**: dataclasses mirror MATLAB structs (`Config`,
  `Geometry`, `SuspensionData`, `Forces`, `KinematicResults`).
- **Array shapes match MATLAB**: scalars `(nSteer, nShock)`, points
  `(3, nSteer, nShock)`.
- **Porting**: one MATLAB function at a time; verify numerically against the
  reference. Preserve sign conventions and behavior 1:1 unless documented.
- **Headless**: figures are written to `outputs/*.html` with
  `include_plotlyjs=True` (self-contained). Never open a GUI window.

## Plotting: one envelope renderer

All envelope-style output — 2-D static, 1-D static, the live overlay below the
3-D graph, and the component figure — goes through the single
`envelope_figure()` in `plotting/kinematics.py`, parameterized by `live`,
`metrics`, `colorbar`, `title`. 1-D sets (no steering sweep) render one line
per metric; 2-D sets render a colorcoded steering family. Do **not** add a
parallel envelope/curve implementation for a new mode — extend
`envelope_figure()` instead.

The live 3-D page (`page.py`) links the viewer's sliders to the envelope with
in-page JS: `Plotly.relayout` moves one **vertical-line shape per subplot** to
mark current shock travel (shapes are layout-level and robust to trace
reindexing), and `Plotly.restyle` swaps the bold current-steer line. Keep it
that way — do not reintroduce per-trace marker restyles for the indicator.

## Environment

Python 3.11+ (sandbox runs 3.14); venv at `.venv/` (gitignored). The venv's
launcher scripts embed absolute paths — if the repo moves, recreate the venv
(`python3 -m venv .venv && .venv/bin/pip install -e .`). The workspace is a
virtiofs share from a macOS host; transient `EPERM` on file writes (Spotlight /
Finder coordination) is handled by retry + temp-file-rename in
`src/sussyanal/__main__.py._atomic_write`. Review is via VS-Codium over SSH —
prefer browser-viewable HTML output.
