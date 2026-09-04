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

- `src/sussyanal/io.py` — shared-format (`"name" = value` SolidWorks txt) parsing
  → `SuspensionData`; legacy CSV parsing deprecated (warns, kept for migration).
- `src/sussyanal/geometry.py` — math helpers, dataclasses, `SuspensionModel`.
- `src/sussyanal/kinematics/solver.py` — `solve_sweep()` + `KinematicResults`.
- `src/sussyanal/kinematics/steer.py` — the `sussy_steer` analyzer.
- `src/sussyanal/kinematics/optimize.py` — hardpoint grid-search optimizer.
- `src/sussyanal/forces/` — `solve_forces.py` + `run_quasistatic.py`.
- `src/sussyanal/plotting/` — Plotly figures (envelope, surfaces, 3-D, forces).
- `tools/points_to_csv.py` — imports ONE LOTUS mm corner export (plain rows or
  `(N) Point N: Name` blocks) into the shared `datanew/*.txt` (`--axle
  front|rear`; merges into an existing file, leaving the other corner blank).
- `tools/csv_to_shared.py` — migrates a legacy front+rear `data/*.csv` pair
  into one shared `datanew/*.txt`.
- `datanew/` — canonical shared inputs (front+rear per year/version in one
  file). `data/` — legacy per-corner CSVs (deprecated). `tests/` — pytest
  suite. `outputs/` — generated HTML (gitignored).

## MATLAB → Python mapping

| MATLAB source | Python target |
| --- | --- |
| `sussy_steer.m` | `kinematics/steer.py` + `kinematics/solver.py` |
| `sussy_optimize.m` | `kinematics/optimize.py` (grid method) |
| `solve_forces.m` | `forces/solve_forces.py` |
| `run_quasistatic.m` | `forces/run_quasistatic.py` |
| `visualize_forces.m` | `plotting/forces3d.py` |
| `print_axle.m` (parsing) | `io.py` |

## Configuration is per hardpoint set / corner

`bump`, `droop`, `wheel_size`, `steer_sweep`, `shock_mount_lca` (and legacy
`wheelbase`) belong to the hardpoint set and are bundled with its geometry via
`SuspensionData(geometry, config)`. Shared `datanew/*.txt` files hold BOTH
axles; `io.parse_csv(path, axle="front"|"rear")` parses ONE corner — front and
rear simulations stay separate, chosen via `--axle` at the CLI. `steering_rack`
is the shared front rack (rear parses get `steer_sweep = 0`), and per-axle
values carry `_front`/`_rear` suffixes. **Never** introduce a shared/global
config; always build models with `SuspensionModel.from_data(data)`. Fields the
file omits keep documented fallback defaults for that corner only, tracked in
`Config.missing`.

**A note on `XXXX` / blank config**: legacy CSVs may ship config values as
`XXXX` (from the old importer) — `parse_csv` raises
`ValueError: could not convert string to float: 'XXXX'` until real numbers are
filled in. Shared files created by `tools/points_to_csv.py` leave config lines
blank (a LOTUS export has no config); parsing then silently falls back to
defaults with the fields listed in `Config.missing` — fill the blanks before
running a real analysis.

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
