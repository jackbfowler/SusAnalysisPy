# SussyAnal — Python Refactor Architecture

Python port of the Baja SAE suspension kinematics + force analysis MATLAB
tooling in `SusAnalysis/` (reference only, gitignored). Scope: the **core
`sussy_steer`** kinematics analyzer and the **quasistatic force solver**.
The deprecated `sussy_shock_only` and the out-of-scope `sussy_tie_on_arm` are
**not** ported.

## 1. Goals

- Faithful, tested port of the `sussy_steer` suspension **model** + **solver**
  and the **force solver**, with Plotly visualization.
- **Per-hardpoint-set configuration**: `bump`, `droop`, `wheel_size`,
  `steer_sweep`, `shock_mount_lca`, `wheelbase` are read from each set's own
  CSV and bundled with its geometry (`SuspensionData`) — never shared across
  sets. Parameters a CSV omits keep documented fallback defaults *for that set
  only*, tracked in `Config.missing`.
- Centralize logic MATLAB duplicated across scripts (CSV parsing, geometry
  setup, solver each appear in several files).

## 2. MATLAB source map

| MATLAB | Status |
| --- | --- |
| `SussyAnal/sussy_steer.m` | ported — kinematics solver + plots |
| `SussyAnal/sussy_shock_only.m` | deprecated, not ported |
| `SussyAnal/sussy_optimize.m` | ported (grid method) |
| `SussyAnal/sussy_tie_on_arm.m` | out of scope, not ported |
| `SussyForces/solve_forces.m` | ported |
| `SussyForces/run_quasistatic.m` | ported |
| `SussyForces/visualize_forces.m` | ported → `plotting/forces3d.py` |
| `print_axle.m` | parsing logic covered by `io.py` |

## 3. Python architecture

```
src/sussyanal/
├── io.py                  # CSV parsing -> SuspensionData (hardpoints + config)
├── geometry.py            # math helpers, dataclasses, SuspensionModel.solve_step()
├── kinematics/
│   ├── solver.py          # KinematicResults + solve_sweep() + post-processing + Ackermann
│   ├── steer.py           # analyze() ~ sussy_steer.m
│   └── optimize.py        # ~ sussy_optimize.m (grid search)
├── forces/
│   ├── solve_forces.py    # ~ solve_forces.m
│   └── run_quasistatic.py # ~ run_quasistatic.m
├── plotting/
│   ├── common.py          # colors, layout, wheel/arrow builders
│   ├── kinematics.py      # curve + surface + envelope figures
│   ├── suspension3d.py    # interactive 3-D model (shock + steering sliders)
│   ├── forces3d.py        # 3-D force vectors
│   └── page.py            # single-page analyze output (viewer + live envelope)
└── __main__.py            # CLI: python -m sussyanal analyze|forces|optimize <csv>
```

### Configuration model

Each CSV ships its own configuration rows (e.g. `Shock bump from ride`,
`Steering rack`, `Shock lower mount`, `Wheelbase`). `parse_csv` returns a
`SuspensionData(geometry, config)` so a hardpoint set and its parameters always
travel together; `SuspensionModel.from_data()` consumes it. `Config.missing`
records which parameters the set's CSV did not define (fallback defaults
active for that set only).

### Key abstractions

- `SuspensionData` — one hardpoint set + its own `Config`.
- `Geometry` — the named 3-D hardpoints (optional axle points).
- `Arm` — a wishbone (front/rear pivots + outer ball joint) with derived
  `axis`, `origin`, `center`, `radius`.
- `ShockGeom`, `Upright`, `TieRod`, `Axle` — pre-computed static geometry.
- `SuspensionModel` — `solve_step(shock_length, rack_position) -> StepGeometry`
  and `solve_arms(shock_length)`.
- `KinematicResults` — pre-allocated arrays (scalars `(nSteer, nShock)`,
  points `(3, nSteer, nShock)`).

### Solver algorithm (from `sussy_steer.m`)

1. **Primary arm** (shock-driven): law-of-cosines on the shock triangle gives
   arm rotation `Δφ`; sign corrected so compression lifts the wheel. Rodrigues
   rotation moves the mounted arm + shock lower point.
2. **Secondary arm** (upright-driven): intersect the driven arm's outer ball
   sphere (radius = upright height) with the other arm's rotation circle
   (`intersect_circle_sphere`), choosing the physically correct branch.
3. **Upright**: rebuild the kingpin basis; rotate the upright by the tie-rod
   angle difference; reconstruct spindle / wheel-center / axle in the kingpin
   frame.
4. **Derive**: camber, caster, KPI, toe, scrub, trail, plunge, CV angles,
   arm articulation angles; then motion ratio / wheel rate / track change,
   Ackermann %.

## 4. Visualization strategy — Plotly

**Decision: Plotly (primary).** Rationale for the SSH/VS-Codium headless setup:

- Emits **self-contained HTML** (`fig.write_html(..., include_plotlyjs=True)`)
  — viewable in a browser with no display server.
- Native **3-D** (`Scatter3d`, `Line3d`, `Mesh3d`, `Cone`) reproduces MATLAB
  `plot3`/`surf`/`fill3`/`quiver3`.
- Native **sliders + frames** replace MATLAB `uicontrol` sliders (no play
  buttons; surfaces are opt-in via `--surfaces`, matching MATLAB's
  `show_surface_plots=false` default).

| MATLAB | Plotly |
| --- | --- |
| `create_plots` 1-D curves | `plotting.kinematics.curve_figure()` |
| `create_plots` 2-D surfaces | `plotting.kinematics.surfaces_figure()` |
| `create_plots` envelope (`do_envelope`) | `plotting.kinematics.envelope_figure()` |
| `create_vis` interactive 3-D + sliders | `plotting.suspension3d.suspension_figure()` (shock **and** steering sliders) |
| `visualize_forces` vectors | `plotting.forces3d.forces_figure()` |
| `sussy_optimize` cost maps | `kinematics.optimize.optimization_figure()` |

Plotly animates a single frame axis, so the interactive 3-D figure uses two
sliders with a compact frame set: the **shock slider** sweeps all shock steps
at static steer and the **steering slider** sweeps all steer steps at static
ride height (dragging one snaps the other to static). The full 2-D coupling is
shown by the surface and envelope plots (one line per steering step vs shock
travel).

### In-page envelope linkage (`plotting/page.py`)

For 2-D sets, `suspension3d.html` is **one page** containing the 3-D viewer
(top, 72vh, sliders included) with the live envelope below — scroll down to
view it. Small in-page JS hooks the viewer's slider events and restyles the
envelope's overlay traces, exactly like the MATLAB visualizer: the **steering
slider switches which gray line is highlighted** (bold blue), the **shock
slider moves the red current-point marker** along it, and the min/max markers
follow the current line. No cross-tab machinery or server is required — the
page works from `file://`. A static standalone envelope
(`kinematics_envelope.html`, no moving parts) is written alongside.

Headless policy: figures are written to `outputs/*.html` (gitignored).

## 5. Dependencies

Minimal by design: **numpy** (linear algebra) and **plotly** (viz). No
pandas/scipy/matplotlib required — CSV parsing uses the stdlib `csv` module,
and the solver is closed-form.

## 6. Verification plan

1. Unit-test `intersect_circle_sphere` / `rotate_point` against hand cases.
2. Per-set config: assert each CSV's own bump/droop/steer_sweep/... are used
   and `Config.missing` tracks omitted rows.
3. Force solver parity: residual check < 1e-3 (equilibrium).
4. Smoke-test `python -m sussyanal analyze data/<file>.csv` writes HTML.

## 7. Roadmap

- [x] Core solver (`sussy_steer`) + forces + Plotly plots + grid optimizer
- [x] Per-hardpoint-set configuration (`SuspensionData`)
- [ ] pytest numerical parity checks against exported MATLAB reference outputs
