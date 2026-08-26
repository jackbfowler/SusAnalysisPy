# SussyAnal — Python Refactor Architecture

Python port of the Baja SAE suspension kinematics + force analysis MATLAB
tooling in `SusAnalysis/` (reference only, gitignored).

## 1. Goals

- Faithful, tested port of the MATLAB suspension **model**, **solver**, and
  **visualizer** into an installable `src/sussyanal` package.
- Centralize logic that MATLAB duplicated across four scripts (CSV parsing,
  geometry setup, and the kinematic solver each appear ~4×).
- Replace MATLAB `figure`/`uicontrol` interactive plots with **Plotly** HTML
  that works in the SSH/VS-Codium headless sandbox.

## 2. MATLAB source map

| MATLAB | Purpose |
| --- | --- |
| `SussyAnal/sussy_steer.m` | **Canonical solver**: 2D sweep (shock × steer), envelope, Ackermann, interactive 3D + sliders |
| `SussyAnal/sussy_shock_only.m` | 1D solver (shock only), slightly different toe definition |
| `SussyAnal/sussy_optimize.m` | Hardpoint grid-search optimizer (reuses solver) |
| `SussyAnal/sussy_tie_on_arm.m` | Tie-rod-on-LCA mount optimizer |
| `SussyForces/solve_forces.m` | 7×7 linear system force solver |
| `SussyForces/run_quasistatic.m` | Driver: WFT loads → geometry → forces → viz |
| `SussyForces/visualize_forces.m` | 3D force vector visualization |
| `print_axle.m` | CSV parsing helper |

## 3. Python architecture

```
src/sussyanal/
├── io.py                  # CSV hardpoint/config parsing -> (Geometry, Config)
├── geometry.py            # math helpers + dataclasses + SuspensionModel.solve_step()
├── kinematics/
│   ├── solver.py          # KinematicResults + solve_sweep() + post-processing + Ackermann
│   ├── steer.py           # analyze()  ~ sussy_steer.m   (2D/1D sweep + plots)
│   ├── shock_only.py      # analyze()  ~ sussy_shock_only.m (1D curves)
│   ├── optimize.py        # ~ sussy_optimize.m  (grid search)      [next]
│   └── tie_on_arm.py      # ~ sussy_tie_on_arm.m (mount optimizer) [next]
├── forces/
│   ├── solve_forces.py    # ~ solve_forces.m
│   └── run_quasistatic.py # ~ run_quasistatic.m
├── plotting/
│   ├── common.py          # colors, layout, wheel/arrow/cone builders
│   ├── kinematics.py      # curve + surface figures
│   ├── suspension3d.py    # interactive 3D model (slider/playback)
│   └── forces3d.py        # 3D force vectors
└── __main__.py            # CLI: python -m sussyanal analyze|forces <csv>
```

### Key abstractions

- `Config` — bump, droop, wheel_size, steer_sweep, shock_mount_lca, wheelbase.
- `Geometry` — the named 3-D hardpoints (dataclass, optional axle points).
- `Arm` — a wishbone (front/rear pivots + outer ball joint) with derived
  `axis`, `origin`, `center`, `radius`.
- `ShockGeom`, `Upright`, `TieRod`, `Axle` — pre-computed static geometry.
- `SuspensionModel` — builds the above from `(Geometry, Config)` and exposes
  `solve_step(shock_length, rack_position) -> StepGeometry`.
- `KinematicResults` — pre-allocated arrays (shapes match MATLAB: scalars
  `(nSteer, nShock)`, points `(3, nSteer, nShock)`).

### Solver algorithm (from `sussy_steer.m`)

1. **Primary arm** (shock-driven): law-of-cosines on the shock triangle gives
   arm rotation `Δφ`; sign is corrected so compression lifts the wheel.
   `Rodrigues` rotation moves the mounted arm + shock lower point.
2. **Secondary arm** (upright-driven): intersect the driven arm's outer ball
   sphere (radius = upright height) with the other arm's rotation circle
   (`intersect_circle_sphere`), choosing the physically correct branch.
3. **Upright**: rebuild the kingpin basis; rotate the upright by the tie-rod
   angle difference; reconstruct spindle/wheel-center/axle in the kingpin frame.
4. **Derive**: camber, caster, KPI, toe, scrub, trail, plunge, CV angles,
   arm articulation angles; then motion ratio / wheel rate / track change.

## 4. Visualization strategy — Plotly

**Decision: Plotly (primary).** Rationale for the SSH/VS-Codium headless setup:

- Emits **self-contained HTML** (`fig.write_html(..., include_plotlyjs=True)`)
  — viewable in a browser with no display server, no port, no `kaleido`.
- Native **3D** (`Scatter3d`, `Line3d`, `Mesh3d`, `Cone`) reproduces MATLAB
  `plot3`/`surf`/`fill3`/`quiver3`.
- Native **sliders + frames** replace MATLAB `uicontrol` sliders and the
  "Play Bump/Droop" buttons.
- Interactive hover/rotate/zoom is strictly better than static screenshots.

Mapping of MATLAB figures:

| MATLAB | Plotly |
| --- | --- |
| `create_plots` (1D curves) | `plotting.kinematics.curve_figure()` subplots |
| `create_plots` (2D surfaces) | `go.Surface` |
| `create_vis` (interactive 3D + sliders) | `plotting.suspension3d.figure()` with frames/slider |
| `visualize_forces` (vectors + toggles) | `plotting.forces3d.figure()` with legend-only traces |
| moment arc / arrows | `go.Scatter3d` arc + `go.Cone` arrowheads |

Headless policy: always write figures to `outputs/*.html` (gitignored). A
static-PNG fallback (matplotlib) is intentionally **not** a dependency.

## 5. Dependencies

Minimal by design: **numpy** (linear algebra) and **plotly** (viz). No
pandas/scipy/matplotlib are required — CSV parsing uses the stdlib `csv`
module, and the solver is closed-form (no optimizer needed for the core).

## 6. Verification plan

1. Unit-test `intersect_circle_sphere` / `rotate_point` against hand cases.
2. Numerical parity: run the port on `data/*.csv` and compare camber/toe/
   plunge tables against the reference MATLAB outputs (same units: in, lbf,
   deg).
3. Force solver parity: `solve_forces` residual check (`A x = b`) and the
   MATLAB "Equilibrium Verified" residual < 1e-3.
4. Smoke-test `python -m sussyanal analyze data/<file>.csv` writes HTML.

## 7. Roadmap

- [x] Repo scaffold, package skeleton, CSV `io`, geometry model
- [x] Kinematic solver (`sussy_steer` + `sussy_shock_only`) + Plotly plots
- [x] Force solver + quasistatic driver + force visualizer
- [ ] `optimize.py` (grid search) and `tie_on_arm.py` (mount optimizer)
- [ ] pytest suite with numerical parity checks
