"""CLI entry point: ``python -m sussyanal analyze|forces|optimize <csv>``."""
from __future__ import annotations

import argparse
import os
import time
from pathlib import Path

from .forces import run_quasistatic, report
from .kinematics import (
    analyze_steer,
    optimization_figure,
    optimize,
    report as optimize_report,
)
from .plotting import forces3d, kinematics as kin_plot, page as page_module, suspension3d

_WRITE_ATTEMPTS = 8
_WRITE_DELAY_S = 1.0


def _atomic_write(fn, path: Path) -> None:
    """Write ``path`` via a temp file + rename, retrying transient EPERM.

    The workspace sits on a virtiofs share from a macOS host; while the host
    briefly touches a file (Spotlight indexing, Finder, file coordination) the
    guest sees ``EPERM: Operation not permitted`` instead of a normal error.
    Writing to a fresh temp name and renaming over the target avoids opening
    the host-locked file, and the retry loop rides out any residual transient.
    """
    out_dir = path.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    tmp = out_dir / f".{path.name}.tmp{os.getpid()}"
    last_err: OSError | None = None
    for attempt in range(_WRITE_ATTEMPTS):
        try:
            fn(tmp)
            os.replace(tmp, path)
            return
        except PermissionError as e:
            last_err = e
            time.sleep(_WRITE_DELAY_S * (attempt + 1))
    if last_err is not None:
        raise last_err


def _write(fig, out_dir: Path, name: str) -> Path:
    path = out_dir / name
    _atomic_write(
        lambda tmp: fig.write_html(tmp, include_plotlyjs=True), path
    )
    print(f"Wrote {path}")
    return path


def _write_page(html: str, out_dir: Path, name: str) -> Path:
    path = out_dir / name
    _atomic_write(
        lambda tmp: tmp.write_text(html, encoding="utf-8"), path
    )
    print(f"Wrote {path}")
    return path


def _cmd_analyze(args) -> int:
    results = analyze_steer(args.csv, n_shock_steps=args.n_shock, progress=True)
    stem = Path(args.csv).stem
    out_dir = Path(args.out_dir)

    # static component-analysis plot (axle plunge, CV, arm articulation angles)
    _write(kin_plot.component_figure(results, title=stem), out_dir,
           f"{stem}_kinematics_component.html")

    from .geometry import SuspensionModel

    model = SuspensionModel(results.geometry, results.config)

    if results.is_2d:
        # static envelope output (no moving parts); standalone keeps the
        # steering colorbar as its legend
        _write(kin_plot.envelope_figure(results), out_dir,
               f"{stem}_kinematics_envelope.html")
        if args.surfaces:
            _write(kin_plot.surfaces_figure(results), out_dir,
                   f"{stem}_kinematics_surfaces.html")
    else:
        # 1-D (zero-steer) static output: same envelope figure as the live
        # overlay below the 3-D graph (3x3 grid, shared formatting), just
        # without the moving red dot — no separate curve implementation.
        _write(kin_plot.envelope_figure(results), out_dir,
               f"{stem}_kinematics_curves.html")

    # combined page for BOTH modes: 3-D viewer fills the top (CSV title), live
    # envelope below (no steering colorbar; 1-D sets show no steering lines)
    live_env = kin_plot.envelope_figure(results, live=True)
    shock_idxs, mid_steer, mid_shock = suspension3d.viewer_indices(results)
    page = page_module.analyze_page(
        suspension3d.suspension_figure(model, results, title=stem),
        live_env,
        live_env._envelope_config,
        shock_idxs,
        mid_steer,
        mid_shock,
    )
    _write_page(page, out_dir, f"{stem}_suspension3d.html")
    return 0


def _cmd_forces(args) -> int:
    result = run_quasistatic(args.csv)
    print(report(result))
    _write(forces3d.forces_figure(result), Path(args.out_dir), "forces3d.html")
    return 0


def _cmd_optimize(args) -> int:
    res = optimize(
        args.csv,
        opt_point=args.point,
        sweep_axes=[int(a) for a in args.axes.split(",")],
        sweep_range=[float(r) for r in args.range.split(",")],
        sweep_steps=[int(s) for s in args.steps.split(",")],
        objective=args.objective,
        n_shock_steps=args.n_shock,
        progress=True,
    )
    print(optimize_report(res))
    _write(optimization_figure(res), Path(args.out_dir), "optimization.html")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="sussyanal", description="Baja suspension analysis")
    sub = parser.add_subparsers(dest="cmd", required=True)

    a = sub.add_parser("analyze", help="Run kinematic sweep + plots (sussy_steer)")
    a.add_argument("csv")
    a.add_argument("--n-shock", type=int, default=200,
                   help="shock steps (2-D grid resolution; default 200)")
    a.add_argument("--surfaces", action="store_true",
                   help="also write 2-D surface plots (off by default)")
    a.add_argument("--out-dir", default="outputs")
    a.set_defaults(fn=_cmd_analyze)

    f = sub.add_parser("forces", help="Run quasistatic force analysis + plot")
    f.add_argument("csv")
    f.add_argument("--out-dir", default="outputs")
    f.set_defaults(fn=_cmd_forces)

    o = sub.add_parser("optimize", help="Hardpoint grid-search optimizer")
    o.add_argument("csv")
    o.add_argument("--point", default="OuterTrackRodBallJoint")
    o.add_argument("--axes", default="3", help="comma-separated 1/2/3 (X/Y/Z)")
    o.add_argument("--range", default="1.0", help="comma-separated +/- sweep (in)")
    o.add_argument("--steps", default="50", help="comma-separated step counts")
    o.add_argument("--objective", default="bump_steer",
                   choices=("plunge_range", "plunge_max", "bump_steer", "camber_gain"))
    o.add_argument("--n-shock", type=int, default=30)
    o.add_argument("--out-dir", default="outputs")
    o.set_defaults(fn=_cmd_optimize)

    args = parser.parse_args(argv)
    return args.fn(args)


if __name__ == "__main__":
    raise SystemExit(main())
