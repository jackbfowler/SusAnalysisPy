"""CLI entry point: ``python -m sussyanal analyze|forces|optimize <csv>``."""
from __future__ import annotations

import argparse
from pathlib import Path

from .forces import run_quasistatic, report
from .kinematics import (
    analyze_steer,
    optimization_figure,
    optimize,
    report as optimize_report,
)
from .plotting import forces3d, kinematics as kin_plot, suspension3d


def _write(fig, out_dir: Path, name: str) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / name
    fig.write_html(path, include_plotlyjs=True)
    print(f"Wrote {path}")
    return path


def _cmd_analyze(args) -> int:
    results = analyze_steer(args.csv, n_shock_steps=args.n_shock, progress=True)
    if results.is_2d:
        # envelope is the default for 2-D; surfaces are opt-in (MATLAB
        # show_surface_plots defaults to false)
        _write(kin_plot.envelope_figure(results), Path(args.out_dir), "kinematics_envelope.html")
        if args.surfaces:
            _write(kin_plot.surfaces_figure(results), Path(args.out_dir), "kinematics_surfaces.html")
    else:
        _write(kin_plot.curve_figure(results), Path(args.out_dir), "kinematics_curves.html")

    from .geometry import SuspensionModel

    model = SuspensionModel(results.geometry, results.config)
    _write(
        suspension3d.suspension_figure(model, results),
        Path(args.out_dir),
        "suspension3d.html",
    )
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
    a.add_argument("--n-shock", type=int, default=100)
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
