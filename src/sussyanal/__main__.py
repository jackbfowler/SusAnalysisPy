"""CLI entry point: ``python -m sussyanal analyze|forces <csv>``."""
from __future__ import annotations

import argparse
from pathlib import Path

from .forces import run_quasistatic, report
from .kinematics import analyze_steer
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
        fig = kin_plot.surfaces_figure(results)
        _write(fig, Path(args.out_dir), "kinematics_surfaces.html")
    else:
        fig = kin_plot.curve_figure(results)
        _write(fig, Path(args.out_dir), "kinematics_curves.html")

    model = _model(results)
    _write(
        suspension3d.suspension_figure(model, results),
        Path(args.out_dir),
        "suspension3d.html",
    )
    return 0


def _model(results):
    from .geometry import SuspensionModel

    return SuspensionModel(results.geometry, results.config)


def _cmd_forces(args) -> int:
    result = run_quasistatic(args.csv)
    print(report(result))
    _write(forces3d.forces_figure(result), Path(args.out_dir), "forces3d.html")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="sussyanal", description="Baja suspension analysis")
    sub = parser.add_subparsers(dest="cmd", required=True)

    a = sub.add_parser("analyze", help="Run kinematic sweep + plots")
    a.add_argument("csv", help="Suspension CSV file")
    a.add_argument("--n-shock", type=int, default=100)
    a.add_argument("--out-dir", default="outputs")
    a.set_defaults(fn=_cmd_analyze)

    f = sub.add_parser("forces", help="Run quasistatic force analysis + plot")
    f.add_argument("csv", help="Suspension CSV file")
    f.add_argument("--out-dir", default="outputs")
    f.set_defaults(fn=_cmd_forces)

    args = parser.parse_args(argv)
    return args.fn(args)


if __name__ == "__main__":
    raise SystemExit(main())
