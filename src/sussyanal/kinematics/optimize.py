"""Hardpoint grid-search optimizer (port of ``sussy_optimize.m``).

Implements the ``grid`` method: sweep a hardpoint over one or more axes and
re-run the kinematic sweep, minimizing an objective. The MATLAB ``geometry``
(sphere-fit) method is not ported.
"""
from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, replace

import numpy as np
import plotly.graph_objects as go

from ..geometry import Geometry, SuspensionModel
from ..io import _camel_to_snake, parse_csv
from .solver import KinematicResults, solve_sweep

_OBJECTIVES = ("plunge_range", "plunge_max", "bump_steer", "camber_gain")


@dataclass
class OptimizationResult:
    opt_point: str
    objective: str
    sweep_axes: list[int]
    base_pos: np.ndarray
    opt_pos: np.ndarray
    base_cost: float
    opt_cost: float
    axis_coords: list[np.ndarray]  # one 1-D array per swept axis
    cost: np.ndarray               # cost grid shaped (n0, n1, ...)


def _objective_cost(res: KinematicResults, objective: str) -> float:
    if objective == "plunge_range":
        if res.plunge is None:
            return float("nan")
        return float(np.max(res.plunge) - np.min(res.plunge))
    if objective == "plunge_max":
        if res.plunge is None:
            return float("nan")
        return float(np.max(np.abs(res.plunge)))
    if objective == "bump_steer":
        return float(np.max(res.toe) - np.min(res.toe))
    if objective == "camber_gain":
        n = res.camber.shape[1]
        mid = res.camber[0, math.ceil(n / 2) - 1]
        return float(np.max(np.abs(res.camber - mid)))
    return float("nan")


def _resolve_field(opt_point: str) -> str:
    if opt_point in Geometry.__dataclass_fields__:
        return opt_point
    snake = _camel_to_snake(opt_point)
    if snake in Geometry.__dataclass_fields__:
        return snake
    raise ValueError(f"Optimization point {opt_point!r} is not a geometry hardpoint")


def optimize(
    csv,
    opt_point: str = "OuterTrackRodBallJoint",
    sweep_axes=(3,),
    sweep_range=(1.0,),
    sweep_steps=(50,),
    objective: str = "bump_steer",
    n_shock_steps: int = 30,
    progress: bool = False,
) -> OptimizationResult:
    """Grid-search ``opt_point`` over ``sweep_axes`` to minimize ``objective``."""
    if objective not in _OBJECTIVES:
        raise ValueError(f"objective must be one of {_OBJECTIVES}")

    geometry, config = parse_csv(csv)
    field = _resolve_field(opt_point)
    base_pos = np.asarray(getattr(geometry, field), dtype=float).copy()

    axis_coords = []
    for axis, rng, steps in zip(sweep_axes, sweep_range, sweep_steps):
        axis_coords.append(np.linspace(base_pos[axis - 1] - rng, base_pos[axis - 1] + rng, steps))

    shape = [len(c) for c in axis_coords]
    cost = np.full(shape, np.nan)
    opt_pos = base_pos.copy()
    opt_cost = float("inf")

    total = int(np.prod(shape))
    done = 0
    for inds in itertools.product(*[range(s) for s in shape]):
        new_pt = base_pos.copy()
        for k, ind in enumerate(inds):
            new_pt[sweep_axes[k] - 1] = axis_coords[k][ind]

        cur_geom = replace(geometry, **{field: new_pt})
        model = SuspensionModel(cur_geom, config)
        res = solve_sweep(model, n_shock_steps=n_shock_steps, n_steer_steps=1)
        c = _objective_cost(res, objective)
        cost[inds] = c
        if np.isfinite(c) and c < opt_cost:
            opt_cost = c
            opt_pos = new_pt.copy()

        done += 1
        if progress and done % 500 == 0:
            print(f"  {done}/{total} iterations")

    base_res = solve_sweep(
        SuspensionModel(geometry, config), n_shock_steps=n_shock_steps, n_steer_steps=1
    )
    base_cost = _objective_cost(base_res, objective)

    return OptimizationResult(
        opt_point=opt_point,
        objective=objective,
        sweep_axes=list(sweep_axes),
        base_pos=base_pos,
        opt_pos=opt_pos,
        base_cost=base_cost,
        opt_cost=opt_cost,
        axis_coords=axis_coords,
        cost=cost,
    )


def report(res: OptimizationResult) -> str:
    return (
        f"Optimizing {res.opt_point} for {res.objective}:\n"
        f"  Original: {res.base_pos[0]:.3f}, {res.base_pos[1]:.3f}, {res.base_pos[2]:.3f} "
        f"(metric {res.base_cost:.4f})\n"
        f"  Optimal:  {res.opt_pos[0]:.3f}, {res.opt_pos[1]:.3f}, {res.opt_pos[2]:.3f} "
        f"(metric {res.opt_cost:.4f})"
    )


def optimization_figure(res: OptimizationResult) -> go.Figure:
    """Plot the cost map: 1-D line, 2-D contour, or 3-D scatter."""
    axes = res.sweep_axes
    axis_names = ["X (long)", "Y (lat)", "Z (vert)"]
    fig = go.Figure()

    if len(axes) == 1:
        x = res.axis_coords[0]
        fig.add_trace(go.Scatter(x=x, y=res.cost, mode="lines+markers", name="cost"))
        fig.add_trace(
            go.Scatter(x=[res.base_pos[axes[0] - 1]], y=[res.base_cost], mode="markers",
                       marker=dict(symbol="x", size=12, color="black"), name="original")
        )
        fig.add_trace(
            go.Scatter(x=[res.opt_pos[axes[0] - 1]], y=[res.opt_cost], mode="markers",
                       marker=dict(symbol="star", size=14, color="red"), name="optimal")
        )
        fig.update_layout(
            xaxis_title=f"{axis_names[axes[0] - 1]} (in)",
            yaxis_title=res.objective,
            title=f"1D sweep of {res.opt_point}",
        )

    elif len(axes) == 2:
        fig.add_trace(
            go.Contour(x=res.axis_coords[0], y=res.axis_coords[1], z=res.cost,
                       colorscale="Viridis", colorbar=dict(title=res.objective))
        )
        fig.add_trace(
            go.Scatter(x=[res.base_pos[axes[0] - 1]], y=[res.base_pos[axes[1] - 1]],
                       mode="markers", marker=dict(symbol="x", size=12, color="white"), name="original")
        )
        fig.add_trace(
            go.Scatter(x=[res.opt_pos[axes[0] - 1]], y=[res.opt_pos[axes[1] - 1]],
                       mode="markers", marker=dict(symbol="star", size=14, color="red"), name="optimal")
        )
        fig.update_layout(
            xaxis_title=f"{axis_names[axes[0] - 1]} (in)",
            yaxis_title=f"{axis_names[axes[1] - 1]} (in)",
            title=f"Optimization of {res.opt_point} for {res.objective}",
        )

    else:
        grid = np.meshgrid(*res.axis_coords, indexing="ij")
        pts = np.stack([g.ravel() for g in grid], axis=1)
        c = res.cost.ravel()
        fig.add_trace(
            go.Scatter3d(x=pts[:, 0], y=pts[:, 1], z=pts[:, 2], mode="markers",
                         marker=dict(size=3, color=c, colorscale="Viridis",
                                     colorbar=dict(title=res.objective)), name="cost")
        )
        common_layout(fig)
        fig.update_layout(title=f"Optimization of {res.opt_point} (3D)")

    return fig


def common_layout(fig: go.Figure) -> None:
    fig.update_layout(margin=dict(l=40, r=40, t=40, b=40), height=600)
