"""Shock-only geometry analyzer (port of ``sussy_shock_only.m``).

Uses the shared ``solve_sweep`` with a single steer step. NOTE: the reference
``sussy_shock_only.m`` computes a slightly different bump-steer toe
(``asind`` of the tie-arm XZ projection); this wrapper reports the steer-solver
toe definition (``atan2d(hub_x, hub_y)``) for consistency across the package.
"""
from __future__ import annotations

from ..geometry import SuspensionModel
from ..io import parse_csv
from .solver import KinematicResults, solve_sweep


def analyze(csv, n_shock_steps: int = 200, progress: bool = False) -> KinematicResults:
    geometry, config = parse_csv(csv)
    model = SuspensionModel(geometry, config)
    return solve_sweep(model, n_shock_steps=n_shock_steps, n_steer_steps=1, progress=progress)
