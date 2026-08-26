"""Suspension geometry / kinematics analysis.

Ports of the MATLAB sources in ``SusAnalysis/SussyAnal/``.
"""
from .optimize import OptimizationResult, optimize, optimization_figure, report
from .shock_only import analyze as analyze_shock_only
from .solver import KinematicResults, solve_sweep
from .steer import analyze as analyze_steer
from .tie_on_arm import TieResult, optimize_tie, tie_figure

__all__ = [
    "KinematicResults",
    "solve_sweep",
    "analyze_steer",
    "analyze_shock_only",
    "optimize",
    "OptimizationResult",
    "optimization_figure",
    "report",
    "optimize_tie",
    "TieResult",
    "tie_figure",
]
