"""Suspension geometry / kinematics analysis.

Port of the core MATLAB analyzer ``SusAnalysis/SussyAnal/sussy_steer.m``
(``sussy_shock_only`` is deprecated and ``sussy_tie_on_arm`` is out of scope).
"""
from .optimize import OptimizationResult, optimize, optimization_figure, report
from .solver import KinematicResults, solve_sweep
from .steer import analyze as analyze_steer

__all__ = [
    "KinematicResults",
    "solve_sweep",
    "analyze_steer",
    "optimize",
    "OptimizationResult",
    "optimization_figure",
    "report",
]
