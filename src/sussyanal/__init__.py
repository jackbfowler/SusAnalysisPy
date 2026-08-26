"""SussyAnal: Baja SAE suspension kinematics and force analysis.

Python port of the MATLAB reference implementation (``SusAnalysis/``, which is
gitignored and used for reference only).
"""
from __future__ import annotations

from .geometry import Config, Geometry, SuspensionModel
from .io import parse_csv

__version__ = "0.1.0"

__all__ = ["Config", "Geometry", "SuspensionModel", "parse_csv", "__version__"]
