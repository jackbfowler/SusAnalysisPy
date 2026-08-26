"""Shared CSV hardpoint/config parsing.

Ports the geometry/config parsing logic that is duplicated across the MATLAB
sources (``detectImportOptions`` + ``readtable`` + row-name -> field mapping),
including the parsing in ``SusAnalysis/print_axle.m``.

CSV format: a header row ``,x,y,z`` followed by rows of
``<point name>,<x>,<y>,<z>`` where ``<point name>`` maps to a suspension
hardpoint (e.g. ``Lower wishbone front pivot``). Configuration rows
(e.g. ``shock bump``, ``shock droop``, ``wheel size``, ``shock lower mount``)
may also be present.
"""
