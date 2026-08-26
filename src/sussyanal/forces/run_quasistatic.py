"""Driver for quasistatic force analysis with wheel-frame-transformed inputs.

Port of ``SusAnalysis/SussyForces/run_quasistatic.m``.

Runs kinematics to obtain geometry at a target shock/steer travel, transforms
wheel-frame (WFT) loads into the global frame, then solves and visualizes the
reaction forces.
"""
