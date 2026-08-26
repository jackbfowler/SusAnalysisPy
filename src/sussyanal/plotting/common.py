"""Shared Plotly helpers: colors, layout, wheel mesh, arrows."""
from __future__ import annotations

import numpy as np
import plotly.graph_objects as go

Vec3 = np.ndarray

# Colors matching the MATLAB visualizers.
COLORS = {
    "lca": "blue",
    "uca": "red",
    "kp": "black",
    "shock": "magenta",
    "tie": "green",
    "axle": "orange",
    "hub": "gray",
    "wheel": "rgb(50,50,50)",
    "ground": "rgb(220,220,220)",
    "static_pt": "rgb(110,110,110)",
}


def unit(v: Vec3) -> Vec3:
    n = float(np.linalg.norm(v))
    return v if n < 1e-12 else v / n


def skew(v: Vec3) -> np.ndarray:
    return np.array(
        [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]]
    )


def rot_align_y(n: Vec3) -> np.ndarray:
    """Rotation matrix mapping local Y -> ``n`` (Rodrigues, matches MATLAB)."""
    n = unit(n)
    ya = np.array([0.0, 1.0, 0.0])
    v = np.cross(ya, n)
    s = float(np.linalg.norm(v))
    if s < 1e-9:
        return np.eye(3) * float(np.sign(np.dot(ya, n)) or 1.0)
    k = float(np.dot(ya, n))
    vx = skew(v)
    return np.eye(3) + vx + vx @ vx * (1.0 - k) / (s * s)


def wheel_mesh(diameter: float, width: float, n_theta: int = 48):
    """Open cylinder (wheel) with axis along local Y. Returns verts + triangles."""
    r = diameter / 2.0
    theta = np.linspace(0.0, 2.0 * np.pi, n_theta, endpoint=False)
    x = r * np.cos(theta)
    z = r * np.sin(theta)
    yb = np.full(n_theta, -width / 2.0)
    yt = np.full(n_theta, width / 2.0)

    verts = np.vstack(
        [np.concatenate([x, x]), np.concatenate([yb, yt]), np.concatenate([z, z])]
    )  # (3, 2*n_theta)

    i = np.arange(n_theta)
    inext = (i + 1) % n_theta
    tri = np.vstack(
        [
            np.vstack([i, inext, i + n_theta]),
            np.vstack([inext, inext + n_theta, i + n_theta]),
        ]
    ).T  # (2*n_theta, 3)
    return verts, tri


def transform_points(verts: np.ndarray, center: Vec3, axis: Vec3):
    """Rotate ``verts`` (3,N) so local Y aligns with ``axis``, then translate."""
    p = rot_align_y(axis) @ verts + np.asarray(center)[:, None]
    return p[0], p[1], p[2]


def rim_circle(center: Vec3, hub_axis: Vec3, diameter: float, n: int = 60):
    """Circle in the wheel plane (MATLAB rim line)."""
    r = diameter / 2.0
    p1 = np.cross(hub_axis, np.array([0.0, 0.0, 1.0]))
    if float(np.linalg.norm(p1)) < 0.01:
        p1 = np.cross(hub_axis, np.array([1.0, 0.0, 0.0]))
    p1 = unit(p1)
    p2 = np.cross(hub_axis, p1)
    t = np.linspace(0.0, 2.0 * np.pi, n)
    pts = (
        np.asarray(center)[:, None]
        + r * (np.cos(t)[None, :] * p1[:, None] + np.sin(t)[None, :] * p2[:, None])
    )
    return pts[0], pts[1], pts[2]


def add_arrow(fig: go.Figure, p: Vec3, v: Vec3, color: str, name: str, scale: float = 1.0):
    """Add a force arrow (shaft line + cone head) to a 3-D figure."""
    v = np.asarray(v, dtype=float)
    tip = p + v * scale
    fig.add_trace(
        go.Scatter3d(
            x=[p[0], tip[0]],
            y=[p[1], tip[1]],
            z=[p[2], tip[2]],
            mode="lines",
            line=dict(color=color, width=4),
            name=name,
            legendgroup=name,
            showlegend=True,
            hovertext=f"{name}: {np.linalg.norm(v):.1f}",
        )
    )
    head_len = 2.0
    d = unit(v)
    fig.add_trace(
        go.Cone(
            x=[tip[0]],
            y=[tip[1]],
            z=[tip[2]],
            u=[d[0] * head_len],
            v=[d[1] * head_len],
            w=[d[2] * head_len],
            sizemode="absolute",
            sizeref=1.0,
            anchor="tip",
            colorscale=[[0, color], [1, color]],
            showscale=False,
            name=f"{name} (head)",
            legendgroup=name,
            showlegend=False,
        )
    )


def layout3d(fig: go.Figure, title: str = "", aspect: str = "cube"):
    fig.update_layout(
        title=title,
        scene=dict(
            xaxis_title="X (in)",
            yaxis_title="Y (in)",
            zaxis_title="Z (in)",
            aspectmode="data",
            camera=dict(eye=dict(x=-1.2, y=-1.2, z=0.8)),
        ),
        margin=dict(l=0, r=0, t=40, b=0),
        height=720,
    )
    return fig
