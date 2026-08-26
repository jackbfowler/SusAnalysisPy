"""Core suspension geometry model and math helpers.

Faithful port of the geometric pre-computation (``sussy_steer.m`` section 2)
and the per-step solver (section 4). All lengths are in inches and all angles
in radians internally (converted to degrees only at the results boundary).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

Vec3 = np.ndarray  # shape (3,)

_Z = np.array([0.0, 0.0, 1.0])
_X = np.array([1.0, 0.0, 0.0])
_Y = np.array([0.0, 1.0, 0.0])


# --------------------------------------------------------------------------- #
# Math helpers
# --------------------------------------------------------------------------- #
def unit(v: Vec3) -> Vec3:
    n = float(np.linalg.norm(v))
    return v if n < 1e-12 else v / n


def rotate_point(pt: Vec3, origin: Vec3, axis: Vec3, theta: float) -> Vec3:
    """Rodrigues rotation of ``pt`` about the line ``(origin, axis)``."""
    v = pt - origin
    a = unit(axis)
    c = math.cos(theta)
    s = math.sin(theta)
    return (
        origin
        + v * c
        + np.cross(a, v) * s
        + a * (np.dot(a, v) * (1.0 - c))
    )


def intersect_circle_sphere(
    c: Vec3, n: Vec3, r: float, s: Vec3, R: float
) -> tuple[Vec3, Vec3]:
    """Intersect circle (center ``c``, normal ``n``, radius ``r``) with sphere
    (center ``s``, radius ``R``). Returns the two intersection points."""
    n = unit(n)
    d = float(np.dot(s - c, n))
    proj = s - d * n
    r2 = R * R - d * d

    if r2 < 0.0:
        # Sphere does not reach the circle plane: nearest point on the circle.
        perp = np.cross(n, proj - c)
        u = unit(np.cross(perp, n))
        p = c + r * u
        return p, p.copy()

    vec = proj - c
    dist = float(np.linalg.norm(vec))
    if dist < 1e-9:
        perp = np.cross(n, _X)
        if float(np.linalg.norm(perp)) < 1e-9:
            perp = np.cross(n, _Y)
        perp = unit(perp)
        return c + r * perp, c - r * perp

    a = (r * r - r2 + dist * dist) / (2.0 * dist)
    h = math.sqrt(max(0.0, r * r - a * a))
    mid = c + a * (vec / dist)
    perp = unit(np.cross(n, vec / dist))
    return mid + h * perp, mid - h * perp


def _perp_basis(axis: Vec3) -> tuple[Vec3, Vec3]:
    """Orthonormal ``(p1, p2)`` spanning the plane normal to ``axis``.

    Matches MATLAB: ``p1 = unit(cross(axis, [0;0;1]))`` with fallback to X,
    then ``p2 = cross(axis, p1)``.
    """
    p1 = np.cross(axis, _Z)
    if float(np.linalg.norm(p1)) < 0.01:
        p1 = np.cross(axis, _X)
    p1 = unit(p1)
    p2 = np.cross(axis, p1)
    return p1, p2


def _ang_diff(a: float, b: float) -> float:
    """Wrapped angle difference in ``[0, pi]`` (MATLAB ``if d > pi, d = 2pi-d``)."""
    d = abs(a - b)
    return 2.0 * math.pi - d if d > math.pi else d


def _asind(x):  # noqa: ANN001 - matches MATLAB asind in degrees
    return math.degrees(math.asin(max(-1.0, min(1.0, x))))


def _atand(y, x):  # noqa: ANN001 - matches MATLAB atand(y/x)
    return math.degrees(math.atan2(y, x))


# --------------------------------------------------------------------------- #
# Data model
# --------------------------------------------------------------------------- #
@dataclass
class Config:
    """Suspension configuration, specific to one hardpoint set.

    Values are parsed from that set's CSV; ``missing`` lists any parameter
    that the CSV did not define, for which the documented fallback default is
    in effect for this set only.
    """

    bump: float = 5.0          # shock bump from ride height, in
    droop: float = 3.0         # shock droop from ride height, in
    wheel_size: float = 23.0   # wheel diameter, in
    steer_sweep: float = 0.0   # +/- steering rack travel, in (0 = no steering)
    shock_mount_lca: int = 0   # 1 = shock on LCA, 0 = shock on UCA
    wheelbase: float = 0.0     # in (0 = not defined)
    missing: tuple[str, ...] = ()

    @property
    def wheel_radius(self) -> float:
        return self.wheel_size / 2.0

    @property
    def wheel_width(self) -> float:
        return 8.0


@dataclass
class Geometry:
    lower_wishbone_front_pivot: Vec3
    lower_wishbone_rear_pivot: Vec3
    lower_wishbone_outer_ball_joint: Vec3
    upper_wishbone_front_pivot: Vec3
    upper_wishbone_rear_pivot: Vec3
    upper_wishbone_outer_ball_joint: Vec3
    lower_spring_pivot_point: Vec3
    upper_spring_pivot_point: Vec3
    outer_track_rod_ball_joint: Vec3
    inner_track_rod_ball_joint: Vec3
    wheel_spindle_point: Vec3
    wheel_centre_point: Vec3
    outer_axle_joint: Vec3 | None = None
    inner_axle_joint: Vec3 | None = None

    @property
    def has_axle(self) -> bool:
        return self.outer_axle_joint is not None and self.inner_axle_joint is not None

    @classmethod
    def from_mapping(cls, points: dict[str, Vec3]) -> "Geometry":
        from .io import _camel_to_snake  # local import avoids a cycle

        snake = {_camel_to_snake(k): v for k, v in points.items()}
        required = [
            "lower_wishbone_front_pivot",
            "lower_wishbone_rear_pivot",
            "lower_wishbone_outer_ball_joint",
            "upper_wishbone_front_pivot",
            "upper_wishbone_rear_pivot",
            "upper_wishbone_outer_ball_joint",
            "lower_spring_pivot_point",
            "upper_spring_pivot_point",
            "outer_track_rod_ball_joint",
            "inner_track_rod_ball_joint",
            "wheel_spindle_point",
            "wheel_centre_point",
        ]
        missing = [k for k in required if k not in snake]
        if missing:
            raise ValueError(f"CSV missing required hardpoints: {missing}")

        kwargs = {k: snake[k] for k in required}
        kwargs["outer_axle_joint"] = snake.get("outer_axle_joint")
        kwargs["inner_axle_joint"] = snake.get("inner_axle_joint")
        return cls(**kwargs)


@dataclass
class SuspensionData:
    """One hardpoint set plus its own configuration.

    The configuration parameters (bump, droop, wheel_size, steer_sweep,
    shock_mount_lca, wheelbase) belong to the hardpoint set they came from —
    they are never shared across sets.
    """

    geometry: Geometry
    config: Config


@dataclass
class Arm:
    front: Vec3
    rear: Vec3
    outer_init: Vec3
    axis: Vec3
    origin: Vec3
    center: Vec3
    radius: float
    axial_dist: float

    @classmethod
    def from_points(cls, front: Vec3, rear: Vec3, outer: Vec3) -> "Arm":
        axis = unit(rear - front)
        origin = front
        vec = outer - origin
        axial_dist = float(np.dot(vec, axis))
        center = origin + axial_dist * axis
        radius = float(np.linalg.norm(outer - center))
        return cls(front, rear, outer, axis, origin, center, radius, axial_dist)


@dataclass
class ShockGeom:
    lower_init: Vec3
    upper: Vec3
    length_init: float
    arm_origin: Vec3
    arm_axis: Vec3
    center_on_arm: Vec3
    radius_from_arm: float
    a: float
    b: float
    phi_init: float


@dataclass
class Upright:
    height: float
    tie_init: Vec3
    spindle_init: Vec3
    wheel_center_init: Vec3
    hub_length: float
    hub_axis_init: Vec3
    # local cylindrical coordinates in the kingpin frame
    tie_axial: float
    tie_rad: float
    tie_ang: float
    spin_axial: float
    spin_rad: float
    spin_ang: float
    wc_axial: float
    wc_rad: float
    wc_ang: float
    axle_axial: float = 0.0
    axle_rad: float = 0.0
    axle_ang: float = 0.0


@dataclass
class TieRod:
    inner_static: Vec3
    length: float


@dataclass
class Axle:
    outer_init: Vec3
    inner: Vec3
    length_init: float


@dataclass
class StepGeometry:
    """Everything the solver knows at one (shock length, rack position)."""
    shock_length: float
    rack_position: Vec3
    lca_outer: Vec3
    uca_outer: Vec3
    tie_outer: Vec3
    shock_lower: Vec3
    spindle: Vec3
    wheel_center: Vec3
    hub_axis: Vec3
    kp_axis: Vec3
    axle_outer: Vec3 | None
    # derived metrics
    camber: float
    caster: float
    kpi: float
    toe: float
    ground_z: float
    contact_patch: Vec3
    kp_ground: Vec3
    scrub: float
    trail: float
    lca_angle: float
    uca_angle: float
    plunge: float | None = None
    cv_in: float | None = None
    cv_out: float | None = None


# --------------------------------------------------------------------------- #
# Suspension model
# --------------------------------------------------------------------------- #
class SuspensionModel:
    """Static suspension geometry + a per-step closed-form solver."""

    def __init__(self, geometry: Geometry, config: Config):
        self.geometry = geometry
        self.config = config
        g = geometry

        self.lca = Arm.from_points(
            g.lower_wishbone_front_pivot,
            g.lower_wishbone_rear_pivot,
            g.lower_wishbone_outer_ball_joint,
        )
        self.uca = Arm.from_points(
            g.upper_wishbone_front_pivot,
            g.upper_wishbone_rear_pivot,
            g.upper_wishbone_outer_ball_joint,
        )

        # ---- shock geometry (mount on LCA or UCA) ----
        lower = g.lower_spring_pivot_point
        upper = g.upper_spring_pivot_point
        length_init = float(np.linalg.norm(upper - lower))
        if config.shock_mount_lca == 1:
            arm_origin, arm_axis = self.lca.origin, self.lca.axis
        else:
            arm_origin, arm_axis = self.uca.origin, self.uca.axis
        vec_shock = lower - arm_origin
        axial_dist_arm = float(np.dot(vec_shock, arm_axis))
        center_on_arm = arm_origin + axial_dist_arm * arm_axis
        radius_from_arm = float(np.linalg.norm(lower - center_on_arm))
        a = float(np.linalg.norm(upper - center_on_arm))
        b = radius_from_arm
        cos_phi = (a * a + b * b - length_init * length_init) / (2.0 * a * b)
        phi_init = math.acos(max(-1.0, min(1.0, cos_phi)))
        self.shock = ShockGeom(
            lower_init=lower,
            upper=upper,
            length_init=length_init,
            arm_origin=arm_origin,
            arm_axis=arm_axis,
            center_on_arm=center_on_arm,
            radius_from_arm=radius_from_arm,
            a=a,
            b=b,
            phi_init=phi_init,
        )

        # ---- upright ----
        tie_init = g.outer_track_rod_ball_joint
        spindle_init = g.wheel_spindle_point
        wheel_center_init = g.wheel_centre_point
        height = float(np.linalg.norm(self.uca.outer_init - self.lca.outer_init))

        kp_axis_init = unit(self.uca.outer_init - self.lca.outer_init)
        p1, p2 = _perp_basis(kp_axis_init)
        origin = self.lca.outer_init

        def get_loc(pt: Vec3):
            v = pt - origin
            axial = float(np.dot(v, kp_axis_init))
            radial = float(np.linalg.norm(np.cross(v, kp_axis_init)))
            perp = v - axial * kp_axis_init
            ang = math.atan2(float(np.dot(perp, p2)), float(np.dot(perp, p1)))
            return axial, radial, ang

        tie_axial, tie_rad, tie_ang = get_loc(tie_init)
        spin_axial, spin_rad, spin_ang = get_loc(spindle_init)
        wc_axial, wc_rad, wc_ang = get_loc(wheel_center_init)

        hub_vec = wheel_center_init - spindle_init
        hub_length = float(np.linalg.norm(hub_vec))
        hub_axis_init = _Y if hub_length < 1e-6 else hub_vec / hub_length

        self.upright = Upright(
            height=height,
            tie_init=tie_init,
            spindle_init=spindle_init,
            wheel_center_init=wheel_center_init,
            hub_length=hub_length,
            hub_axis_init=hub_axis_init,
            tie_axial=tie_axial,
            tie_rad=tie_rad,
            tie_ang=tie_ang,
            spin_axial=spin_axial,
            spin_rad=spin_rad,
            spin_ang=spin_ang,
            wc_axial=wc_axial,
            wc_rad=wc_rad,
            wc_ang=wc_ang,
        )

        # ---- axle ----
        self.axle: Axle | None = None
        if geometry.has_axle:
            axle_outer, axle_rad, axle_ang = get_loc(g.outer_axle_joint)
            self.upright.axle_axial = axle_outer
            self.upright.axle_rad = axle_rad
            self.upright.axle_ang = axle_ang
            self.axle = Axle(
                outer_init=g.outer_axle_joint,
                inner=g.inner_axle_joint,
                length_init=float(np.linalg.norm(g.outer_axle_joint - g.inner_axle_joint)),
            )

        # ---- tie rod ----
        self.tierod = TieRod(
            inner_static=g.inner_track_rod_ball_joint,
            length=float(np.linalg.norm(tie_init - g.inner_track_rod_ball_joint)),
        )

        self.track_width = 2.0 * abs(float(wheel_center_init[1]))

    # -- convenience accessors used across the codebase --
    @property
    def has_axle(self) -> bool:
        return self.axle is not None

    @classmethod
    def from_data(cls, data: "SuspensionData") -> "SuspensionModel":
        """Build a model from a bundled hardpoint set + its own config."""
        return cls(data.geometry, data.config)

    # ------------------------------------------------------------------ #
    def solve_arms(self, shock_length: float) -> tuple[Vec3, Vec3, Vec3, float]:
        """Solve primary + secondary arm positions for one shock length.

        Returns ``(lca_outer, uca_outer, shock_lower, delta_phi)`` where
        ``delta_phi`` is the shock-driven arm rotation.
        """
        cfg = self.config
        shock = self.shock
        up = self.upright

        cos_phi = (shock.a**2 + shock.b**2 - shock_length**2) / (2.0 * shock.a * shock.b)
        delta_phi = math.acos(max(-1.0, min(1.0, cos_phi))) - shock.phi_init

        if cfg.shock_mount_lca == 1:
            test = rotate_point(self.lca.outer_init, shock.arm_origin, shock.arm_axis, delta_phi)
            if shock_length < shock.length_init and test[2] < self.lca.outer_init[2]:
                delta_phi = -delta_phi
            cur_lca_outer = rotate_point(self.lca.outer_init, self.lca.origin, self.lca.axis, delta_phi)
            u1, u2 = intersect_circle_sphere(
                self.uca.center, self.uca.axis, self.uca.radius, cur_lca_outer, up.height
            )
            cur_uca_outer = u1 if u1[2] > u2[2] else u2
        else:
            test = rotate_point(self.uca.outer_init, shock.arm_origin, shock.arm_axis, delta_phi)
            if shock_length < shock.length_init and test[2] < self.uca.outer_init[2]:
                delta_phi = -delta_phi
            cur_uca_outer = rotate_point(self.uca.outer_init, self.uca.origin, self.uca.axis, delta_phi)
            l1, l2 = intersect_circle_sphere(
                self.lca.center, self.lca.axis, self.lca.radius, cur_uca_outer, up.height
            )
            cur_lca_outer = l1 if l1[2] < l2[2] else l2

        cur_shock_lower = rotate_point(shock.lower_init, shock.arm_origin, shock.arm_axis, delta_phi)
        return cur_lca_outer, cur_uca_outer, cur_shock_lower, delta_phi

    def solve_step(self, shock_length: float, rack_position: Vec3) -> StepGeometry:
        """Solve the suspension pose for one shock length + rack position."""
        cfg = self.config
        up = self.upright
        cur_lca_outer, cur_uca_outer, cur_shock_lower, _ = self.solve_arms(shock_length)

        kp_axis = unit(cur_uca_outer - cur_lca_outer)
        tie_center = cur_lca_outer + up.tie_axial * kp_axis
        p1, p2 = _perp_basis(kp_axis)

        t1, t2 = intersect_circle_sphere(
            tie_center, kp_axis, up.tie_rad, rack_position, self.tierod.length
        )
        ang1 = math.atan2(float(np.dot(t1 - tie_center, p2)), float(np.dot(t1 - tie_center, p1)))
        ang2 = math.atan2(float(np.dot(t2 - tie_center, p2)), float(np.dot(t2 - tie_center, p1)))
        if _ang_diff(ang1, up.tie_ang) < _ang_diff(ang2, up.tie_ang):
            cur_tie, cur_ang = t1, ang1
        else:
            cur_tie, cur_ang = t2, ang2
        rot_ang = cur_ang - up.tie_ang

        def pt(axial: float, radial: float, ang: float) -> Vec3:
            return (
                cur_lca_outer
                + axial * kp_axis
                + radial * (math.cos(ang + rot_ang) * p1 + math.sin(ang + rot_ang) * p2)
            )

        cur_spin = pt(up.spin_axial, up.spin_rad, up.spin_ang)
        cur_wc = pt(up.wc_axial, up.wc_rad, up.wc_ang)
        cur_hub = unit(cur_wc - cur_spin)

        camber = -_asind(cur_hub[2])
        caster = _atand(kp_axis[0], kp_axis[2])
        kpi = _atand(kp_axis[1], kp_axis[2])
        toe = math.degrees(math.atan2(cur_hub[0], cur_hub[1]))

        ground_z = cur_wc[2] - cfg.wheel_radius
        contact_patch = np.array([cur_wc[0], cur_wc[1], ground_z])

        if abs(kp_axis[2]) > 1e-4:
            kp_ground = cur_lca_outer + ((ground_z - cur_lca_outer[2]) / kp_axis[2]) * kp_axis
            scrub = float(cur_wc[1] - kp_ground[1])
            trail = float(cur_wc[0] - kp_ground[0])
        else:
            kp_ground = np.full(3, np.nan)
            scrub = trail = float("nan")

        lca_vec = unit(cur_lca_outer - self.lca.center)
        lca_angle = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(lca_vec, kp_axis))))))
        uca_vec = unit(cur_uca_outer - self.uca.center)
        uca_angle = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(uca_vec, kp_axis))))))

        plunge = cv_in = cv_out = None
        cur_axle = None
        if self.has_axle:
            cur_axle = pt(up.axle_axial, up.axle_rad, up.axle_ang)
            s_vec = cur_axle - self.axle.inner
            plunge = float(np.linalg.norm(s_vec)) - self.axle.length_init
            s_unit = unit(s_vec)
            cv_in = math.degrees(math.acos(abs(float(np.dot(s_unit, _Y)))))
            cv_out = math.degrees(math.acos(abs(float(np.dot(s_unit, cur_hub)))))

        return StepGeometry(
            shock_length=shock_length,
            rack_position=rack_position,
            lca_outer=cur_lca_outer,
            uca_outer=cur_uca_outer,
            tie_outer=cur_tie,
            shock_lower=cur_shock_lower,
            spindle=cur_spin,
            wheel_center=cur_wc,
            hub_axis=cur_hub,
            kp_axis=kp_axis,
            axle_outer=cur_axle,
            camber=camber,
            caster=caster,
            kpi=kpi,
            toe=toe,
            ground_z=ground_z,
            contact_patch=contact_patch,
            kp_ground=kp_ground,
            scrub=scrub,
            trail=trail,
            lca_angle=lca_angle,
            uca_angle=uca_angle,
            plunge=plunge,
            cv_in=cv_in,
            cv_out=cv_out,
        )
