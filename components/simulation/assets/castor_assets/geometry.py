"""Geometry and mass properties for the S500 and the payload rig. numpy only, no USD.

Frames: body frame is FLU (x forward, y left, z up) with its origin at the centre of the arm plane. That origin is
the body prim's pose, which Pegasus reports and RAPTOR controls. World frame is ENU, z up.
Quaternions are (w, x, y, z) unless a name says xyzw.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .config import ConfigError, RigCfg, VehicleCfg

GRAVITY = 9.81  # m/s^2, what Isaac Sim's World sets

# PX4 quad-X order, which mc_raptor assumes: 1 front-right, 2 back-left, 3 front-left, 4 back-right (FLU signs of x, y)
QUAD_X_SIGNS = [(1, -1), (-1, 1), (1, 1), (-1, -1)]


# --------------------------------------------------------------------------------------------------------------------
# Small math helpers
# --------------------------------------------------------------------------------------------------------------------


def rot_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def quat_from_matrix(R):
    """Rotation matrix -> unit quaternion (w, x, y, z)."""
    R = np.asarray(R, dtype=float)
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    q /= np.linalg.norm(q)
    return q if q[0] >= 0 else -q


def quat_z_to(direction):
    """Quaternion (w, x, y, z) rotating +z onto `direction`."""
    d = np.asarray(direction, dtype=float)
    d = d / np.linalg.norm(d)
    z = np.array([0.0, 0.0, 1.0])
    c = float(np.dot(z, d))
    if c > 1.0 - 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    if c < -1.0 + 1e-12:
        return np.array([0.0, 1.0, 0.0, 0.0])
    axis = np.cross(z, d)
    axis /= np.linalg.norm(axis)
    half = math.acos(c) / 2.0
    return np.concatenate([[math.cos(half)], axis * math.sin(half)])


def box_inertia(m, size):
    a, b, c = size
    return m / 12.0 * np.diag([b * b + c * c, a * a + c * c, a * a + b * b])


def cylinder_inertia(m, radius, length, axis="z"):
    """Solid cylinder about its centre."""
    i_perp = m * (3 * radius * radius + length * length) / 12.0
    i_axis = m * radius * radius / 2.0
    d = {"x": [i_axis, i_perp, i_perp], "y": [i_perp, i_axis, i_perp], "z": [i_perp, i_perp, i_axis]}[axis]
    return np.diag(d)


def parallel_axis(m, d):
    d = np.asarray(d, dtype=float)
    return m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))


def combine(parts):
    """parts: list of (mass, centre, inertia_about_centre_in_body_axes). Returns (mass, com, inertia_about_com)."""
    m = sum(p[0] for p in parts)
    com = sum(p[0] * np.asarray(p[1], dtype=float) for p in parts) / m
    inertia = sum(p[2] + parallel_axis(p[0], np.asarray(p[1]) - com) for p in parts)
    return m, com, inertia


def principal_axes(inertia):
    """Diagonalise. Returns (diag, quat wxyz) with axes kept as close as possible to the body axes."""
    inertia = 0.5 * (inertia + inertia.T)
    off = np.abs(inertia - np.diag(np.diag(inertia))).max()
    if off <= 1e-12 * max(np.trace(inertia), 1e-12):
        return np.diag(inertia).copy(), np.array([1.0, 0.0, 0.0, 0.0])
    vals, vecs = np.linalg.eigh(inertia)
    order = [int(np.argmax(np.abs(vecs[j, :]))) for j in range(3)]
    if sorted(order) != [0, 1, 2]:
        order = [0, 1, 2]
    R = vecs[:, order]
    vals = vals[order]
    for j in range(3):
        if R[j, j] < 0:
            R[:, j] *= -1
    if np.linalg.det(R) < 0:
        R[:, 2] *= -1
    return vals, quat_from_matrix(R)


# --------------------------------------------------------------------------------------------------------------------
# S500 geometry
# --------------------------------------------------------------------------------------------------------------------


@dataclass
class S500Geometry:
    """Derived z-levels and positions, all in the body frame."""

    cfg: VehicleCfg

    def __post_init__(self):
        f, mo, pr, lg = self.cfg.frame, self.cfg.motor, self.cfg.propeller, self.cfg.landing_gear
        self.arm_radius = f.wheelbase / 2.0
        self.z_arm_top = f.arm_height / 2.0
        self.z_arm_bottom = -f.arm_height / 2.0
        self.z_top_plate = self.z_arm_top + f.plate_thickness / 2.0
        self.z_bottom_plate = self.z_arm_bottom - f.plate_thickness / 2.0
        self.z_plate_underside = self.z_arm_bottom - f.plate_thickness
        self.z_motor_centre = self.z_arm_top + mo.height / 2.0
        self.z_motor_top = self.z_arm_top + mo.height
        self.z_prop = self.z_motor_top + pr.height_above_motor
        self.z_pole_top = self.z_plate_underside - lg.top_mount_height
        self.z_pole_bottom = self.z_pole_top - lg.pole_length
        tee_x, tee_y, tee_z = lg.tee_size
        self.z_tee_centre = self.z_pole_bottom - tee_z / 2.0
        self.z_skid = self.z_pole_bottom - tee_z + tee_y / 2.0
        self.z_ground = self.z_skid - max(lg.foam_diameter, lg.skid_diameter) / 2.0
        self.pole_y = [lg.pole_spacing / 2.0, -lg.pole_spacing / 2.0]
        d = self.arm_radius / math.sqrt(2.0)
        self.rotor_xy = [(sx * d, sy * d) for sx, sy in QUAD_X_SIGNS]
        self.rotor_positions = [np.array([x, y, self.z_prop]) for x, y in self.rotor_xy]
        self.motor_positions = [np.array([x, y, self.z_motor_centre]) for x, y in self.rotor_xy]
        self.arm_angles = [math.atan2(y, x) for x, y in self.rotor_xy]
        # radius of the circle (about the body z axis) that contains every rotor disc
        self.rotor_envelope_radius = self.arm_radius + self.cfg.propeller.diameter / 2.0
        self.height_overall = self.z_motor_top - self.z_ground

    @property
    def spawn_height_on_ground(self):
        """Body-origin height that puts the skids on z = 0."""
        return -self.z_ground

    @property
    def mount_height_max(self):
        """The lowest the cross rod can sit: level with the skids."""
        return self.z_pole_top - self.z_skid

    def mount_point(self, mount_height):
        """Cable attach point (centre of the cross rod between the two poles), body frame.

        mount_height = 0 puts the rod at the top of the poles, just under the frame; mount_height = pole_length puts it
        at the bottom of the poles, on top of the tee connectors; anything up to mount_height_max puts it on the
        landing gear's feet, between the tees, down to the level of the skids.
        """
        lg = self.cfg.landing_gear
        if not 0.0 <= mount_height <= self.mount_height_max + 1e-9:
            raise ConfigError(
                f"mount.height {mount_height:.3f} m is outside the landing gear: 0 is the top of the poles, "
                f"{lg.pole_length:.3f} m their bottom (landing_gear.pole_length), {self.mount_height_max:.4f} m the skids"
            )
        return np.array([lg.pole_x, 0.0, self.z_pole_top - mount_height])


def _mass_parts(geo: S500Geometry, scale: float):
    """Body mass budget as (mass, centre, inertia) parts, propellers excluded (they are separate rigid bodies)."""
    cfg = geo.cfg
    f, mo, lg = cfg.frame, cfg.motor, cfg.landing_gear
    parts = []
    for item in cfg.mass.items:
        m = item.mass * scale
        if m <= 0:
            continue
        if item.kind == "box":
            parts.append((m, np.array(item.pos, dtype=float), box_inertia(m, item.size)))
        elif item.kind == "motors":
            for p in geo.motor_positions:
                parts.append((m / 4, p, cylinder_inertia(m / 4, mo.diameter / 2, mo.height, "z")))
        elif item.kind == "arms":
            r0, r1 = f.arm_root_radius, geo.arm_radius
            length = r1 - r0
            for ang in geo.arm_angles:
                c = np.array([math.cos(ang), math.sin(ang), 0.0]) * (r0 + r1) / 2
                R = rot_z(ang)
                inertia = R @ box_inertia(m / 4, [length, f.arm_width, f.arm_height]) @ R.T
                parts.append((m / 4, c, inertia))
        elif item.kind == "landing_gear":
            m_pole, m_tee, m_skid = 0.40 * m / 2, 0.20 * m / 2, 0.40 * m / 2
            z_pole = (geo.z_pole_top + geo.z_pole_bottom) / 2
            for y in geo.pole_y:
                parts.append(
                    (m_pole, np.array([lg.pole_x, y, z_pole]),
                     cylinder_inertia(m_pole, lg.pole_diameter / 2, lg.pole_length, "z"))
                )
                parts.append((m_tee, np.array([lg.pole_x, y, geo.z_tee_centre]), box_inertia(m_tee, lg.tee_size)))
                parts.append(
                    (m_skid, np.array([lg.pole_x, y, geo.z_skid]),
                     cylinder_inertia(m_skid, lg.foam_diameter / 2, lg.skid_length, "x"))
                )
    return parts


@dataclass
class MassProperties:
    body_mass: float
    com: np.ndarray
    inertia: np.ndarray  # about com, body axes (3x3)
    diag: np.ndarray
    principal_axes: np.ndarray  # wxyz
    prop_mass: float
    prop_inertia: np.ndarray  # diag, rotor frame
    total_mass: float
    scale: float


def vehicle_mass_properties(cfg: VehicleCfg) -> MassProperties:
    geo = S500Geometry(cfg)
    prop_mass = cfg.propeller.mass
    item_sum = sum(max(i.mass, 0.0) for i in cfg.mass.items)
    if item_sum <= 0:
        raise ConfigError("mass.items sum to zero")
    scale = 1.0 if cfg.mass.total is None else (cfg.mass.total - 4 * prop_mass) / item_sum
    m, com, inertia = combine(_mass_parts(geo, scale))
    if cfg.mass.com_override is not None:
        com = np.asarray(cfg.mass.com_override, dtype=float)
    if cfg.mass.inertia_override is not None:
        inertia = np.diag(cfg.mass.inertia_override).astype(float)
    diag, q = principal_axes(inertia)
    # a propeller as a thin rod along its rotor-frame x axis; the floor keeps PhysX away from a degenerate inertia
    d = cfg.propeller.diameter
    i_long = prop_mass * d * d / 12.0
    prop_inertia = np.array([max(prop_mass * 0.012 ** 2 / 12.0, 1e-6), i_long, i_long])
    return MassProperties(m, com, inertia, diag, q, prop_mass, prop_inertia, m + 4 * prop_mass, scale)


def thrust_constants(cfg: VehicleCfg):
    """Pegasus QuadraticThrustCurve constants. RaptorBackend maps throttle u to omega = 1000 u + 100 rad/s, so the
    rotor constant is chosen so omega_max gives the configured full-throttle thrust."""
    p = cfg.propulsion
    k = p.max_thrust_per_motor / p.max_rotor_velocity ** 2
    return {
        "num_rotors": 4,
        "rotor_constant": [k] * 4,
        "rolling_moment_coefficient": [p.torque_to_thrust * k] * 4,
        "rot_dir": list(p.spin_directions),
        "min_rotor_velocity": [0.0] * 4,
        "max_rotor_velocity": [p.max_rotor_velocity] * 4,
    }


# --------------------------------------------------------------------------------------------------------------------
# Rig layout
# --------------------------------------------------------------------------------------------------------------------


@dataclass
class RigLayout:
    n: int
    payload_pos: np.ndarray  # world, payload body origin = cylinder centre
    anchors_local: list  # payload frame
    anchors_world: list
    mounts_local: list  # drone body frame (same for every drone)
    mounts_world: list
    drone_pos: list  # world, body origin
    drone_yaw: list  # rad
    angles: list  # rad, anchor/drone azimuth
    anchor_radius: float
    horizontal_distance: float
    cable_angle: float  # rad from vertical, signed (+ = drones outside the anchors)
    rotor_clearance: float  # m between neighbouring rotor envelopes (inf for N = 1)
    static_tension: float  # N per cable, rigid cables, payload hanging still, cable mass ignored
    hover_throttle_share: float  # per-drone weight incl. payload share / full thrust


def payload_inertia(cfg: RigCfg):
    p = cfg.payload
    if p.inertia is not None:
        return np.asarray(p.inertia, dtype=float)
    return np.diag(cylinder_inertia(p.mass, p.radius, p.height, "z"))


def rig_layout(rig: RigCfg, mount_local, rotor_envelope_radius, drone_mass, max_thrust_total) -> RigLayout:
    n = rig.num_drones
    p, f, c = rig.payload, rig.formation, rig.cable
    r_a = p.anchor_radius if p.anchor_radius is not None else (p.radius if n >= 2 else 0.0)
    z_a = p.anchor_height if p.anchor_height is not None else p.height / 2.0
    if f.horizontal_distance is not None:
        r_d = f.horizontal_distance
    else:
        r_d = r_a + c.length * math.sin(math.radians(f.cable_angle_deg))
    if n == 1 and r_a == 0.0:
        r_d = 0.0 if f.horizontal_distance is None else f.horizontal_distance
    if abs(r_d - r_a) >= c.length:
        raise ConfigError(
            f"drones at {r_d:.3f} m from the payload axis cannot reach anchors at {r_a:.3f} m with "
            f"{c.length:.3f} m cables"
        )
    dz = math.sqrt(c.length ** 2 - (r_d - r_a) ** 2)
    theta = math.atan2(r_d - r_a, dz)
    centre = np.array([rig.spawn.position[0], rig.spawn.position[1], rig.spawn.payload_clearance + p.height / 2.0])
    phi0 = math.radians(p.angle_offset_deg)
    angles = [phi0 + 2.0 * math.pi * i / n for i in range(n)]
    anchors_local, anchors_world, mounts_world, drone_pos, drone_yaw = [], [], [], [], []
    for phi in angles:
        a = np.array([r_a * math.cos(phi), r_a * math.sin(phi), z_a])
        anchors_local.append(a)
        anchors_world.append(centre + a)
        m_w = centre + np.array([r_d * math.cos(phi), r_d * math.sin(phi), z_a + dz])
        mounts_world.append(m_w)
        yaw = phi if f.drone_yaw == "outward" else 0.0
        drone_yaw.append(yaw)
        drone_pos.append(m_w - rot_z(yaw) @ np.asarray(mount_local, dtype=float))
    if n >= 2:
        # nearest-neighbour distance between drone body origins (they all sit on one circle at one height)
        horiz = [d[:2] for d in drone_pos]
        nearest = min(np.linalg.norm(horiz[i] - horiz[j]) for i in range(n) for j in range(i + 1, n))
        clearance = nearest - 2.0 * rotor_envelope_radius
    else:
        clearance = math.inf
    tension = p.mass * GRAVITY / n / math.cos(theta)
    share = (drone_mass + p.mass / n + (c.mass if c.model == "rope" else 0.0)) * GRAVITY / max_thrust_total
    return RigLayout(n, centre, anchors_local, anchors_world, [np.asarray(mount_local)] * n, mounts_world,
                     drone_pos, drone_yaw, angles, r_a, r_d, theta, clearance, tension, share)


def check_layout(rig: RigCfg, layout: RigLayout):
    """Raise ConfigError for layouts that cannot be flown; return a list of warnings for marginal ones."""
    warnings = []
    if layout.n >= 2 and layout.rotor_clearance < rig.formation.min_rotor_clearance:
        raise ConfigError(
            f"neighbouring drones' rotors are {layout.rotor_clearance:.3f} m apart (need "
            f">= {rig.formation.min_rotor_clearance:.3f} m, formation.min_rotor_clearance): increase "
            f"formation.horizontal_distance / cable_angle_deg, payload.radius or cable.length, or fly fewer drones"
        )
    if layout.hover_throttle_share > 0.7:
        warnings.append(
            f"each drone needs {100 * layout.hover_throttle_share:.0f}% of its full thrust just to hover with its "
            f"payload share; little margin left for control"
        )
    if layout.n >= 2 and layout.cable_angle < 0:
        warnings.append("drones sit inside the anchor circle (cables splay inward)")
    return warnings


# --------------------------------------------------------------------------------------------------------------------
# Cable drawing: a distance-joint cable has no geometry, so its shape is drawn from its end points
# --------------------------------------------------------------------------------------------------------------------


def cable_points(p0, p1, length, n, floor=None):
    """n + 1 points from p0 to p1 along a cable of `length`, evenly spaced along the cable.

    Ends at least `length` apart: the straight line between them. Closer (slack): the catenary of that length through
    both ends, the static shape of a uniform cable hanging under gravity. Drawing only; the distance joint is massless
    and shapeless, so waves and whip along a slack cable are not simulated. `floor` clamps z from below.
    """
    p0, p1 = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)
    d = p1 - p0
    u = np.linspace(0.0, 1.0, n + 1)
    if length - float(np.linalg.norm(d)) < 1e-4:
        pts = p0 + u[:, None] * d
    else:
        h = float(np.hypot(d[0], d[1]))
        e = d[:2] / h if h > 1e-9 else np.array([1.0, 0.0])
        h = max(h, 1e-4)  # ends one above the other: a very narrow catenary, i.e. the cable hangs folded
        v = float(d[2])
        # the span h fixes the catenary parameter a through sinh(b) / b = sqrt(L^2 - v^2) / h, b = h / (2a)
        r = math.sqrt(length ** 2 - v ** 2) / h
        lo, hi = 1e-9, 50.0
        for _ in range(80):
            b = 0.5 * (lo + hi)
            lo, hi = (b, hi) if math.sinh(b) / b < r else (lo, b)
        a = h / (2.0 * b)
        xm = h / 2.0 - a * math.atanh(v / length)  # lowest point, horizontal distance from p0
        x = xm + a * np.arcsinh(u * length / a - math.sinh(xm / a))  # inverse of the arc length from p0
        z = a * (np.cosh((x - xm) / a) - math.cosh(xm / a))
        pts = np.column_stack([p0[0] + x * e[0], p0[1] + x * e[1], p0[2] + z])
    if floor is not None:
        pts[:, 2] = np.maximum(pts[:, 2], floor)
    return pts


def hanging_cable_points(top, length, n, floor, lay_dir):
    """n + 1 points along a cable hanging straight down from `top`; what reaches `floor` lies along `lay_dir` (x y)."""
    top = np.asarray(top, dtype=float)
    e = np.asarray(lay_dir, dtype=float)[:2]
    e = e / np.linalg.norm(e) if np.linalg.norm(e) > 1e-9 else np.array([1.0, 0.0])
    drop = min(length, max(top[2] - floor, 0.0))
    s = np.linspace(0.0, length, n + 1)
    flat = np.maximum(s - drop, 0.0)
    return np.column_stack([top[0] + flat * e[0], top[1] + flat * e[1], top[2] - np.minimum(s, drop)])
