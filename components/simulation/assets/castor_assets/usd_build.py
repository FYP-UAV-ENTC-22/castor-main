"""USD authoring for the S500 and the payload rig. Needs pxr, so import it only after an Isaac Sim SimulationApp exists.

The S500 asset keeps the prim layout Pegasus' Multirotor expects from its Iris:
    /vehicle            articulation root (PhysicsArticulationRootAPI)
      body              rigid body; all frame, landing-gear and avionics geometry hangs under it
      rotor0 .. rotor3  rigid bodies, PX4 quad-X order, each with a revolute joint0 .. joint3 to the body
Thrust is applied in Python by Pegasus, so the propellers only need mass and a spinning joint.

The rig (payload, cables, mount rods, release joints) is authored into whatever stage it is given: a live stage where
Pegasus has already spawned the drones, or a standalone assembly file built by ``build_rig_file``.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from dataclasses import dataclass, field

import numpy as np
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

from . import geometry as G
from .config import RigCfg, VehicleCfg, to_dict

COLORS = {
    "carbon": ((0.035, 0.035, 0.04), 0.35, 0.0),
    "nylon": ((0.07, 0.07, 0.075), 0.7, 0.0),
    "pcb": ((0.04, 0.04, 0.04), 0.45, 0.2),
    "motor": ((0.06, 0.06, 0.065), 0.3, 0.8),
    "motor_cap": ((0.75, 0.06, 0.06), 0.4, 0.3),
    "prop_ccw": ((0.82, 0.82, 0.80), 0.5, 0.0),
    "prop_cw": ((0.16, 0.16, 0.17), 0.5, 0.0),
    "foam": ((0.03, 0.03, 0.03), 0.95, 0.0),
    "foam_ring": ((0.70, 0.05, 0.05), 0.9, 0.0),
    "battery": ((0.13, 0.22, 0.50), 0.6, 0.0),
    "avionics": ((0.08, 0.32, 0.18), 0.5, 0.1),
    "gps": ((0.85, 0.85, 0.85), 0.5, 0.0),
    "misc": ((0.20, 0.20, 0.20), 0.7, 0.0),
    "rod": ((0.55, 0.55, 0.58), 0.3, 0.9),
    "release": ((0.95, 0.55, 0.05), 0.5, 0.0),
    "payload": ((0.95, 0.70, 0.12), 0.55, 0.0),
    "anchor": ((0.6, 0.6, 0.62), 0.3, 0.9),
    "cable": ((0.92, 0.92, 0.88), 0.8, 0.0),
    "ground": ((0.40, 0.40, 0.45), 0.9, 0.0),
}


# PhysX's PxDistanceJoint "tolerance": the joint only becomes active this far beyond maxDistance. omni.physx does not
# expose it; 25.0 mm was measured with 1 m and 2 m cables (pinned drones, spring and rigid limits).
DISTANCE_JOINT_TOLERANCE = 0.025

# a distance cable is drawn as this many cylinders, so it can sag when slack
CABLE_VISUAL_SEGMENTS = 24

# --------------------------------------------------------------------------------------------------------------------
# Low-level helpers
# --------------------------------------------------------------------------------------------------------------------


def _f3(v):
    return Gf.Vec3f(*[float(x) for x in v])


def _qf(q):
    return Gf.Quatf(float(q[0]), Gf.Vec3f(float(q[1]), float(q[2]), float(q[3])))


def _quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def _quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def _quat_yaw(yaw):
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


def set_pose(prim, pos=(0, 0, 0), quat=(1, 0, 0, 0), scale=None):
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*[float(x) for x in pos]))
    xf.AddOrientOp().Set(_qf(quat))
    if scale is not None:
        xf.AddScaleOp().Set(_f3(scale))


def _units(stage):
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    return stage


def save_atomic(stage, path):
    """Write an in-memory stage to `path` so that a process reading it never sees a half-written file: export to a
    temporary file in the same directory, then rename. Reloads the layer if this process already has it open."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    tmp = f"{path}.{os.getpid()}.tmp.usd"
    stage.GetRootLayer().Export(tmp)
    os.replace(tmp, path)
    layer = Sdf.Layer.Find(path)
    if layer is not None:
        layer.Reload(True)


def new_stage(path):
    """Create (or truncate) a USD file, even if this process already has it open."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    layer = Sdf.Layer.Find(path)
    if layer is not None:
        layer.Clear()
        stage = Usd.Stage.Open(layer)
    else:
        if os.path.exists(path):
            os.remove(path)
        stage = Usd.Stage.CreateNew(path)
    return _units(stage)


class Materials:
    def __init__(self, stage, scope):
        self.stage, self.scope, self.cache = stage, scope, {}
        UsdGeom.Scope.Define(stage, scope)

    def __call__(self, name, opacity=1.0):
        key = (name, opacity)
        if key in self.cache:
            return self.cache[key]
        color, roughness, metallic = COLORS[name]
        path = f"{self.scope}/{name}" + ("" if opacity >= 1.0 else f"_a{int(opacity * 100)}")
        mat = UsdShade.Material.Define(self.stage, path)
        shader = UsdShade.Shader.Define(self.stage, path + "/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
        if opacity < 1.0:
            shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(opacity))
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        self.cache[key] = mat
        return mat


def _finish(gprim, mat=None, collider=False, guide=False, color=None):
    prim = gprim.GetPrim()
    if mat is not None:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)
    if color is not None:
        gprim.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collider:
        UsdPhysics.CollisionAPI.Apply(prim)
    if guide:
        UsdGeom.Imageable(prim).CreatePurposeAttr(UsdGeom.Tokens.guide)
    return prim


def add_box(stage, path, size, pos=(0, 0, 0), quat=(1, 0, 0, 0), mat=None, collider=False, guide=False):
    g = UsdGeom.Cube.Define(stage, path)
    g.CreateSizeAttr(1.0)
    g.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
    set_pose(g.GetPrim(), pos, quat, size)
    return _finish(g, mat, collider, guide)


def _axis_extent(axis, r, half):
    lo = {"X": (-half, -r, -r), "Y": (-r, -half, -r), "Z": (-r, -r, -half)}[axis]
    return [Gf.Vec3f(*lo), Gf.Vec3f(*[-x for x in lo])]


def add_cylinder(stage, path, radius, height, axis="Z", pos=(0, 0, 0), quat=(1, 0, 0, 0), mat=None, collider=False,
                 guide=False):
    g = UsdGeom.Cylinder.Define(stage, path)
    g.CreateRadiusAttr(float(radius))
    g.CreateHeightAttr(float(height))
    g.CreateAxisAttr(axis)
    g.CreateExtentAttr(_axis_extent(axis, radius, height / 2))
    set_pose(g.GetPrim(), pos, quat)
    return _finish(g, mat, collider, guide)


def add_capsule(stage, path, radius, height, axis="Z", pos=(0, 0, 0), quat=(1, 0, 0, 0), mat=None, collider=False,
                guide=False):
    g = UsdGeom.Capsule.Define(stage, path)
    g.CreateRadiusAttr(float(radius))
    g.CreateHeightAttr(float(height))
    g.CreateAxisAttr(axis)
    g.CreateExtentAttr(_axis_extent(axis, radius, height / 2 + radius))
    set_pose(g.GetPrim(), pos, quat)
    return _finish(g, mat, collider, guide)


def add_mesh_cylinder(stage, path, radius, height, segments=64, pos=(0, 0, 0), mat=None):
    """Visual z-axis cylinder as a mesh with smooth sides (an implicit UsdGeom.Cylinder renders as an octagon)."""
    angles = np.linspace(0.0, 2.0 * math.pi, segments, endpoint=False)
    ring = np.stack([radius * np.cos(angles), radius * np.sin(angles)], axis=1)
    h = height / 2.0
    points = [(x, y, -h) for x, y in ring] + [(x, y, h) for x, y in ring] + [(0.0, 0.0, -h), (0.0, 0.0, h)]
    counts, indices, normals = [], [], []
    for k in range(segments):
        j = (k + 1) % segments
        counts.append(4)
        indices += [k, j, segments + j, segments + k]
        nk, nj = (math.cos(angles[k]), math.sin(angles[k]), 0.0), (math.cos(angles[j]), math.sin(angles[j]), 0.0)
        normals += [nk, nj, nj, nk]
    bottom, top = 2 * segments, 2 * segments + 1
    for k in range(segments):
        j = (k + 1) % segments
        counts += [3, 3]
        indices += [bottom, j, k, top, segments + k, segments + j]
        normals += [(0, 0, -1)] * 3 + [(0, 0, 1)] * 3
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in points])
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateNormalsAttr([Gf.Vec3f(*map(float, n)) for n in normals])
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateExtentAttr([Gf.Vec3f(-radius, -radius, -h), Gf.Vec3f(radius, radius, h)])
    set_pose(mesh.GetPrim(), pos)
    return _finish(mesh, mat)


def add_sphere(stage, path, radius, pos=(0, 0, 0), mat=None, collider=False, guide=False):
    g = UsdGeom.Sphere.Define(stage, path)
    g.CreateRadiusAttr(float(radius))
    g.CreateExtentAttr([Gf.Vec3f(-radius, -radius, -radius), Gf.Vec3f(radius, radius, radius)])
    set_pose(g.GetPrim(), pos)
    return _finish(g, mat, collider, guide)


def _rigid_body(prim, mass, com=(0, 0, 0), diag=None, axes=(1, 0, 0, 0), angular_damping=None, iterations=None,
                sleep=False):
    """Rigid body with authored mass properties. Sleeping is off by default: a payload hanging still under hovering
    drones would otherwise fall asleep, and disabling a release joint does not wake it."""
    UsdPhysics.RigidBodyAPI.Apply(prim)
    mapi = UsdPhysics.MassAPI.Apply(prim)
    mapi.CreateMassAttr(float(mass))
    mapi.CreateCenterOfMassAttr(_f3(com))
    if diag is not None:
        mapi.CreateDiagonalInertiaAttr(_f3(diag))
        mapi.CreatePrincipalAxesAttr(_qf(axes))
    papi = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    if not sleep:
        papi.CreateSleepThresholdAttr(0.0)
    if angular_damping is not None:
        papi.CreateAngularDampingAttr(float(angular_damping))
    if iterations is not None:
        papi.CreateSolverPositionIterationCountAttr(int(iterations[0]))
        papi.CreateSolverVelocityIterationCountAttr(int(iterations[1]))
    return prim


def _xform(stage, path, pos=None, quat=(1, 0, 0, 0)):
    x = UsdGeom.Xform.Define(stage, path)
    if pos is not None:
        set_pose(x.GetPrim(), pos, quat)
    return x.GetPrim()


# --------------------------------------------------------------------------------------------------------------------
# S500 vehicle
# --------------------------------------------------------------------------------------------------------------------


def build_vehicle_usd(cfg: VehicleCfg, path: str) -> dict:
    """Write the S500 asset. Returns a summary of the derived physical parameters."""
    geo = G.S500Geometry(cfg)
    mp = G.vehicle_mass_properties(cfg)
    f, mo, pr, lg, bm = cfg.frame, cfg.motor, cfg.propeller, cfg.landing_gear, cfg.battery_mount
    stage = _units(Usd.Stage.CreateInMemory())  # saved atomically at the end: parallel runs may read this file

    root = _xform(stage, "/vehicle")
    stage.SetDefaultPrim(root)
    UsdPhysics.ArticulationRootAPI.Apply(root)
    art = PhysxSchema.PhysxArticulationAPI.Apply(root)
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSleepThresholdAttr(0.0)
    art.CreateSolverPositionIterationCountAttr(int(cfg.physics.solver_position_iterations))
    art.CreateSolverVelocityIterationCountAttr(int(cfg.physics.solver_velocity_iterations))
    mat = Materials(stage, "/vehicle/Looks")

    body = _xform(stage, "/vehicle/body", (0, 0, 0))
    _rigid_body(body, mp.body_mass, mp.com, mp.diag, mp.principal_axes, cfg.physics.angular_damping)
    v = "/vehicle/body/visuals"
    c = "/vehicle/body/collisions"
    _xform(stage, v)
    _xform(stage, c)

    # centre plates and arms
    px, py = f.plate_size
    add_box(stage, f"{v}/top_plate", [px, py, f.plate_thickness], (0, 0, geo.z_top_plate), mat=mat("carbon"))
    add_box(stage, f"{v}/bottom_plate", [px, py, f.plate_thickness], (0, 0, geo.z_bottom_plate), mat=mat("pcb"))
    add_box(stage, f"{c}/plates", [px, py, geo.z_top_plate - geo.z_bottom_plate + f.plate_thickness], (0, 0, 0),
            collider=True, guide=True)
    arm_len = geo.arm_radius - f.arm_root_radius
    for i, ang in enumerate(geo.arm_angles):
        mid = (f.arm_root_radius + geo.arm_radius) / 2
        pos = (mid * math.cos(ang), mid * math.sin(ang), 0.0)
        add_box(stage, f"{v}/arm{i}", [arm_len, f.arm_width, f.arm_height], pos, _quat_yaw(ang), mat=mat("nylon"))
        add_box(stage, f"{c}/arm{i}", [arm_len, f.arm_width, f.arm_height], pos, _quat_yaw(ang), collider=True,
                guide=True)
        mx, my = geo.rotor_xy[i]
        add_cylinder(stage, f"{v}/motor_pad{i}", f.arm_width * 0.65, f.arm_height, "Z", (mx, my, 0.0),
                     mat=mat("nylon"))
        add_cylinder(stage, f"{v}/motor{i}", mo.diameter / 2, mo.height, "Z", (mx, my, geo.z_motor_centre),
                     mat=mat("motor"))
        add_cylinder(stage, f"{v}/motor_cap{i}", mo.diameter / 2 * 0.7, 0.003, "Z", (mx, my, geo.z_motor_top + 0.0015),
                     mat=mat("motor_cap"))

    # landing gear: top mount, pole, tee, skid with foam rings, per side
    tee_x, tee_y, tee_z = lg.tee_size
    z_pole = (geo.z_pole_top + geo.z_pole_bottom) / 2
    for side, y in zip("lr", geo.pole_y):
        x = lg.pole_x
        add_box(stage, f"{v}/gear_mount_{side}", [0.032, 0.030, lg.top_mount_height],
                (x, y, geo.z_plate_underside - lg.top_mount_height / 2), mat=mat("nylon"))
        add_cylinder(stage, f"{v}/pole_{side}", lg.pole_diameter / 2, lg.pole_length, "Z", (x, y, z_pole),
                     mat=mat("carbon"))
        add_box(stage, f"{v}/tee_{side}", lg.tee_size, (x, y, geo.z_tee_centre), mat=mat("nylon"))
        add_cylinder(stage, f"{v}/skid_{side}", lg.skid_diameter / 2, lg.skid_length, "X", (x, y, geo.z_skid),
                     mat=mat("carbon"))
        foam_len = (lg.skid_length - tee_x) / 2 - 0.02
        for k, sx in enumerate((-1, 1)):
            cx = x + sx * (tee_x / 2 + 0.005 + foam_len / 2)
            add_cylinder(stage, f"{v}/foam_{side}{k}", lg.foam_diameter / 2, foam_len, "X", (cx, y, geo.z_skid),
                         mat=mat("foam"))
            for j in range(3):
                rx = cx + (j - 1) * foam_len / 3
                add_cylinder(stage, f"{v}/foam_ring_{side}{k}{j}", lg.foam_diameter / 2 + 0.0005, 0.004, "X",
                             (rx, y, geo.z_skid), mat=mat("foam_ring"))
        add_capsule(stage, f"{c}/pole_{side}", lg.pole_diameter / 2, lg.pole_length, "Z", (x, y, z_pole),
                    collider=True, guide=True)
        add_capsule(stage, f"{c}/skid_{side}", max(lg.foam_diameter, lg.skid_diameter) / 2, lg.skid_length, "X",
                    (x, y, geo.z_skid), collider=True, guide=True)

    if bm.enabled:
        z_rail = geo.z_plate_underside - bm.drop
        for side, y in zip("lr", (bm.rail_spacing / 2, -bm.rail_spacing / 2)):
            add_cylinder(stage, f"{v}/battery_rail_{side}", bm.rail_diameter / 2, bm.rail_length, "X", (0, y, z_rail),
                         mat=mat("carbon"))
            for k, sx in enumerate((-1, 1)):
                add_box(stage, f"{v}/battery_bracket_{side}{k}", [0.012, 0.016, bm.drop + bm.rail_diameter / 2],
                        (sx * 0.055, y, geo.z_plate_underside - (bm.drop + bm.rail_diameter / 2) / 2), mat=mat("nylon"))

    # mass-budget boxes drawn where the budget puts them
    look = {"battery": "battery", "avionics": "avionics", "gps_mast": "gps"}
    for item in cfg.mass.items:
        if item.kind != "box" or item.name in ("plates_pmb", "wiring_misc", "battery_mount"):
            continue
        add_box(stage, f"{v}/{item.name}", item.size, item.pos, mat=mat(look.get(item.name, "misc")))
        if item.name == "battery":
            add_box(stage, f"{c}/battery", item.size, item.pos, collider=True, guide=True)
        if item.name == "gps_mast":
            mast_h = item.pos[2] - item.size[2] / 2 - geo.z_top_plate
            add_cylinder(stage, f"{v}/gps_pole", 0.005, mast_h, "Z",
                         (item.pos[0], item.pos[1], geo.z_top_plate + mast_h / 2), mat=mat("carbon"))

    # rotors: PX4 quad-X order, the same joint names Pegasus looks up for its propeller animation
    blade_len = pr.diameter / 2 - 0.008
    for i, p in enumerate(geo.rotor_positions):
        rp = f"/vehicle/rotor{i}"
        rotor = _xform(stage, rp, p)
        _rigid_body(rotor, mp.prop_mass, (0, 0, 0), mp.prop_inertia)
        ccw = cfg.propulsion.spin_directions[i] < 0
        look_prop = mat("prop_ccw" if ccw else "prop_cw")
        add_cylinder(stage, f"{rp}/visuals/hub", 0.008, 0.010, "Z", (0, 0, 0), mat=mat("motor"))
        twist = math.radians(10.0) * (1 if ccw else -1)
        for k, sx in enumerate((1, -1)):
            qt = _quat_mul(_quat_yaw(0.0 if sx > 0 else math.pi), np.array([math.cos(twist / 2), math.sin(twist / 2),
                                                                           0, 0]))
            add_box(stage, f"{rp}/visuals/blade{k}", [blade_len, 0.022, 0.003], (sx * (0.008 + blade_len / 2), 0, 0),
                    qt, mat=look_prop)
        if pr.show_disc:
            add_cylinder(stage, f"{rp}/visuals/disc", pr.diameter / 2, 0.0008, "Z", (0, 0, 0),
                         mat=mat("prop_ccw" if ccw else "prop_cw", opacity=0.12))
        joint = UsdPhysics.RevoluteJoint.Define(stage, f"{rp}/joint{i}")
        joint.CreateAxisAttr("Z")
        joint.CreateBody0Rel().SetTargets([Sdf.Path("/vehicle/body")])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(rp)])
        joint.CreateLocalPos0Attr(_f3(p))
        joint.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        joint.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))

    tc = G.thrust_constants(cfg)
    summary = {
        "name": cfg.name,
        "total_mass": float(mp.total_mass),
        "body_mass": float(mp.body_mass),
        "com": [float(x) for x in mp.com],
        "inertia_diag": [float(x) for x in mp.diag],
        "principal_axes_wxyz": [float(x) for x in mp.principal_axes],
        "rotor_positions": [[float(x) for x in p] for p in geo.rotor_positions],
        "rotor_constant": tc["rotor_constant"][0],
        "rolling_moment_coefficient": tc["rolling_moment_coefficient"][0],
        "thrust_to_weight": 4 * cfg.propulsion.max_thrust_per_motor / (mp.total_mass * G.GRAVITY),
        "hover_throttle": (math.sqrt(mp.total_mass * G.GRAVITY / 4 / tc["rotor_constant"][0]) - 100.0) / 1000.0,
        "spawn_height_on_ground": float(geo.spawn_height_on_ground),
        "overall_height": float(geo.height_overall),
        "mount_z_range": [float(geo.z_pole_top), float(geo.z_skid)],
    }
    stage.GetRootLayer().customLayerData = {"castor_vehicle": _json(summary), "castor_config": _json(to_dict(cfg))}
    save_atomic(stage, path)
    return summary


def _json(obj):
    """customLayerData cannot hold Python lists in a .usd (crate) file, so provenance is stored as JSON text."""
    return json.dumps(obj, default=lambda o: o.item() if hasattr(o, "item") else list(o))


# --------------------------------------------------------------------------------------------------------------------
# Vehicle description the rig builder needs, for either airframe
# --------------------------------------------------------------------------------------------------------------------


@dataclass
class VehicleInfo:
    kind: str  # s500 | iris
    usd_path: str
    mount_local: np.ndarray  # cable attach point, body frame
    rotor_envelope_radius: float
    total_mass: float
    max_thrust_total: float
    geometry: object = None  # S500Geometry for s500
    cfg: VehicleCfg = None
    thrust_curve: dict = field(default_factory=dict)  # QuadraticThrustCurve config; empty = Pegasus defaults
    linear_drag: list = field(default_factory=lambda: [0.50, 0.30, 0.0])
    motor_time_constant: float = 0.0
    spawn_height_on_ground: float = 0.07
    summary: dict = field(default_factory=dict)


IRIS_MASS = 1.5  # body as authored in iris.usd; the rotors carry a few grams of PhysX-derived mass on top
IRIS_MOUNT = np.array([0.0, 0.0, -0.067])  # bottom of the Iris body mesh
IRIS_ENVELOPE = 0.38  # furthest rotor 0.255 m + 10 in prop radius
IRIS_MAX_THRUST = 4 * 8.54858e-6 * 1100.0 ** 2  # Pegasus QuadraticThrustCurve defaults


def vehicle_info(rig: RigCfg, vehicle_cfg: VehicleCfg, out_dir: str, iris_usd: str = None,
                 unique: bool = False) -> VehicleInfo:
    """Build the vehicle asset (S500) or describe Pegasus' Iris, and return what the rig builder needs.

    unique: name the file after a hash of the vehicle config (<name>_<hash>.usd), so runs with different airframe
    settings in parallel never share, or overwrite, each other's asset."""
    if rig.vehicle == "iris":
        return VehicleInfo("iris", iris_usd, IRIS_MOUNT.copy(), IRIS_ENVELOPE, IRIS_MASS, IRIS_MAX_THRUST)
    name = vehicle_cfg.name
    if unique:
        name += "_" + hashlib.sha1(_json(to_dict(vehicle_cfg)).encode()).hexdigest()[:10]
    path = os.path.join(out_dir, f"{name}.usd")
    summary = build_vehicle_usd(vehicle_cfg, path)
    geo = G.S500Geometry(vehicle_cfg)
    p = vehicle_cfg.propulsion
    return VehicleInfo(
        "s500", path, geo.mount_point(rig.mount.height), geo.rotor_envelope_radius, summary["total_mass"],
        4 * p.max_thrust_per_motor, geo, vehicle_cfg, G.thrust_constants(vehicle_cfg), list(p.linear_drag),
        p.motor_time_constant, geo.spawn_height_on_ground + 0.002, summary,
    )


# --------------------------------------------------------------------------------------------------------------------
# Payload rig
# --------------------------------------------------------------------------------------------------------------------


@dataclass
class CableHandles:
    index: int
    model: str
    drone_body: str
    joint_drone: str  # joint released at the drone end
    joint_payload: str  # joint released at the payload end
    segments: list = field(default_factory=list)  # rope segment body paths, payload end first
    articulation: str = None  # rope articulation root (rope with >= 2 segments)
    visual: str = None  # distance model: the Xform holding the drawn cable
    segment_length: float = 0.0
    visual_segments: list = field(default_factory=list)  # distance model: cylinders under `visual`, payload end first


@dataclass
class RigHandles:
    layout: G.RigLayout
    payload: str
    cables: list
    release_ends: tuple  # which ends have a releasable mechanism
    root: str


def _ball_joint(stage, path, body0, body1, pos0, pos1, rot0=(1, 0, 0, 0), rot1=(1, 0, 0, 0), damping=0.0,
                exclude=False):
    """Three free rotations. With damping it is a D6 joint (translations locked, angular drives with zero stiffness)
    so PhysX applies the damping; without, a plain spherical joint."""
    if damping > 0.0:
        joint = UsdPhysics.Joint.Define(stage, path)
        prim = joint.GetPrim()
        for axis in ("transX", "transY", "transZ"):
            lim = UsdPhysics.LimitAPI.Apply(prim, axis)
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)
        for axis in ("rotX", "rotY", "rotZ"):
            drive = UsdPhysics.DriveAPI.Apply(prim, axis)
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(0.0)
            drive.CreateDampingAttr(float(damping))
    else:
        joint = UsdPhysics.SphericalJoint.Define(stage, path)
        prim = joint.GetPrim()
    joint.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    joint.CreateLocalPos0Attr(_f3(pos0))
    joint.CreateLocalPos1Attr(_f3(pos1))
    joint.CreateLocalRot0Attr(_qf(rot0))
    joint.CreateLocalRot1Attr(_qf(rot1))
    if exclude:
        joint.CreateExcludeFromArticulationAttr(True)
    return prim


def _add_mount_rod(stage, drone_path, vinfo: VehicleInfo, rig: RigCfg, mat, release_here):
    """Cross rod between the two landing-gear poles, under the drone's body, with the tie point at its centre."""
    m = vinfo.mount_local
    base = f"{drone_path}/body/mount"
    _xform(stage, base)
    if vinfo.kind == "s500":
        lg = vinfo.cfg.landing_gear
        r = rig.mount.rod_diameter / 2
        length = lg.pole_spacing + lg.pole_diameter
        add_cylinder(stage, f"{base}/rod", r, length, "Y", m, mat=mat("rod"))
        add_capsule(stage, f"{base}/rod_collider", r, length, "Y", m, collider=True, guide=True)
        for side, y in zip("lr", vinfo.geometry.pole_y):
            add_box(stage, f"{base}/clamp_{side}", [lg.pole_diameter + 0.008, 0.012, 2 * r + 0.008], (m[0], y, m[2]),
                    mat=mat("nylon"))
    if release_here:
        add_box(stage, f"{base}/release", [0.018, 0.012, 0.012], (m[0], m[1], m[2] - 0.009), mat=mat("release"))
    add_sphere(stage, f"{base}/tie", 0.005, (m[0], m[1], m[2] - (0.016 if release_here else 0.006)),
               mat=mat("anchor"))


def author_rig(stage, rig: RigCfg, vinfo: VehicleInfo, layout: G.RigLayout, drone_paths, root="/World") -> RigHandles:
    """Author payload, mount rods, cables and release joints into `stage`. The drones must already exist at
    `drone_paths` (Pegasus-spawned, or references in an assembly file)."""
    p, c = rig.payload, rig.cable
    loc = rig.release.location
    release_ends = (loc in ("drone", "both"), loc in ("payload", "both"))
    mat = Materials(stage, f"{root}/RigLooks")
    iters = (rig.physics.solver_position_iterations, rig.physics.solver_velocity_iterations)

    payload = f"{root}/payload"
    prim = _xform(stage, payload, layout.payload_pos)
    inertia = G.payload_inertia(rig)
    _rigid_body(prim, p.mass, p.com_offset, inertia, iterations=iters)
    add_mesh_cylinder(stage, f"{payload}/visual", p.radius, p.height, mat=mat("payload"))
    add_cylinder(stage, f"{payload}/collider", p.radius, p.height, "Z", (0, 0, 0), collider=True, guide=True)
    for i, a in enumerate(layout.anchors_local):
        add_sphere(stage, f"{payload}/anchor{i}", 0.008, a, mat=mat("anchor"))
        if release_ends[1]:
            add_box(stage, f"{payload}/release{i}", [0.020, 0.020, 0.012], (a[0], a[1], a[2] - 0.006),
                    mat=mat("release"))

    for i, dp in enumerate(drone_paths):
        _add_mount_rod(stage, dp, vinfo, rig, mat, release_ends[0])

    cables = []
    cable_root = f"{root}/cables"
    _xform(stage, cable_root)
    for i, dp in enumerate(drone_paths):
        body = f"{dp}/body"
        a_w, m_w = layout.anchors_world[i], layout.mounts_world[i]
        if c.model == "distance":
            path = f"{cable_root}/cable{i}"
            joint = UsdPhysics.DistanceJoint.Define(stage, path)
            joint.CreateBody0Rel().SetTargets([Sdf.Path(body)])
            joint.CreateBody1Rel().SetTargets([Sdf.Path(payload)])
            joint.CreateLocalPos0Attr(_f3(layout.mounts_local[i]))
            joint.CreateLocalPos1Attr(_f3(layout.anchors_local[i]))
            # PhysX only engages a distance joint DISTANCE_JOINT_TOLERANCE past maxDistance (measured, not exposed in
            # USD), so the limit is shortened by it. minDistance stays unset (-1): the cable can go slack.
            joint.CreateMaxDistanceAttr(float(c.length - DISTANCE_JOINT_TOLERANCE))
            joint.CreateExcludeFromArticulationAttr(True)
            if c.stiffness is not None:
                dj = PhysxSchema.PhysxPhysicsDistanceJointAPI.Apply(joint.GetPrim())
                dj.CreateSpringEnabledAttr(True)
                dj.CreateSpringStiffnessAttr(float(c.stiffness))
                dj.CreateSpringDampingAttr(float(c.damping))
            vis = f"{cable_root}/cable{i}_visual"
            _xform(stage, vis)
            pts = G.cable_points(a_w, m_w, c.length, CABLE_VISUAL_SEGMENTS)
            vis_segs = []
            for k in range(CABLE_VISUAL_SEGMENTS):
                cyl = UsdGeom.Cylinder.Define(stage, f"{vis}/seg{k}")
                cyl.CreateRadiusAttr(float(c.radius))
                cyl.CreateHeightAttr(1.0)
                cyl.CreateAxisAttr("Z")
                cyl.CreateExtentAttr(_axis_extent("Z", c.radius, 0.5))
                _finish(cyl, mat("cable"))
                place_segment(cyl.GetPrim(), pts[k], pts[k + 1])
                vis_segs.append(str(cyl.GetPath()))
            cables.append(CableHandles(i, "distance", body, path, path, visual=vis, segment_length=c.length,
                                       visual_segments=vis_segs))
            continue

        # rope: segments from the payload anchor (k = 0) up to the drone mount
        n = c.segments
        seg_len = c.length / n
        seg_mass = c.mass / n
        direction = (m_w - a_w) / np.linalg.norm(m_w - a_w)
        q_seg = G.quat_z_to(direction)
        rope = f"{cable_root}/rope{i}"
        rope_prim = _xform(stage, rope)
        if n >= 2:
            UsdPhysics.ArticulationRootAPI.Apply(rope_prim)
            art = PhysxSchema.PhysxArticulationAPI.Apply(rope_prim)
            art.CreateEnabledSelfCollisionsAttr(False)
            art.CreateSleepThresholdAttr(0.0)
            art.CreateSolverPositionIterationCountAttr(int(iters[0]))
            art.CreateSolverVelocityIterationCountAttr(int(iters[1]))
        if c.flycrane_inertia:
            seg_inertia = [1e-5, 1e-5, 1e-5]
        else:
            seg_inertia = list(np.diag(G.cylinder_inertia(seg_mass, c.radius, seg_len, "z")))
        segs = []
        for k in range(n):
            centre = a_w + direction * seg_len * (k + 0.5)
            sp = f"{rope}/seg{k}"
            sprim = _xform(stage, sp, centre, q_seg)
            _rigid_body(sprim, seg_mass, (0, 0, 0), seg_inertia, iterations=None if n >= 2 else iters)
            add_capsule(stage, f"{sp}/visual", c.radius, seg_len, "Z", mat=mat("cable"))
            if c.collisions:
                add_capsule(stage, f"{sp}/collider", c.radius, max(seg_len - 4 * c.radius, 1e-3), "Z", collider=True,
                            guide=True)
            segs.append(sp)
        half = (0, 0, seg_len / 2)
        neg_half = (0, 0, -seg_len / 2)
        for k in range(n - 1):
            _ball_joint(stage, f"{rope}/joint{k + 1}", segs[k], segs[k + 1], half, neg_half, damping=c.joint_damping)
        q_drone = _quat_yaw(layout.drone_yaw[i])
        jp = _ball_joint(stage, f"{rope}/attach_payload", payload, segs[0], layout.anchors_local[i], neg_half,
                         rot0=q_seg, damping=c.joint_damping, exclude=True)
        jd = _ball_joint(stage, f"{rope}/attach_drone", segs[-1], body, half, layout.mounts_local[i],
                         rot1=_quat_mul(_quat_conj(q_drone), q_seg), damping=c.joint_damping, exclude=True)
        cables.append(CableHandles(i, "rope", body, str(jd.GetPath()), str(jp.GetPath()), segs,
                                   rope if n >= 2 else None, segment_length=seg_len))
    return RigHandles(layout, payload, cables, release_ends, root)


def place_segment(prim, p0, p1):
    """Pose a unit-height, z-axis cylinder so it spans p0 -> p1."""
    p0, p1 = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)
    d = p1 - p0
    length = float(np.linalg.norm(d))
    q = G.quat_z_to(d) if length > 1e-9 else np.array([1.0, 0, 0, 0])
    set_pose(prim, (p0 + p1) / 2, q, (1.0, 1.0, max(length, 1e-6)))


def rig_layout_for(rig: RigCfg, vinfo: VehicleInfo):
    layout = G.rig_layout(rig, vinfo.mount_local, vinfo.rotor_envelope_radius, vinfo.total_mass,
                          vinfo.max_thrust_total)
    warnings = G.check_layout(rig, layout)
    if vinfo.kind == "s500" and vinfo.cfg.battery_mount.enabled:
        batt = next((it for it in vinfo.cfg.mass.items if it.name == "battery"), None)
        z_rod_top = vinfo.mount_local[2] + rig.mount.rod_diameter / 2
        z_mount_bottom = vinfo.geometry.z_plate_underside - vinfo.cfg.battery_mount.drop
        if batt is not None:
            z_mount_bottom = min(z_mount_bottom, batt.pos[2] - batt.size[2] / 2)
        if z_rod_top > z_mount_bottom:
            warnings.append(
                f"mount.height {rig.mount.height:.3f} m puts the rod inside the stock battery mount (rod top at "
                f"z = {z_rod_top:.3f} m, battery bottom at {z_mount_bottom:.3f} m); fine in simulation, not on the "
                f"real frame"
            )
    return layout, warnings


# --------------------------------------------------------------------------------------------------------------------
# Standalone assembly file
# --------------------------------------------------------------------------------------------------------------------


def build_rig_file(rig: RigCfg, vinfo: VehicleInfo, out_path: str, pin_drones=False, physics_hz=400.0):
    """A self-contained stage: physics scene, ground, lights, N referenced drones, payload, cables, release joints.

    Open it in Isaac Sim and press Play. Nothing drives the rotors here, so the drones fall unless `pin_drones` fixes
    each drone body to the world (a fixed joint makes its articulation fixed-base); the payload then hangs.
    """
    layout, warnings = rig_layout_for(rig, vinfo)
    stage = new_stage(out_path)
    world = _xform(stage, "/World")
    stage.SetDefaultPrim(world)
    scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(G.GRAVITY)
    px = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    px.CreateTimeStepsPerSecondAttr(int(physics_hz))
    px.CreateSolverTypeAttr("TGS")
    px.CreateEnableCCDAttr(True)
    mat = Materials(stage, "/World/Looks")
    ground = UsdGeom.Plane.Define(stage, "/World/ground")
    ground.CreateAxisAttr("Z")
    ground.CreateWidthAttr(40.0)
    ground.CreateLengthAttr(40.0)
    ground.CreateExtentAttr([Gf.Vec3f(-20, -20, 0), Gf.Vec3f(20, 20, 0)])
    _finish(ground, mat("ground"), collider=True)
    sun = UsdLux.DistantLight.Define(stage, "/World/sun")
    sun.CreateIntensityAttr(2500.0)
    set_pose(sun.GetPrim(), (0, 0, 10), G.quat_from_matrix(G.rot_z(0.6) @ np.array(
        [[1, 0, 0], [0, math.cos(0.7), -math.sin(0.7)], [0, math.sin(0.7), math.cos(0.7)]])))
    UsdLux.DomeLight.Define(stage, "/World/sky").CreateIntensityAttr(600.0)

    rel = os.path.relpath(vinfo.usd_path, os.path.dirname(os.path.abspath(out_path)))
    drone_paths = []
    for i in range(rig.num_drones):
        dp = f"/World/drone{i}"
        prim = _xform(stage, dp, layout.drone_pos[i], _quat_yaw(layout.drone_yaw[i]))
        prim.GetReferences().AddReference(rel if not rel.startswith("..") else vinfo.usd_path)
        drone_paths.append(dp)
        if pin_drones:
            fj = UsdPhysics.FixedJoint.Define(stage, f"/World/pins/drone{i}")
            fj.CreateBody1Rel().SetTargets([Sdf.Path(f"{dp}/body")])
            fj.CreateLocalPos0Attr(_f3(layout.drone_pos[i]))
            fj.CreateLocalRot0Attr(_qf(_quat_yaw(layout.drone_yaw[i])))
            fj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            fj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    handles = author_rig(stage, rig, vinfo, layout, drone_paths)
    stage.GetRootLayer().customLayerData = {
        "castor_rig": _json({
            "num_drones": rig.num_drones,
            "cable_model": rig.cable.model,
            "cable_angle_deg": math.degrees(layout.cable_angle),
            "horizontal_distance": layout.horizontal_distance,
            "anchor_radius": layout.anchor_radius,
            "rotor_clearance": layout.rotor_clearance if layout.n >= 2 else -1.0,
            "static_tension_N": layout.static_tension,
            "pinned_drones": bool(pin_drones),
        }),
        "castor_config": _json(to_dict(rig)),
    }
    stage.GetRootLayer().Save()
    return handles, warnings
