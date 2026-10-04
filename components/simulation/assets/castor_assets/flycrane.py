"""The MARL flycrane training asset, rebuilt for the S500 and the payload rig from the same YAML as everything else.

The training env (MARL_mav_carry_ext, Isaac-flycrane-*) finds its bodies by name: ``Falcon<k>/base_link_inertia``
(drone CoM), ``Falcon<k>/rotor_<j>``, ``rope_<k>_link``, ``load/odometry_sensor_link`` and the
``rope_<k>_sphere_joint_*`` ball joints. So, like the ext's own scripts/gen_flycrane_urdf.py, this clones the
Falcon-1 and rope-1 blocks of a shipped flycrane URDF (structure and names only) and renumbers them, then
replaces every number that describes hardware:

- drone: S500 body mass, CoM and inertia (castor_assets.geometry), propellers at the S500 rotor positions in the
  Falcon's rotor order, collision box and rotor discs from s500.yaml; ``Falcon<k>/base_link`` is the cable mount
  (mount.height), as on the Falcon, where it is 3 cm under the body
- cable: one rigid rod per drone (training's model), cable.length / cable.mass / cable.joint_damping
- payload: the rig's cylinder (mass, inertia, size), its frame at the cylinder centre; anchors, the cable angle and
  the drone yaw from the rig layout; the joints' rest pose (all zero) is the spawn formation

The controllers in the ext carry their own Falcon constants (mass, inertia, arm, thrust map); ``drone_params``
returns the S500 values they would need. Nothing here changes those.
"""

from __future__ import annotations

import copy
import math
import re
import xml.etree.ElementTree as ET

import numpy as np

from . import config as C
from . import geometry as G

FALCON_RE = re.compile(r"^Falcon(\d+)/")
ROPE_RE = re.compile(r"^rope_(\d+)_")
# the Falcon's rotor order (rotor_0 .. rotor_3) as (sign x, sign y) in the body frame; the ext's allocation
# matrices and motor directions assume it
FALCON_ROTOR_SIGNS = [(1, 1), (1, -1), (-1, -1), (-1, 1)]


def rpy_from_matrix(R) -> tuple:
    """URDF rpy (fixed axes X, Y, Z: R = Rz(yaw) Ry(pitch) Rx(roll))."""
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:  # gimbal lock: put everything in yaw
        roll, yaw = 0.0, math.atan2(-R[0, 1], R[1, 1])
    return roll, pitch, yaw


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _f(v) -> str:
    return " ".join(f"{float(x):.9g}" for x in np.atleast_1d(v))


def _set_origin(el, xyz=(0, 0, 0), rpy=(0, 0, 0)):
    o = el.find("origin")
    if o is None:
        o = ET.Element("origin")
        el.insert(0, o)
    o.set("xyz", _f(xyz))
    o.set("rpy", _f(rpy))


def _set_inertial(link, mass, inertia, com=(0, 0, 0)):
    for old in link.findall("inertial"):
        link.remove(old)
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", xyz=_f(com), rpy="0 0 0")
    ET.SubElement(inertial, "mass", value=f"{mass:.9g}")
    I = np.asarray(inertia, dtype=float)
    I = np.diag(I) if I.ndim == 1 else I
    ET.SubElement(inertial, "inertia", ixx=f"{I[0, 0]:.9g}", ixy=f"{I[0, 1]:.9g}", ixz=f"{I[0, 2]:.9g}",
                  iyy=f"{I[1, 1]:.9g}", iyz=f"{I[1, 2]:.9g}", izz=f"{I[2, 2]:.9g}")


def _set_shapes(link, shapes, collision=True):
    """shapes: list of (geometry tag, attrs, xyz, rpy). Replaces every visual (and collision) of the link."""
    for tag in ("visual", "collision") if collision else ("visual",):
        for old in link.findall(tag):
            link.remove(old)
    for tag in ("visual", "collision") if collision else ("visual",):
        for geom, attrs, xyz, rpy in shapes:
            el = ET.SubElement(link, tag)
            ET.SubElement(el, "origin", xyz=_f(xyz), rpy=_f(rpy))
            g = ET.SubElement(el, "geometry")
            ET.SubElement(g, geom, **{k: _f(v) for k, v in attrs.items()})


def _renumber(el, old, new):
    c = copy.deepcopy(el)

    def fix(s):
        return re.sub(rf"^rope_{old}_", f"rope_{new}_", re.sub(rf"^Falcon{old}/", f"Falcon{new}/", s))

    if c.get("name"):
        c.set("name", fix(c.get("name")))
    for tag in ("parent", "child"):
        sub = c.find(tag)
        if sub is not None and sub.get("link"):
            sub.set("link", fix(sub.get("link")))
    return c


def drone_params(vehicle: C.VehicleCfg, rig: C.RigCfg) -> dict:
    """The S500 numbers the ext's controllers hardcode for the Falcon (geometric.py, indi.py, motor_model.py,
    max_thrust_pp in the env cfg), in the same terms."""
    geo = G.S500Geometry(vehicle)
    mp = G.vehicle_mass_properties(vehicle)
    p = vehicle.propulsion
    k = p.max_thrust_per_motor / p.max_rotor_velocity ** 2
    mount = geo.mount_point(rig.mount.height)
    return {
        "falcon_mass": mp.total_mass,  # controllers: whole drone, propellers included (Falcon 0.6017)
        "inertia_diag": [float(x) for x in np.diag(mp.inertia)],  # about the CoM (Falcon 0.00164 0.00184 0.0030)
        "arm_length_l": float(geo.arm_radius),  # centre to rotor (Falcon 0.106)
        "kappa": p.torque_to_thrust,  # m (Falcon 0.022)
        "max_thrust_pp": p.max_thrust_per_motor,  # N (Falcon 6.25)
        "motor_omega_max": p.max_rotor_velocity,  # rad/s (Falcon 2800)
        "thrust_map": k,  # N/(rad/s)^2 (Falcon 1.562522e-06)
        "torque_map": p.torque_to_thrust * k,  # N m/(rad/s)^2 (Falcon 3.4375484e-08)
        "motor_time_constant": p.motor_time_constant,  # s (Falcon tau_up = tau_down = 0.033)
        "rope_offset": float(mount[2] - mp.com[2]),  # cable mount below the CoM, body z (Falcon -0.03)
        "motor_inertia": None,  # not in s500.yaml; the Falcon uses 9.3575e-6 kg m^2
    }


def build_flycrane_urdf(template_path: str, out_path: str, rig: C.RigCfg, vehicle: C.VehicleCfg) -> dict:
    """Write the flycrane URDF for `rig` (num_drones, payload, cable, formation, mount) and `vehicle`."""
    if rig.vehicle != "s500":
        raise C.ConfigError(f"flycrane assets are built for the S500 only (rig.vehicle is {rig.vehicle!r})")
    geo = G.S500Geometry(vehicle)
    mp = G.vehicle_mass_properties(vehicle)
    mount = geo.mount_point(rig.mount.height)
    max_thrust_total = 4 * vehicle.propulsion.max_thrust_per_motor
    lay = G.rig_layout(rig, mount, geo.rotor_envelope_radius, mp.total_mass, max_thrust_total)
    warnings = G.check_layout(rig, lay)

    tree = ET.parse(template_path)
    root = tree.getroot()
    falcons = sorted({int(m.group(1)) for el in root for m in [FALCON_RE.match(el.get("name", ""))] if m})
    if not falcons:
        raise C.ConfigError(f"{template_path}: no Falcon<k>/ elements to clone")
    src = falcons[0]
    falcon_tpl = [el for el in root if el.tag in ("link", "joint") and (m := FALCON_RE.match(el.get("name", "")))
                  and int(m.group(1)) == src]
    rope_tpl = [el for el in root if el.tag in ("link", "joint") and (m := ROPE_RE.match(el.get("name", "")))
                and int(m.group(1)) == src]
    rope_links = [e for e in rope_tpl if e.tag == "link" and re.match(rf"^rope_{src}_link(_\d+)?$", e.get("name"))]
    if len(rope_links) != 1:
        raise C.ConfigError(f"{template_path}: template cable has {len(rope_links)} segments; need the one-rod "
                            "flycrane_rod template")
    for el in list(root):
        if el.tag in ("link", "joint") and (FALCON_RE.match(el.get("name", "")) or ROPE_RE.match(el.get("name", ""))):
            root.remove(el)

    # payload: frame at the cylinder centre
    pay = rig.payload
    load = root.find("link[@name='load_link']")
    _set_inertial(load, pay.mass, G.payload_inertia(rig), pay.com_offset)
    _set_shapes(load, [("cylinder", {"radius": pay.radius, "length": pay.height}, (0, 0, 0), (0, 0, 0))])
    for mat in load.findall("material"):
        load.remove(mat)
    _set_origin(root.find("joint[@name='load/odometry_sensor_joint']"))

    body_from_mount = -mount  # Falcon<k>/base_link is the mount; base_link_inertia is the body frame
    rotor_by_sign = {(int(np.sign(x)), int(np.sign(y))): p for (x, y), p in zip(geo.rotor_xy, geo.rotor_positions)}
    f, pr, c = vehicle.frame, vehicle.propeller, rig.cable
    z_lo, z_hi = geo.z_plate_underside, geo.z_motor_top
    body_shapes = [("box", {"size": (f.plate_size[0], f.plate_size[1], z_hi - z_lo)}, (0, 0, (z_hi + z_lo) / 2),
                    (0, 0, 0))]
    arm_shapes = [("box", {"size": (f.wheelbase, f.arm_width, f.arm_height)}, (0, 0, 0), (0, 0, a))
                  for a in (math.pi / 4, -math.pi / 4)]

    for k in range(1, rig.num_drones + 1):
        phi, yaw = lay.angles[k - 1], lay.drone_yaw[k - 1]
        R_anchor = G.rot_z(phi) @ rot_y(lay.cable_angle)  # rope +z from the anchor towards the drone's mount
        R_top = R_anchor.T @ G.rot_z(yaw)  # undo it at the top: the drone spawns level, at its formation yaw
        for el in falcon_tpl:
            new = _renumber(el, src, k)
            name = new.get("name").split("/", 1)[1]
            if name == "base_link":
                # the template's base_link has no <inertial>, so the URDF importer gives it 1 kg (measured on the
                # shipped flycrane_rod.usd: every Falcon is 1.617 kg in PhysX); a sensor-sized one instead
                _set_inertial(new, 1e-5, [1e-5, 1e-5, 1e-5])
            elif name == "base_joint":
                _set_origin(new, body_from_mount)
            elif name == "base_link_inertia":
                _set_inertial(new, mp.body_mass, mp.inertia, mp.com)
                _set_shapes(new, body_shapes)
                for shape in arm_shapes:  # arms: visual only, the box above is the drone's collider
                    v = ET.SubElement(new, "visual")
                    ET.SubElement(v, "origin", xyz=_f(shape[2]), rpy=_f(shape[3]))
                    ET.SubElement(ET.SubElement(v, "geometry"), "box", size=_f(shape[1]["size"]))
            elif re.fullmatch(r"rotor_\d_joint", name):
                j = int(name[6])
                _set_origin(new, body_from_mount + rotor_by_sign[FALCON_ROTOR_SIGNS[j]])
            elif re.fullmatch(r"rotor_\d", name):
                _set_inertial(new, mp.prop_mass, mp.prop_inertia)
                _set_shapes(new, [("cylinder", {"radius": pr.diameter / 2, "length": 0.005}, (0, 0, 0), (0, 0, 0))])
            elif name == "imu_joint":
                _set_origin(new, body_from_mount)
            root.append(new)
        anchor = lay.anchors_local[k - 1]
        for el in rope_tpl:
            new = _renumber(el, src, k)
            name = new.get("name")
            if name == f"rope_{k}_sphere_joint_0_joint_x":
                _set_origin(new, anchor, rpy_from_matrix(R_anchor))
            elif name == f"rope_{k}_sphere_joint_{_top_index(rope_tpl, src)}_joint_x":
                _set_origin(new, (0, 0, c.length), rpy_from_matrix(R_top))
            elif name == f"rope_{k}_link":
                _set_inertial(new, c.mass, [1e-5, 1e-5, 1e-5] if c.flycrane_inertia
                              else G.cylinder_inertia(c.mass, c.radius, c.length, "z"))
                _set_shapes(new, [("cylinder", {"radius": c.radius, "length": c.length}, (0, 0, c.length / 2),
                                   (0, 0, 0))])
            if new.tag == "joint" and new.find("dynamics") is not None:
                new.find("dynamics").set("damping", f"{c.joint_damping:.9g}")
            root.append(new)

    root.set("name", f"flycrane_s500_n{rig.num_drones}")
    ET.indent(tree)
    tree.write(out_path, xml_declaration=True, encoding="utf-8")
    links = [e for e in root if e.tag == "link"]
    total = sum(float(e.find("inertial/mass").get("value")) for e in links if e.find("inertial/mass") is not None)
    return {
        "urdf": out_path, "num_drones": rig.num_drones, "links": len(links),
        "joints": sum(1 for e in root if e.tag == "joint"), "total_mass": total,
        "drone_mass": mp.total_mass, "payload_mass": pay.mass, "cable_length": c.length,
        "cable_angle_deg": math.degrees(lay.cable_angle), "anchor_radius": lay.anchor_radius,
        "horizontal_distance": lay.horizontal_distance, "rotor_clearance": lay.rotor_clearance,
        "hover_throttle_share": lay.hover_throttle_share, "warnings": warnings,
    }


def _top_index(rope_tpl, src):
    """Index of the ball joint at the drone end of the template rope (7 in flycrane_rod)."""
    idx = [int(m.group(1)) for e in rope_tpl if e.tag == "joint"
           for m in [re.match(rf"^rope_{src}_sphere_joint_(\d+)_joint_x$", e.get("name"))] if m]
    return max(idx)
