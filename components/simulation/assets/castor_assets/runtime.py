"""The rig inside a running Isaac Sim: state logging, cable tension, distance-cable drawing, and the release API.

Needs Kit (omni.isaac.dynamic_control, pxr). Create it after author_rig(), call initialize() after world.reset(),
register on_physics_step() as a physics callback, and call apply_due_releases() from the stepping loop between
physics steps. The callback only records raw body states; stretch and tension are computed afterwards by metrics(),
which keeps the per-step cost down.

Release model: a release joint is switched off with physics:jointEnabled = False. PhysX applies it on the next step
(verified with pinned drones: the payload enters free fall one step after the call). This is the same switch as the
"Joint Enabled" box in the GUI's property panel. There is no latch dynamics: the hardware's servo travel is modelled
only as a delay (release.actuation_delay). The USD change must happen between physics steps: changing a physics
attribute inside a physics-step callback aborts the process (omni.physx handles the change notice mid-step), so
request_release() only queues, and apply_due_releases() makes the change.

Tension:
  * newton  (every model): the payload's Newton-Euler equations solved for non-negative tensions along each attached
            cable's direction at its anchor. Exact for massless cables while the payload touches nothing else; with
            N <= 6 cables the 6 equations determine every tension. NaN while the payload may touch the ground.
  * spring  (distance model with cable.stiffness): k * stretch + c * stretch rate while stretched. A cross-check only:
            it reads ~2-3 % low, because PhysX's distance-joint dead band is not exactly the 25 mm compensated for.
"""

from __future__ import annotations

import numpy as np
from omni.isaac.dynamic_control import _dynamic_control
from pxr import Gf, Sdf, UsdGeom
from scipy.optimize import nnls

from . import geometry as G
from .config import RigCfg
from .usd_build import RigHandles

ENDS = ("drone", "payload")


def quat_rotate(q, v):
    """Rotate v by quaternions q given as (x, y, z, w); broadcasts over leading axes."""
    qv, w = q[..., :3], q[..., 3:4]
    t = 2.0 * np.cross(qv, v)
    return v + w * t + np.cross(qv, t)


def quat_to_matrix(q):
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1),
    ], -2)


class RigRuntime:
    def __init__(self, stage, handles: RigHandles, rig: RigCfg, physics_dt: float, time_fn=None):
        """time_fn: returns simulated time (e.g. lambda: world.current_time), so the rig log lines up with the physics
        steps world.reset() already took; without it the clock starts at the first callback."""
        self.stage, self.h, self.rig, self.dt = stage, handles, rig, physics_dt
        self.time_fn = time_fn
        self.n = len(handles.cables)
        self.model = rig.cable.model
        self.L = rig.cable.length
        self.attached = np.ones((self.n, 2), dtype=bool)  # [drone end, payload end]
        self.pending = []  # [t_apply, cable, end, t_command]
        self.events = []  # (t_command, t_applied, cable, end)
        self.t = 0.0
        self.anchors_local = np.array(handles.layout.anchors_local)
        self.mounts_local = np.array(handles.layout.mounts_local)
        self._vis_ops = {}
        self.log = {k: [] for k in ("t", "payload_pos", "payload_quat", "payload_vel", "payload_angvel", "drone_pos",
                                    "drone_quat", "seg0_pos", "seg0_quat", "segN_pos", "segN_quat", "attached")}

    # ---------------------------------------------------------------------------------------------------------------
    def initialize(self):
        self.dc = _dynamic_control.acquire_dynamic_control_interface()
        self.payload_h = self.dc.get_rigid_body(self.h.payload)
        self.drone_h = [self.dc.get_rigid_body(c.drone_body) for c in self.h.cables]
        self.seg0_h = [self.dc.get_rigid_body(c.segments[0]) for c in self.h.cables if c.segments]
        self.segN_h = [self.dc.get_rigid_body(c.segments[-1]) for c in self.h.cables if c.segments]
        handles = [(self.h.payload, self.payload_h)] + [(c.drone_body, d) for c, d in zip(self.h.cables, self.drone_h)]
        missing = [p for p, hd in handles if hd == _dynamic_control.INVALID_HANDLE]
        if missing:
            raise RuntimeError(f"no PhysX rigid body behind {missing}; was world.reset() called?")

    def _pose(self, handle):
        p = self.dc.get_rigid_body_pose(handle)
        return np.array(p.p, dtype=float), np.array(p.r, dtype=float)  # quaternion (x, y, z, w)

    def payload_position(self):
        return self._pose(self.payload_h)[0]

    def endpoints(self):
        """Current world anchor and mount points, each (n, 3)."""
        pp, pq = self._pose(self.payload_h)
        anchors = pp + quat_rotate(pq, self.anchors_local)
        mounts = []
        for i, dh in enumerate(self.drone_h):
            dp, dq = self._pose(dh)
            mounts.append(dp + quat_rotate(dq, self.mounts_local[i]))
        return anchors, np.array(mounts)

    # ---------------------------------------------------------------------------------------------------------------
    def request_release(self, cable: int, end: str, t_command: float = None):
        """Command the release mechanism at `end` ("drone" or "payload") of cable `cable`. Takes effect after
        release.actuation_delay. Raises if the configured hardware has no mechanism there."""
        if end not in ENDS:
            raise ValueError(f"end must be one of {ENDS}, got {end!r}")
        if not 0 <= cable < self.n:
            raise ValueError(f"cable index {cable} out of range 0..{self.n - 1}")
        if not self.h.release_ends[ENDS.index(end)]:
            raise ValueError(f"release.location is {self.rig.release.location!r}: no mechanism at the {end} end")
        t_command = self.t if t_command is None else t_command
        self.pending.append([t_command + self.rig.release.actuation_delay, cable, end, t_command])

    def apply_due_releases(self, now=None):
        """Apply every queued release whose time has come. Call between physics steps, never from a physics callback."""
        now = (self.time_fn() if self.time_fn is not None else self.t) if now is None else now
        for item in list(self.pending):
            if item[0] <= now + 1e-9:
                self._apply_release(item[1], item[2])
                self.events.append((item[3], now, item[1], item[2]))
                self.pending.remove(item)

    def _apply_release(self, cable, end):
        c = self.h.cables[cable]
        path = c.joint_drone if end == "drone" else c.joint_payload
        self.stage.GetPrimAtPath(path).GetAttribute("physics:jointEnabled").Set(False)
        self.attached[cable, ENDS.index(end)] = False

    # ---------------------------------------------------------------------------------------------------------------
    def on_physics_step(self, dt):
        self.t = self.time_fn() if self.time_fn is not None else self.t + dt
        dc, L = self.dc, self.log
        pp, pq = self._pose(self.payload_h)
        L["t"].append(self.t)
        L["payload_pos"].append(pp)
        L["payload_quat"].append(pq)
        L["payload_vel"].append(np.array(dc.get_rigid_body_linear_velocity(self.payload_h), dtype=float))
        L["payload_angvel"].append(np.array(dc.get_rigid_body_angular_velocity(self.payload_h), dtype=float))
        poses = [self._pose(h) for h in self.drone_h]
        L["drone_pos"].append(np.array([p for p, _ in poses]))
        L["drone_quat"].append(np.array([q for _, q in poses]))
        if self.seg0_h:
            s0 = [self._pose(h) for h in self.seg0_h]
            sn = [self._pose(h) for h in self.segN_h]
            L["seg0_pos"].append(np.array([p for p, _ in s0]))
            L["seg0_quat"].append(np.array([q for _, q in s0]))
            L["segN_pos"].append(np.array([p for p, _ in sn]))
            L["segN_quat"].append(np.array([q for _, q in sn]))
        L["attached"].append(self.attached.copy())

    # ---------------------------------------------------------------------------------------------------------------
    def metrics(self):
        """Raw log plus derived per-step arrays: anchors, mounts, distance, stretch, directions, tensions, contact."""
        raw = {k: np.array(v) for k, v in self.log.items() if len(v)}
        if "t" not in raw:
            return raw
        pp, pq = raw["payload_pos"], raw["payload_quat"]
        anchors = pp[:, None, :] + quat_rotate(pq[:, None, :], self.anchors_local[None])
        mounts = raw["drone_pos"] + quat_rotate(raw["drone_quat"], self.mounts_local[None])
        dist = np.linalg.norm(mounts - anchors, axis=2)
        stretch = dist - self.L
        if self.model == "rope":
            dirs = quat_rotate(raw["seg0_quat"], np.array([0.0, 0.0, 1.0]))
        else:
            dirs = (mounts - anchors) / np.maximum(dist[..., None], 1e-12)
        attached = raw["attached"].all(axis=2)
        R = quat_to_matrix(pq)
        p = self.rig.payload
        bottom = pp[:, 2] - p.height / 2 * np.abs(R[:, 2, 2]) - p.radius * np.hypot(R[:, 2, 0], R[:, 2, 1])
        contact = bottom < 0.005

        t_spring = np.full(dist.shape, np.nan)
        if self.model == "distance" and self.rig.cable.stiffness is not None:
            rate = np.vstack([np.zeros((1, self.n)), np.diff(dist, axis=0) / self.dt])
            f = self.rig.cable.stiffness * stretch + self.rig.cable.damping * rate
            t_spring = np.where((stretch > 0) & attached, np.maximum(f, 0.0), 0.0)

        t_newton = np.full(dist.shape, np.nan)
        m = p.mass
        inertia_body = np.diag(G.payload_inertia(self.rig))
        com_local = np.asarray(p.com_offset, dtype=float)
        g = np.array([0.0, 0.0, -G.GRAVITY])
        rho = max(p.radius, 0.05)
        v, w = raw["payload_vel"], raw["payload_angvel"]
        for k in range(1, len(pp)):
            idx = np.flatnonzero(attached[k])
            if contact[k] or idx.size == 0:
                continue
            a = (v[k] - v[k - 1]) / self.dt
            alpha = (w[k] - w[k - 1]) / self.dt
            inertia = R[k] @ inertia_body @ R[k].T
            com = pp[k] + R[k] @ com_local
            rhs = np.concatenate([m * (a - g), (inertia @ alpha + np.cross(w[k], inertia @ w[k])) / rho])
            A = np.zeros((6, idx.size))
            for col, i in enumerate(idx):
                A[:3, col] = dirs[k, i]
                A[3:, col] = np.cross(anchors[k, i] - com, dirs[k, i]) / rho
            sol, _ = nnls(A, rhs)
            t_newton[k] = 0.0
            t_newton[k, idx] = sol
        if self.model == "rope":
            half = np.array([0.0, 0.0, self.h.cables[0].segment_length / 2])
            raw["gap_payload_end"] = np.linalg.norm(raw["seg0_pos"] + quat_rotate(raw["seg0_quat"], -half) - anchors, axis=2)
            raw["gap_drone_end"] = np.linalg.norm(raw["segN_pos"] + quat_rotate(raw["segN_quat"], half) - mounts, axis=2)
        raw.update(anchors=anchors, mounts=mounts, distance=dist, stretch=stretch, cable_dir=dirs,
                   tension_newton=t_newton, tension_spring=t_spring, payload_contact=contact)
        return raw

    def arrays(self, metrics=None):
        metrics = self.metrics() if metrics is None else metrics
        return {f"rig_{k}": v for k, v in metrics.items()}

    # ---------------------------------------------------------------------------------------------------------------
    def update_visuals(self):
        """Distance cables have no geometry of their own: draw each as a chain of thin cylinders, straight when taut,
        a catenary of the cable's length when slack, or hanging from the end that is still attached."""
        if self.model != "distance":
            return
        anchors, mounts = self.endpoints()
        centre = self.payload_position()
        floor = self.rig.cable.radius
        with Sdf.ChangeBlock():
            for i, c in enumerate(self.h.cables):
                drone_on, payload_on = self.attached[i]
                n = len(c.visual_segments)
                if drone_on and payload_on:
                    pts = G.cable_points(anchors[i], mounts[i], self.L, n, floor)
                elif drone_on or payload_on:
                    top = mounts[i] if drone_on else anchors[i]
                    pts = G.hanging_cable_points(top, self.L, n, floor, top - centre)
                else:
                    pts = np.repeat(anchors[i][None], n + 1, axis=0)
                for k, path in enumerate(c.visual_segments):
                    self._place(path, pts[k], pts[k + 1])

    def _place(self, path, p0, p1):
        ops = self._vis_ops.get(path)
        if ops is None:
            ops = UsdGeom.Xformable(self.stage.GetPrimAtPath(path)).GetOrderedXformOps()
            self._vis_ops[path] = ops
        d = np.asarray(p1) - np.asarray(p0)
        length = float(np.linalg.norm(d))
        q = G.quat_z_to(d) if length > 1e-9 else np.array([1.0, 0, 0, 0])
        mid = (np.asarray(p0) + np.asarray(p1)) / 2
        ops[0].Set(Gf.Vec3d(*[float(x) for x in mid]))
        ops[1].Set(Gf.Quatf(float(q[0]), Gf.Vec3f(float(q[1]), float(q[2]), float(q[3]))))
        ops[2].Set(Gf.Vec3f(1.0, 1.0, max(length, 1e-6)))
