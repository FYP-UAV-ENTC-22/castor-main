"""The payload pose as each drone gets it on the aircraft: the payload's own estimate, over a Wi-Fi link per drone.

The same model the policy is trained against (MARL_mav_carry_ext localization/payload_estimator.py and wifi_link.py),
for one rig instead of a batch of environments, from components/simulation/assets/config/payload_link.yaml:

  estimate   the payload's flight controller fuses an RTK receiver and a compass. Its errors drift (a filter's
             output), so each is a first-order Gauss-Markov process, around an offset that is fixed for the run:
             position, and a compass offset in yaw. A new estimate leaves at the telemetry rate.
  link       every drone has its own delay for the run, every read a little jitter, and now and then the newest
             packet is late: the drone then steps on an older pose.

Spreads given as [low, high] are drawn once per run, as training draws them per episode. Plain Python + numpy, so it
is tested on the host (test_payload_link.py) and runs in Isaac Sim's Python (stack_sim_pegasus.py).
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np


def quat_mul(a, b):
    """Hamilton product of two quaternions (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])


def rotvec_quat(v):
    """The rotation vector v [rad] as a quaternion (x, y, z, w)."""
    angle = float(np.linalg.norm(v))
    if angle < 1e-12:
        return np.array([0.5 * v[0], 0.5 * v[1], 0.5 * v[2], 1.0])
    return np.append(np.asarray(v) / angle * math.sin(angle / 2.0), math.cos(angle / 2.0))


def _spread(rng, value):
    """A [low, high] pair is drawn once; a number is used as it is."""
    if isinstance(value, (list, tuple)):
        return float(rng.uniform(value[0], value[1]))
    return float(value)


class PayloadEstimate:
    """Position and orientation (x, y, z, w) of the payload as its own estimator reports them."""

    def __init__(self, cfg: dict, physics_dt: float, rng):
        self.rng = rng
        self.every = max(1, round(1.0 / (float(cfg["rate_hz"]) * physics_dt)))
        self.keep = math.exp(-physics_dt / float(cfg["correlation_time"]))
        self.drive = math.sqrt(1.0 - self.keep ** 2)
        xy, tilt = _spread(rng, cfg["pos_noise_std_xy"]), _spread(rng, cfg["tilt_noise_std"])
        self.pos_noise = np.array([xy, xy, _spread(rng, cfg["pos_noise_std_z"])])
        self.rot_noise = np.array([tilt, tilt, _spread(rng, cfg["yaw_noise_std"])])
        bias_xy, bias_z, yaw = float(cfg["pos_bias_std_xy"]), float(cfg["pos_bias_std_z"]), float(cfg["yaw_bias_std"])
        self.pos_bias = rng.normal(0.0, 1.0, 3) * np.array([bias_xy, bias_xy, bias_z])
        self.yaw_bias = float(np.clip(rng.normal(0.0, yaw), -2.0 * yaw, 2.0 * yaw)) if yaw > 0.0 else 0.0
        # start anywhere in the error's own spread, not at zero
        self.pos_error = rng.normal(0.0, 1.0, 3) * self.pos_noise
        self.rot_error = rng.normal(0.0, 1.0, 3) * self.rot_noise
        self.steps = 0
        self.position = self.orientation = None

    def step(self, position, orientation):
        """Advance by one physics step from the true pose; returns the estimate the payload last sent."""
        self.pos_error = self.keep * self.pos_error + self.drive * self.rng.normal(0.0, 1.0, 3) * self.pos_noise
        self.rot_error = self.keep * self.rot_error + self.drive * self.rng.normal(0.0, 1.0, 3) * self.rot_noise
        if self.position is None or self.steps % self.every == 0:
            error = self.rot_error + np.array([0.0, 0.0, self.yaw_bias])
            self.position = np.asarray(position, dtype=float) + self.pos_bias + self.pos_error
            self.orientation = quat_mul(rotvec_quat(error), np.asarray(orientation, dtype=float))  # error in the world frame
        self.steps += 1
        return self.position, self.orientation


class WifiLink:
    """What each of `num_drones` drones has received of a stream pushed once per physics step."""

    def __init__(self, cfg: dict, num_drones: int, physics_dt: float, rng):
        self.rng, self.dt, self.cfg = rng, physics_dt, cfg
        self.delay = np.array([_spread(rng, cfg["delay_s"]) for _ in range(num_drones)])
        longest = float(np.max(self.delay)) + 4.0 * float(cfg["jitter_s"]) + float(cfg["late_extra_s"])
        self.sent = deque(maxlen=int(longest / physics_dt) + 2)

    def push(self, sample):
        self.sent.appendleft(sample)  # newest first

    def read(self, drone: int):
        """The newest sample that has reached `drone` by now."""
        delay = self.delay[drone] + self.rng.normal(0.0, float(self.cfg["jitter_s"]))
        if self.rng.random() < float(self.cfg["late_probability"]):
            delay += self.rng.random() * float(self.cfg["late_extra_s"])
        steps = min(max(int(round(delay / self.dt)), 0), len(self.sent) - 1)
        return self.sent[steps]


class PayloadLink:
    """The estimate and the links together: step() once per physics step, read(i) for drone i's pose."""

    def __init__(self, cfg: dict, num_drones: int, physics_dt: float, seed=None):
        rng = np.random.default_rng(seed)
        self.estimate = PayloadEstimate(cfg["estimate"], physics_dt, rng)
        self.link = WifiLink(cfg["link"], num_drones, physics_dt, rng)

    def step(self, position, orientation):
        self.link.push(self.estimate.step(position, orientation))

    def read(self, drone: int):
        return self.link.read(drone)

    def summary(self) -> str:
        e, l = self.estimate, self.link
        return (f"estimate noise {e.pos_noise[0] * 1000:.0f}/{e.pos_noise[2] * 1000:.0f} mm xy/z, "
                f"{math.degrees(e.rot_noise[0]):.1f}/{math.degrees(e.rot_noise[2]):.1f} deg tilt/yaw, offset "
                f"{np.round(e.pos_bias * 1000).astype(int).tolist()} mm, compass {math.degrees(e.yaw_bias):+.1f} deg; "
                f"link delay {np.round(l.delay * 1000).astype(int).tolist()} ms")
