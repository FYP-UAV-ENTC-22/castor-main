"""RAPTOR as a Pegasus backend, shared by raptor_pegasus.py and raptor_payload.py. Import after SimulationApp exists
(it imports Pegasus) and with Pegasus' extension folder and .deps on sys.path.

Each vehicle gets its own RaptorBackend (own GRU state), so N vehicles = N independent RAPTOR instances.
The backend mirrors what src/modules/mc_raptor does on the flight controller:
  * called every physics step (like PX4 calls it on every gyro sample),
  * the GRU state only advances every round(physics_hz / 100) calls ("native" 100 Hz step),
    other calls evaluate from the current state without advancing it ("intermediate"),
  * previous-action input is the running average of actions since the last native step,
  * output (a + 1) / 2 remapped Crazyflie -> PX4 quad-X order, then Pegasus' Iris mapping
    omega = 1000 * u + 100 rad/s (the same mapping its PX4 backend applies to HIL_ACTUATOR_CONTROLS).
"""

import numpy as np
import torch
from scipy.spatial.transform import Rotation

from pegasus.simulator.logic.backends.backend import Backend

MAX_POSITION_ERROR = 0.5  # m, mc_raptor.hpp
MAX_VELOCITY_ERROR = 1.0  # m/s, mc_raptor.hpp
IRIS_INPUT_SCALING = 1000.0  # rad/s, PX4MavlinkBackendConfig default for Iris
IRIS_ZERO_POSITION_ARMED = 100.0  # rad/s
CRAZYFLIE_TO_PX4 = [0, 2, 3, 1]  # px4[i] = cf[CRAZYFLIE_TO_PX4[i]], mc_raptor.cpp REMAP_FROM_CRAZYFLIE


def yaw_matrix(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def make_trajectory(kind):
    """Returns f(t) -> (position ENU, velocity ENU, yaw rad), relative to the vehicle's home point."""
    if kind == "hover":
        return lambda t: (np.zeros(3), np.zeros(3), 0.0), 10.0

    if kind == "waypoints":
        wps = [  # (start time s, offset from home m, yaw deg)
            (0.0, [0.0, 0.0, 0.0], 0.0),
            (4.0, [1.5, 0.0, 0.5], 0.0),
            (9.0, [1.5, 1.5, 0.0], 90.0),
            (14.0, [0.0, 0.0, 0.0], 0.0),
        ]

        def f(t):
            wp = [w for w in wps if w[0] <= t][-1]
            return np.array(wp[1]), np.zeros(3), np.deg2rad(wp[2])

        f.waypoints = wps
        return f, 20.0

    # lissajous, same formula as mc_raptor/trajectories/lissajous.hpp
    A, B, C, a, b, c, period, ramp = 1.0, 0.5, 0.0, 1.0, 2.0, 1.0, 8.0, 3.0

    def f(t):
        tv = min(t, ramp) / ramp
        ramp_time = tv * min(t, ramp) / 2.0
        prog = (ramp_time + max(0.0, t - ramp)) * 2.0 * np.pi / period
        dprog = 2.0 * np.pi * tv / period
        pos = np.array([A * np.sin(a * prog), B * np.sin(b * prog), C * np.sin(c * prog)])
        vel = np.array([A * np.cos(a * prog) * a, B * np.cos(b * prog) * b, C * np.cos(c * prog) * c]) * dprog
        return pos, vel, 0.0

    return f, ramp + 2 * period


class RaptorBackend(Backend):
    def __init__(self, policy, trajectory, home, physics_hz):
        super().__init__(config=None)
        self.policy = policy
        self.trajectory = trajectory
        self.home = np.asarray(home, dtype=np.float64)
        self.native_every = max(1, int(round(physics_hz / 100.0)))
        self.log = {k: [] for k in ("t", "pos", "vel", "quat", "omega", "target", "yaw_target", "action", "native")}
        self.reset()

    # --- Backend API ---
    def reset(self):
        self.hidden = self.policy.initial_state(1)
        self.prev_action = torch.zeros(1, 4)  # RESET_PREVIOUS_ACTION_VALUE = 0 in mc_raptor.hpp
        self.action_hist = torch.zeros(1, 4)
        self.steps_since_native = 0
        self.substep = 0
        self.t = 0.0
        self.rotor_velocities = [0.0, 0.0, 0.0, 0.0]

    def start(self):
        self.reset()

    def stop(self):
        pass

    def update_sensor(self, sensor_type, data):
        pass

    def update_graphical_sensor(self, sensor_type, data):
        pass

    def update_state(self, state):
        pass

    def input_reference(self):
        return self.rotor_velocities

    def update(self, dt):
        state = self.vehicle.state
        p_off, v_t, yaw_t = self.trajectory(self.t)
        p_t = self.home + p_off

        R_t = yaw_matrix(yaw_t)
        R = Rotation.from_quat(state.attitude).as_matrix()  # body FLU -> world ENU
        p_err = np.clip(R_t.T @ (state.position - p_t), -MAX_POSITION_ERROR, MAX_POSITION_ERROR)
        v_err = np.clip(R_t.T @ (state.linear_velocity - v_t), -MAX_VELOCITY_ERROR, MAX_VELOCITY_ERROR)
        core = np.concatenate([p_err, (R_t.T @ R).reshape(-1), v_err, state.angular_velocity])

        # rl_tools l2f executor: average previous actions since the last native step
        if self.steps_since_native == 0:
            self.action_hist = self.prev_action.clone()
        else:
            n = self.steps_since_native
            self.action_hist = (self.action_hist * n + self.prev_action) / (n + 1)
        obs = torch.cat([torch.as_tensor(core, dtype=torch.float32).unsqueeze(0), self.action_hist], dim=1)

        native = self.substep % self.native_every == 0
        with torch.no_grad():
            action, hidden = self.policy(obs, self.hidden)
        if native:
            self.hidden = hidden
        action = action.clamp(-1.0, 1.0)
        self.steps_since_native = 0 if native else self.steps_since_native + 1
        self.substep += 1
        self.prev_action = action

        throttle_cf = ((action[0] + 1.0) / 2.0).numpy()
        throttle_px4 = throttle_cf[CRAZYFLIE_TO_PX4]
        self.rotor_velocities = list(throttle_px4 * IRIS_INPUT_SCALING + IRIS_ZERO_POSITION_ARMED)

        L = self.log
        L["t"].append(self.t)
        L["pos"].append(np.array(state.position))
        L["vel"].append(np.array(state.linear_velocity))
        L["quat"].append(np.array(state.attitude))
        L["omega"].append(np.array(state.angular_velocity))
        L["target"].append(p_t)
        L["yaw_target"].append(yaw_t)
        L["action"].append(throttle_px4)
        L["native"].append(native)
        self.t += dt
