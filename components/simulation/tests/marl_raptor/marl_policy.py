"""The MARL policy's side of the MARL -> RAPTOR chain, outside the training environment.

What Isaac-castor-payload-decentralized-hovering-v0 does around the trained network, rebuilt in NumPy so it can run
next to Pegasus (and serve as the reference for the onboard C++ runner):

  Observation     one 45-value frame per drone, three frames of history, oldest first -> 135 values
  OnnxPolicy      the exported actor; its input scaler is inside the file
  SetpointIntegrator   action * step scale added to a persistent position setpoint, the step no longer than
                  max speed * dt, velocity = increment / dt, setpoint kept within 1.5 m of the drone

The step scale, the speed cap and the point on the drone the policy calls its position belong to the task a policy
was trained on; TRAINED_ON has them for the two there are.

Everything is in one world frame (ENU here). Rotation matrices map body to world. Both velocities are in the world
frame, the angular one too: that is what the training environment feeds the policy.

    frame = [payload position (3), payload rotation (9, row-major), one-hot slot (3),
             drone position (3), drone rotation (9), drone linear velocity (3), drone angular velocity (3),
             goal position - payload position (3), goal rotation @ payload rotation^T (9)]

Run it on its own to check all three pieces against a trace recorded in the training environment
(scripts/tools/capture_policy_trace.py in the MARL repo); exits 1 on a mismatch:

    python marl_policy.py                       # the Falcon task's policy: models/castor_hover_falcon/v1
    python marl_policy.py --trained_on s500     # models/castor_hover_s500/v1
    python marl_policy.py --models ../../../../models/<name>/<version>
"""

import argparse
import ast
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
# onnxruntime, see README: .deps for the castor env (Python 3.11), .deps-py312 for the ROS containers
sys.path.insert(0, os.path.join(HERE, ".deps" if sys.version_info[:2] == (3, 11) else ".deps-py%d%d" % sys.version_info[:2]))

# Policy model packages (models/README.md at the repo root); the containers mount them and set CASTOR_MODELS_ROOT.
MODELS_ROOT = os.environ.get("CASTOR_MODELS_ROOT") or os.path.normpath(os.path.join(HERE, "../../../../models"))


FRAME_DIM = 45
HISTORY = 3
STEP_SCALE = 0.05  # m per policy step at unit action (setpoint_step_scale)
LEASH = 1.5  # m (setpoint_leash)
POLICY_DT = 0.02  # s: the policy runs at 50 Hz

# What each training task fixes, for the runners here (--trained_on):
#   rig         the rig config the task's asset was built from
#   point       the point the policy calls the drone's position: "mount" = 0.03 m above the cable tie point (the
#               Falcon reports base_link_inertia, that far above where its cable ties on), "com" = the body's centre
#               of mass (the S500 task reports base_link's)
#   step_scale  setpoint_step_scale in training, m per unit action per step
#   max_speed   setpoint_max_speed in training, m/s; None = no cap
#   model       the exported policy of that task: a package under MODELS_ROOT
TRAINED_ON = {
    "falcon": {"rig": "payload_rig_marl.yaml", "point": "mount", "step_scale": 0.05, "max_speed": None,
               "model": "castor_hover_falcon/v1"},
    "s500": {"rig": "payload_rig_marl_s500.yaml", "point": "com", "step_scale": 0.02, "max_speed": 1.0,
             "model": "castor_hover_s500/v1"},
}
POINT_ABOVE_MOUNT = 0.03  # m, for point = "mount"


def models_dir(trained_on):
    """The package the exported policy of that task is kept in."""
    return os.path.join(MODELS_ROOT, TRAINED_ON[trained_on]["model"])


def policy_point(trained_on, mount_local, com_local):
    """The policy's drone point in the body frame."""
    if TRAINED_ON[trained_on]["point"] == "com":
        return np.asarray(com_local, dtype=float)
    return np.asarray(mount_local, dtype=float) + np.array([0.0, 0.0, POINT_ABOVE_MOUNT])


def quat_wxyz_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def euler_xyz_to_matrix(roll, pitch, yaw):
    """Rz(yaw) @ Ry(pitch) @ Rx(roll), the convention the training environment draws its goals with."""
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def frame(slot, num_drones, payload_pos, payload_rot, drone_pos, drone_rot, drone_lin_vel, drone_ang_vel, goal_pos,
          goal_rot):
    """One 45-value observation frame for the drone in `slot`."""
    one_hot = np.zeros(num_drones)
    one_hot[slot] = 1.0
    return np.concatenate([
        payload_pos, np.asarray(payload_rot).reshape(-1), one_hot,
        drone_pos, np.asarray(drone_rot).reshape(-1), drone_lin_vel, drone_ang_vel,
        np.asarray(goal_pos) - payload_pos, (np.asarray(goal_rot) @ np.asarray(payload_rot).T).reshape(-1),
    ]).astype(np.float32)


class Observation:
    """The frame history of one drone. The first frame after a reset fills the whole history, as in training."""

    def __init__(self, slot, num_drones=3):
        self.slot, self.num_drones = slot, num_drones
        self.history = None

    def reset(self):
        self.history = None

    def push(self, **state):
        f = frame(self.slot, self.num_drones, **state)
        if self.history is None:
            self.history = [f] * HISTORY
        else:
            self.history = self.history[1:] + [f]
        return np.concatenate(self.history)


class OnnxPolicy:
    """The exported actor of one slot: raw observation in, mean action out."""

    def __init__(self, path):
        import onnxruntime

        options = onnxruntime.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.session = onnxruntime.InferenceSession(path, options, providers=["CPUExecutionProvider"])
        self.obs_dim = self.session.get_inputs()[0].shape[-1]
        if self.obs_dim != FRAME_DIM * HISTORY:
            raise ValueError(f"{path} takes {self.obs_dim} inputs, this pipeline builds {FRAME_DIM * HISTORY}")

    def __call__(self, observation):
        return self.session.run(["action"], {"observation": observation[None].astype(np.float32)})[0][0]


class SetpointIntegrator:
    """The position setpoint of one drone: absolute, persistent, moved by the policy's actions."""

    def __init__(self, step_scale=STEP_SCALE, leash=LEASH, dt=POLICY_DT, max_speed=None):
        self.step_scale, self.leash, self.dt, self.max_speed = step_scale, leash, dt, max_speed
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)

    def seed(self, drone_pos):
        """Start from where the drone is, so switching the policy on never makes the setpoint jump."""
        self.position = np.array(drone_pos, dtype=float)
        self.velocity = np.zeros(3)

    def step(self, action, drone_pos):
        increment = np.asarray(action, dtype=float) * self.step_scale
        if self.max_speed is not None:
            # actions are unbounded: the step scale is only the speed at unit action
            step = max(float(np.linalg.norm(increment)), 1e-9)
            increment = increment * min(self.max_speed * self.dt / step, 1.0)
        self.position = self.position + increment
        # The increment is the velocity command, so the feedforward cannot contradict the position.
        self.velocity = increment / self.dt
        offset = self.position - drone_pos
        distance = max(float(np.linalg.norm(offset)), 1e-6)
        self.position = drone_pos + offset * min(self.leash / distance, 1.0)
        return self.position, self.velocity


def check(trace_path, model_dir):
    """Replay a trace from the training environment through this module."""
    t = np.load(trace_path)
    steps, n = t["obs"].shape[:2]
    # the integration settings of the task the trace was recorded in
    meta = ast.literal_eval(str(t["meta"])) if "meta" in t.files else {}
    settings = {"step_scale": meta.get("setpoint_step_scale", STEP_SCALE), "leash": meta.get("setpoint_leash", LEASH),
                "max_speed": meta.get("setpoint_max_speed")}
    obs_err = act_err = sp_err = vel_err = 0.0
    observations = [Observation(i, n) for i in range(n)]
    policies = [OnnxPolicy(os.path.join(model_dir, f"policy_falcon{i + 1}.onnx")) for i in range(n)]
    integrators = [SetpointIntegrator(**settings) for _ in range(n)]
    for i in range(n):
        integrators[i].seed(t["sp_pos_before"][0, i])
    for k in range(steps):
        payload_rot, goal_rot = quat_wxyz_to_matrix(t["payload_quat"][k]), quat_wxyz_to_matrix(t["goal_quat"][k])
        for i in range(n):
            obs = observations[i].push(
                payload_pos=t["payload_pos"][k], payload_rot=payload_rot, drone_pos=t["drone_pos"][k, i],
                drone_rot=t["drone_rot"][k, i], drone_lin_vel=t["drone_lin_vel"][k, i],
                drone_ang_vel=t["drone_ang_vel"][k, i], goal_pos=t["goal_pos"][k], goal_rot=goal_rot)
            obs_err = max(obs_err, float(np.abs(obs - t["obs"][k, i]).max()))
            # each check is fed the recorded upstream values, so an error shows up in the stage that made it
            action = policies[i](t["obs"][k, i])
            act_err = max(act_err, float(np.abs(action - t["action"][k, i]).max()))
            position, velocity = integrators[i].step(t["action"][k, i], t["drone_pos"][k, i])
            sp_err = max(sp_err, float(np.abs(position - t["sp_pos"][k, i]).max()))
            vel_err = max(vel_err, float(np.abs(velocity - t["sp_vel"][k, i]).max()))
    rows = [("observation", obs_err, 1e-4), ("policy action", act_err, 1e-4), ("setpoint position", sp_err, 1e-4),
            ("setpoint velocity", vel_err, 1e-3)]
    ok = True
    print(f"{trace_path}: {steps} steps, {n} drones, task {meta.get('task', 'not recorded')}, {settings}")
    for name, err, tol in rows:
        ok &= err <= tol
        print(f"  {name:18s} max |difference| = {err:.2e}  (tolerance {tol:.0e})  {'PASS' if err <= tol else 'FAIL'}")
    return ok


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--trained_on", choices=tuple(TRAINED_ON), default="falcon")
    parser.add_argument("--models", default=None, help="model package directory; default: the --trained_on task's")
    parser.add_argument("--trace", default=None)
    args = parser.parse_args()
    args.models = args.models or models_dir(args.trained_on)
    sys.exit(0 if check(args.trace or os.path.join(args.models, "policy_trace.npz"), args.models) else 1)
