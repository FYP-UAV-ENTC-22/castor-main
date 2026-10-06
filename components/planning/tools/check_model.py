"""Check a policy model package (models/<name>/<version>) against the trace recorded in the training environment.

It rebuilds, in NumPy, what the training task does around the network (the reference for the onboard C++ runner in
castor_policy), replays the package's policy_trace.npz through it and compares every stage:

  observation         one 45-value frame per drone, three frames of history, oldest first -> 135 values
  policy action       each slot's ONNX file from model.yaml policy.slots (sha256 checked); its input scaler is inside
  setpoint            action * step scale added to a persistent position setpoint, the step no longer than
                      max speed * dt, velocity = increment / dt, the setpoint kept within the leash of the drone

    frame = [payload position (3), payload rotation (9, row-major), one-hot slot (3),
             drone position (3), drone rotation (9), drone linear velocity (3), drone angular velocity (3),
             goal position - payload position (3), goal rotation @ payload rotation^T (9)]

The trace comes from scripts/tools/capture_policy_trace.py in the MARL repo. Run in the castor conda env (numpy,
pyyaml, onnxruntime); exits 1 on a mismatch:

    python components/planning/tools/check_model.py                       # the package models/DEFAULT names
    python components/planning/tools/check_model.py models/<name>/<version>
"""

import argparse
import ast
import hashlib
import os
import sys

import numpy as np
import yaml

MODELS_ROOT = os.environ.get("CASTOR_MODELS_ROOT") or os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../models"))

# What the trace's meta falls back to when it does not record them (the Falcon task's training settings).
STEP_SCALE = 0.05  # m per policy step at unit action (setpoint_step_scale)
LEASH = 1.5  # m (setpoint_leash)
POLICY_DT = 0.02  # s: the policy runs at 50 Hz


def quat_wxyz_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
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

    def __init__(self, slot, num_drones, history):
        self.slot, self.num_drones, self.length = slot, num_drones, history
        self.history = None

    def push(self, **state):
        f = frame(self.slot, self.num_drones, **state)
        self.history = [f] * self.length if self.history is None else self.history[1:] + [f]
        return np.concatenate(self.history)


class OnnxPolicy:
    """The exported actor of one slot: raw observation in, mean action out."""

    def __init__(self, path, obs_dim):
        import onnxruntime

        options = onnxruntime.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.session = onnxruntime.InferenceSession(path, options, providers=["CPUExecutionProvider"])
        width = self.session.get_inputs()[0].shape[-1]
        if width != obs_dim:
            raise ValueError(f"{path} takes {width} inputs, model.yaml says {obs_dim}")

    def __call__(self, observation):
        return self.session.run(["action"], {"observation": observation[None].astype(np.float32)})[0][0]


class SetpointIntegrator:
    """The position setpoint of one drone: absolute, persistent, moved by the policy's actions."""

    def __init__(self, step_scale, leash, max_speed, dt=POLICY_DT):
        self.step_scale, self.leash, self.dt, self.max_speed = step_scale, leash, dt, max_speed
        self.position = np.zeros(3)

    def seed(self, drone_pos):
        self.position = np.array(drone_pos, dtype=float)

    def step(self, action, drone_pos):
        increment = np.asarray(action, dtype=float) * self.step_scale
        if self.max_speed is not None:
            # actions are unbounded: the step scale is only the speed at unit action
            step = max(float(np.linalg.norm(increment)), 1e-9)
            increment = increment * min(self.max_speed * self.dt / step, 1.0)
        self.position = self.position + increment
        offset = self.position - drone_pos
        distance = max(float(np.linalg.norm(offset)), 1e-6)
        self.position = drone_pos + offset * min(self.leash / distance, 1.0)
        return self.position, increment / self.dt


def sha256(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def check(model_dir, trace_path):
    with open(os.path.join(model_dir, "model.yaml")) as f:
        policy = yaml.safe_load(f)["policy"]
    ok = True
    for i, s in enumerate(policy["slots"]):
        good = sha256(os.path.join(model_dir, s["file"])) == s["sha256"]
        ok &= good
        print(f"  slot {i}: {s['file']} sha256 {'PASS' if good else 'FAIL (model.yaml has another hash)'}")

    t = np.load(trace_path)
    steps, n = t["obs"].shape[:2]
    meta = ast.literal_eval(str(t["meta"])) if "meta" in t.files else {}
    settings = {"step_scale": meta.get("setpoint_step_scale", STEP_SCALE), "leash": meta.get("setpoint_leash", LEASH),
                "max_speed": meta.get("setpoint_max_speed")}
    history, obs_dim = policy["history"], policy["frame_dim"] * policy["history"]
    observations = [Observation(i, n, history) for i in range(n)]
    policies = [OnnxPolicy(os.path.join(model_dir, policy["slots"][i]["file"]), obs_dim) for i in range(n)]
    integrators = [SetpointIntegrator(**settings) for _ in range(n)]
    for i in range(n):
        integrators[i].seed(t["sp_pos_before"][0, i])
    obs_err = act_err = sp_err = vel_err = 0.0
    for k in range(steps):
        payload_rot, goal_rot = quat_wxyz_to_matrix(t["payload_quat"][k]), quat_wxyz_to_matrix(t["goal_quat"][k])
        for i in range(n):
            obs = observations[i].push(
                payload_pos=t["payload_pos"][k], payload_rot=payload_rot, drone_pos=t["drone_pos"][k, i],
                drone_rot=t["drone_rot"][k, i], drone_lin_vel=t["drone_lin_vel"][k, i],
                drone_ang_vel=t["drone_ang_vel"][k, i], goal_pos=t["goal_pos"][k], goal_rot=goal_rot)
            obs_err = max(obs_err, float(np.abs(obs - t["obs"][k, i]).max()))
            # each check is fed the recorded upstream values, so an error shows up in the stage that made it
            act_err = max(act_err, float(np.abs(policies[i](t["obs"][k, i]) - t["action"][k, i]).max()))
            position, velocity = integrators[i].step(t["action"][k, i], t["drone_pos"][k, i])
            sp_err = max(sp_err, float(np.abs(position - t["sp_pos"][k, i]).max()))
            vel_err = max(vel_err, float(np.abs(velocity - t["sp_vel"][k, i]).max()))
    print(f"  trace: {steps} steps, {n} drones, task {meta.get('task', 'not recorded')}, {settings}")
    for name, err, tol in [("observation", obs_err, 1e-4), ("policy action", act_err, 1e-4),
                           ("setpoint position", sp_err, 1e-4), ("setpoint velocity", vel_err, 1e-3)]:
        ok &= err <= tol
        print(f"  {name:18s} max |difference| = {err:.2e}  (tolerance {tol:.0e})  {'PASS' if err <= tol else 'FAIL'}")
    return ok


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("model", nargs="?", default=None, help="model package directory; default: models/DEFAULT's")
    parser.add_argument("--trace", default=None, help="default: the package's policy_trace.npz")
    args = parser.parse_args()
    if args.model is None:
        with open(os.path.join(MODELS_ROOT, "DEFAULT")) as f:
            args.model = os.path.join(MODELS_ROOT, f.read().strip())
    print(args.model)
    sys.exit(0 if check(args.model, args.trace or os.path.join(args.model, "policy_trace.npz")) else 1)
