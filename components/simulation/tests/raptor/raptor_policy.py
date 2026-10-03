"""Batched PyTorch port of the RAPTOR foundation policy (rl_tools checkpoint).

Architecture read from policy.tar metadata:
    Dense(22 -> 16, ReLU) -> GRU(16) -> Dense(16 -> 4, identity), outputs clamped to [-1, 1].

rl_tools' GRU uses the PyTorch gate convention ([r, z, n], n = tanh(W_in x + b_in + r * (W_hn h + b_hn)),
h' = (1 - z) * n + z * h), so torch.nn.GRUCell takes the weights unchanged. FAST_TANH is false in this checkpoint.

Observation (22), all in the target-yaw-aligned FLU frame:
    [0:3]   position error (p - p_target), clipped to +-0.5 m
    [3:12]  rotation matrix of q_target^-1 * q, row-major
    [12:15] linear velocity error, clipped to +-1.0 m/s
    [15:18] body angular velocity (FLU)
    [18:22] previous action, [-1, 1], Crazyflie motor order
Action (4): motor throttle in [-1, 1] -> (a + 1) / 2 in [0, 1], Crazyflie order
    (PX4 remaps: px4[0]=a[0], px4[1]=a[2], px4[2]=a[3], px4[3]=a[1]).
"""

import tarfile

import numpy as np
import torch
import torch.nn as nn


def _read_tar(path):
    tensors = {}
    with tarfile.open(path) as tar:
        members = {m.name: m for m in tar.getmembers()}
        for name in members:
            if not name.endswith("/data"):
                continue
            base = name[: -len("/data")]
            meta = tar.extractfile(members[base + "/meta"]).read().decode()
            fields = dict(line.split(": ", 1) for line in meta.strip().splitlines())
            assert fields["dtype"] == "float32", fields
            shape = [int(fields[f"dim_{i}"]) for i in range(int(fields["num_dims"]))]
            raw = tar.extractfile(members[name]).read()
            tensors[base] = np.frombuffer(raw, dtype="<f4").reshape(shape).copy()
    return tensors


class RaptorPolicy(nn.Module):
    OBS_DIM = 22
    ACTION_DIM = 4
    HIDDEN_DIM = 16
    HOVER_THROTTLE = 0.66  # rl_tools l2f executor seeds the action history with this

    def __init__(self, tar_path):
        super().__init__()
        t = _read_tar(tar_path)
        p = "actor/layers"
        self.input_layer = nn.Linear(self.OBS_DIM, self.HIDDEN_DIM)
        self.gru = nn.GRUCell(self.HIDDEN_DIM, self.HIDDEN_DIM)
        self.output_layer = nn.Linear(self.HIDDEN_DIM, self.ACTION_DIM)
        with torch.no_grad():
            self.input_layer.weight.copy_(torch.from_numpy(t[f"{p}/0/weights/parameters"]))
            self.input_layer.bias.copy_(torch.from_numpy(t[f"{p}/0/biases/parameters"]))
            self.gru.weight_ih.copy_(torch.from_numpy(t[f"{p}/1/weights_input/parameters"]))
            self.gru.weight_hh.copy_(torch.from_numpy(t[f"{p}/1/weights_hidden/parameters"]))
            self.gru.bias_ih.copy_(torch.from_numpy(t[f"{p}/1/biases_input/parameters"]))
            self.gru.bias_hh.copy_(torch.from_numpy(t[f"{p}/1/biases_hidden/parameters"]))
            self.output_layer.weight.copy_(torch.from_numpy(t[f"{p}/2/weights/parameters"]))
            self.output_layer.bias.copy_(torch.from_numpy(t[f"{p}/2/biases/parameters"]))
        self.register_buffer("initial_hidden", torch.from_numpy(t[f"{p}/1/initial_hidden_state/parameters"]))
        self.example_input = torch.from_numpy(t["example/input"])
        self.example_output = torch.from_numpy(t["example/output"])
        self.requires_grad_(False)

    def initial_state(self, batch):
        return self.initial_hidden.expand(batch, -1).clone()

    def forward(self, obs, hidden):
        """One raw step: returns (unclamped action, new hidden)."""
        hidden = self.gru(torch.relu(self.input_layer(obs)), hidden)
        return self.output_layer(hidden), hidden


def verify(policy):
    """Replay the checkpoint's own example sequence ([500, 2, 22] -> [500, 2, 4]) with the hidden state carried
    across steps and return the max abs difference to the stored outputs."""
    x, y = policy.example_input, policy.example_output
    h = policy.initial_state(x.shape[1])
    out = []
    for t in range(x.shape[0]):
        a, h = policy(x[t], h)
        out.append(a)
    return (torch.stack(out) - y).abs().max().item()


if __name__ == "__main__":
    import os
    import sys

    here = os.path.dirname(os.path.abspath(__file__))
    root = os.environ.get("CASTOR_ROOT") or os.path.abspath(os.path.join(here, "../../../.."))
    default = os.path.join(root, "components/vehicle/PX4-Autopilot/src/modules/mc_raptor/blob/policy.tar")
    path = sys.argv[1] if len(sys.argv) > 1 else default

    max_err = verify(RaptorPolicy(path))
    ok = max_err < 1e-5  # same threshold mc_raptor's test_policy uses
    print(f"{path}\nmax |torch - checkpoint example| = {max_err:.3e}  {'PASS' if ok else 'FAIL'}")
    sys.exit(0 if ok else 1)
