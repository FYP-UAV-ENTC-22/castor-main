"""Export a trained skrl MAPPO/IPPO actor to the ONNX model the onboard runner loads.

The graph takes raw observations: skrl's RunningStandardScaler (the agent's
state_preprocessor) is baked in front of the MLP, so the Pi feeds exactly what
the environment would. The output is the Gaussian policy's mean action (no
sampling, no log_std); any clipping/scaling of actions stays where the
environment does it. Layer sizes and the activation must match the agent cfg
(skrl_mappo_cfg.yaml: [1024, 512, 256, 128], elu). Run in the castor conda env:

    python components/planning/tools/export_policy.py best_agent.pt -o policy.onnx
"""

import argparse

import numpy as np
import torch
from torch import nn

ACTIVATIONS = {"elu": nn.ELU, "relu": nn.ReLU, "tanh": nn.Tanh, "leaky_relu": nn.LeakyReLU}


class Actor(nn.Module):
    def __init__(self, policy: dict, scaler: dict | None, activation: str, clip: float, eps: float):
        super().__init__()
        idx = sorted({int(k.split(".")[1]) for k in policy if k.startswith("net_container.")})
        linears = [nn.Linear(*policy[f"net_container.{i}.weight"].shape[::-1]) for i in idx]
        mods = []
        for i, lin in zip(idx, linears):
            lin.weight.data.copy_(policy[f"net_container.{i}.weight"])
            lin.bias.data.copy_(policy[f"net_container.{i}.bias"])
            mods += [lin, ACTIVATIONS[activation]()]
        self.net = nn.Sequential(*mods[:-1])  # no activation after the output layer
        width = linears[0].in_features
        mean = scaler["running_mean"].float() if scaler else torch.zeros(width)
        std = scaler["running_variance"].float().sqrt() + eps if scaler else torch.ones(width)
        self.register_buffer("mean", mean)
        self.register_buffer("std", std)
        self.clip = clip if scaler else float("inf")

    def forward(self, obs):
        return self.net(torch.clamp((obs - self.mean) / self.std, -self.clip, self.clip))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("checkpoint", help="skrl checkpoint (.pt), e.g. runs/.../checkpoints/best_agent.pt")
    ap.add_argument("--agent", help="agent key in the checkpoint (default: the first, e.g. falcon1)")
    ap.add_argument("--activation", default="elu", choices=sorted(ACTIVATIONS))
    ap.add_argument("--clip", type=float, default=5.0, help="RunningStandardScaler clip_threshold")
    ap.add_argument("--epsilon", type=float, default=1e-8, help="RunningStandardScaler epsilon")
    ap.add_argument("-o", "--out", default="policy.onnx")
    args = ap.parse_args()

    ck = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    agents = [k for k, v in ck.items() if isinstance(v, dict) and "policy" in v] or [None]
    agent = args.agent or agents[0]
    entry = ck[agent] if agent else ck
    shared = agent and all(
        all(torch.equal(ck[a]["policy"][k], entry["policy"][k]) for k in entry["policy"]) for a in agents)
    scaler = entry.get("state_preprocessor") or None
    actor = Actor(entry["policy"], scaler, args.activation, args.clip, args.epsilon).eval()

    width = actor.mean.numel()
    torch.onnx.export(actor, torch.zeros(1, width), args.out, input_names=["obs"], output_names=["actions"],
                      opset_version=17, dynamo=False)

    # Same numbers from ONNX Runtime as from torch, on observations around the training distribution.
    x = (actor.mean + actor.std * torch.randn(256, width)).float()
    with torch.no_grad():
        ref = actor(x).numpy()
    try:
        import onnxruntime as ort
    except ImportError:
        ort, err = None, None
    else:
        sess = ort.InferenceSession(args.out, providers=["CPUExecutionProvider"])
        got = np.concatenate([sess.run(None, {"obs": row[None].numpy()})[0] for row in x])
        err = float(np.abs(got - ref).max())

    params = sum(p.numel() for p in actor.parameters())
    print(f"wrote {args.out} from {args.checkpoint} [{agent or 'single agent'}]")
    print(f"  input [1, {width}] raw observations, output [1, {ref.shape[1]}] mean actions, {params} parameters")
    print(f"  state preprocessor: {'baked in' if scaler else 'none in checkpoint'}; "
          f"agents {agents}: {'one shared policy' if shared else 'policies differ, exported ' + str(agent)}")
    if err is None:
        print("  onnxruntime not installed: output not checked against ONNX Runtime")
    else:
        print(f"  torch vs onnxruntime max abs diff {err:.2e}")
    if err is not None and err > 1e-4:
        raise SystemExit("ONNX output does not match torch")


if __name__ == "__main__":
    main()
