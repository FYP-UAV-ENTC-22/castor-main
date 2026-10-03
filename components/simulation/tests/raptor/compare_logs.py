"""Compare two runs step by step, e.g. the same scenario with and without the Isaac Sim window.

    python compare_logs.py runs/a.npz runs/b.npz

Works on raptor_payload.py and raptor_pegasus.py logs. Reports log lengths, simulated time and the largest
difference in every drone's and the payload's trajectory. No Isaac Sim needed.
"""

import json
import sys

import numpy as np


def load(path):
    d = np.load(path, allow_pickle=True)
    meta = json.loads(str(d["meta"])) if "meta" in d.files else {}
    return d, meta


def main(a_path, b_path):
    a, ma = load(a_path)
    b, mb = load(b_path)
    for name, m, d in (("A", ma, a), ("B", mb, b)):
        drones = sorted({k.split("_")[0] for k in d.files if k.startswith("v") and k.endswith("_pos")})
        lens = {k: len(d[f"{k}_t"]) for k in drones}
        extra = f", rig {len(d['rig_t'])} steps" if "rig_t" in d.files else ""
        sim = m.get("sim_time")
        print(f"{name}: {a_path if name == 'A' else b_path}\n   sim time {sim if sim is None else round(sim, 6)} s, "
              f"headless {m.get('args', {}).get('headless')}, drone log lengths {sorted(set(lens.values()))}{extra}")
    keys = sorted(k for k in a.files if k.endswith("_pos") and k in b.files and a[k].ndim >= 2)
    worst = 0.0
    for k in keys:
        n = min(len(a[k]), len(b[k]))
        diff = np.abs(a[k][:n] - b[k][:n])
        mx = float(np.nanmax(diff)) if diff.size else float("nan")
        worst = max(worst, mx)
        where = np.unravel_index(int(np.nanargmax(diff)), diff.shape)[0] if diff.size else -1
        print(f"   {k:22s} steps compared {n:6d}  max |A - B| {mx:.3e} m (at step {where}); lengths {len(a[k])} vs {len(b[k])}")
    for k in ("rig_tension_newton", "v0_action"):
        if k in a.files and k in b.files:
            n = min(len(a[k]), len(b[k]))
            print(f"   {k:22s} max |A - B| {float(np.nanmax(np.abs(a[k][:n] - b[k][:n]))):.3e}")
    print(f"largest position difference: {worst:.3e} m {'(bit-identical)' if worst == 0.0 else ''}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(2)
    main(sys.argv[1], sys.argv[2])
