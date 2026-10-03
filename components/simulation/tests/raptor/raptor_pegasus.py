"""Fly the RAPTOR foundation policy on Pegasus Iris quadrotors in Isaac Sim, without PX4.

The controller is RaptorBackend (raptor_backend.py): one per vehicle, each with its own GRU state, mirroring
src/modules/mc_raptor. With the window, the loop still takes one physics step per world.step(render=False) and
redraws with world.render(), so GUI and headless runs step identically and last exactly --duration.

Run:
    source env/env.sh
    python components/simulation/tests/raptor/raptor_pegasus.py --headless --trajectory waypoints
"""

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CASTOR_ROOT = os.environ.get("CASTOR_ROOT") or os.path.abspath(os.path.join(HERE, "../../../.."))
PEGASUS_EXT = os.path.join(CASTOR_ROOT, "components/simulation/pegasus_simulator/extensions/pegasus.simulator")
DEFAULT_POLICY = os.path.join(CASTOR_ROOT, "components/vehicle/PX4-Autopilot/src/modules/mc_raptor/blob/policy.tar")

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
parser.add_argument("--trajectory", choices=["hover", "waypoints", "lissajous"], default="waypoints")
parser.add_argument("--duration", type=float, default=None, help="seconds; default depends on trajectory")
parser.add_argument("--physics_hz", type=float, default=400.0)
parser.add_argument("--num_vehicles", type=int, default=1)
parser.add_argument("--spawn_z", type=float, default=1.0, help="initial height; 0.07 = on the ground")
parser.add_argument("--policy", default=DEFAULT_POLICY)
parser.add_argument("--log", default=None, help="default: runs/<trajectory>_n<num_vehicles>.npz next to this file")
parser.add_argument("--render_hz", type=float, default=50.0, help="GUI redraw rate; physics_hz / render_hz steps/frame")
parser.add_argument("--fast", action="store_true", help="GUI: run as fast as possible instead of real time")
parser.add_argument("--keep_open", action="store_true", help="GUI: pause at the end and keep the window open")
args = parser.parse_args()
if args.log is None:
    args.log = os.path.join(HERE, "runs", f"{args.trajectory}_n{args.num_vehicles}.npz")

# SimulationApp.close() hard-exits, which drops block-buffered output when stdout is a file
sys.stdout.reconfigure(line_buffering=True)

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

# Pegasus 5.1 still imports these deprecated extensions; a standalone app does not enable them for us.
enable_extension("omni.isaac.dynamic_control")
simulation_app.update()

sys.path.insert(0, PEGASUS_EXT)
sys.path.insert(0, os.path.join(HERE, ".deps"))  # pymavlink via pip --target, see README (castor env untouched)
sys.path.insert(0, HERE)

import numpy as np  # noqa: E402
import torch  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api.objects import GroundPlane  # noqa: E402
from isaacsim.core.api.world import World  # noqa: E402
from pxr import UsdGeom, UsdLux  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface  # noqa: E402
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig  # noqa: E402
from pegasus.simulator.params import ROBOTS  # noqa: E402

from raptor_policy import RaptorPolicy  # noqa: E402

from raptor_backend import RaptorBackend, make_trajectory  # noqa: E402
from sim_loop import run_physics  # noqa: E402

torch.set_num_threads(1)


def main():
    trajectory, default_duration = make_trajectory(args.trajectory)
    duration = args.duration or default_duration
    policy = RaptorPolicy(args.policy)

    timeline = omni.timeline.get_timeline_interface()
    pg = PegasusInterface()
    pg._world = World(physics_dt=1.0 / args.physics_hz, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0)
    world = pg.world

    GroundPlane(prim_path="/World/GroundPlane", size=100.0, color=np.array([0.4, 0.4, 0.45]))
    stage = omni.usd.get_context().get_stage()
    UsdLux.DistantLight.Define(stage, "/World/Sun").CreateIntensityAttr(2500.0)
    UsdLux.DomeLight.Define(stage, "/World/Sky").CreateIntensityAttr(600.0)

    backends = []
    for i in range(args.num_vehicles):
        home = [0.0, 2.5 * i, max(args.spawn_z, 1.0)]  # target home is at least 1 m up
        spawn = [0.0, 2.5 * i, args.spawn_z]
        backend = RaptorBackend(policy, trajectory, home, args.physics_hz)
        cfg = MultirotorConfig()
        cfg.backends = [backend]
        cfg.sensors = []
        Multirotor(f"/World/quadrotor{i}", ROBOTS["Iris"], i, spawn, [0.0, 0.0, 0.0, 1.0], config=cfg)
        backends.append(backend)

    world.reset()

    # rotor layout check: mc_raptor assumes PX4 quad X (1 FR, 2 BL, 3 FL, 4 BR)
    cache = UsdGeom.XformCache()
    body = stage.GetPrimAtPath("/World/quadrotor0/body")
    body_inv = cache.GetLocalToWorldTransform(body).GetInverse()
    expected = [(1, -1), (-1, 1), (1, 1), (-1, -1)]  # sign of (x, y) in FLU body
    for r in range(4):
        prim = stage.GetPrimAtPath(f"/World/quadrotor0/rotor{r}")
        xyz = (cache.GetLocalToWorldTransform(prim) * body_inv).ExtractTranslation()
        ok = (np.sign(xyz[0]), np.sign(xyz[1])) == expected[r]
        print(f"[raptor] rotor{r} body xyz = ({xyz[0]:+.3f}, {xyz[1]:+.3f}, {xyz[2]:+.3f})  quad-X match: {ok}")

    timeline.play()
    steps = run_physics(world, simulation_app, duration, args.physics_hz, render=not args.headless,
                        render_hz=args.render_hz, realtime=not args.fast)
    print(f"[raptor] simulated {world.current_time:.4f} s in {steps} physics steps")
    if args.keep_open and not args.headless:
        timeline.pause()
    else:
        timeline.stop()

    os.makedirs(os.path.dirname(args.log), exist_ok=True)
    out = {}
    for i, b in enumerate(backends):
        for k, v in b.log.items():
            out[f"v{i}_{k}"] = np.array(v)
    np.savez(args.log, trajectory=args.trajectory, physics_hz=args.physics_hz, **out)
    print(f"[raptor] log written to {args.log}")

    for i, b in enumerate(backends):
        t = np.array(b.log["t"])
        pos, tgt = np.array(b.log["pos"]), np.array(b.log["target"])
        err = np.linalg.norm(pos - tgt, axis=1)
        tilt = np.degrees(np.arccos(np.clip([Rotation.from_quat(q).as_matrix()[2, 2] for q in b.log["quat"]], -1, 1)))
        print(f"[raptor] vehicle {i}: steps {len(t)}, min z {pos[:, 2].min():.2f} m, max tilt {tilt.max():.1f} deg, "
              f"final err {err[-1]:.3f} m, mean throttle {np.mean(b.log['action']):.3f}")
        if hasattr(trajectory, "waypoints"):
            wps = trajectory.waypoints
            for j, (t0, off, yaw) in enumerate(wps):
                t1 = wps[j + 1][0] if j + 1 < len(wps) else t[-1]
                m = (t > t1 - 1.0) & (t <= t1)
                yaw_err = []
                for q, yt in zip(np.array(b.log["quat"])[m], np.array(b.log["yaw_target"])[m]):
                    yaw_err.append(np.degrees(np.angle(np.exp(1j * (Rotation.from_quat(q).as_euler("ZYX")[0] - yt)))))
                print(f"[raptor]   wp{j} {off} yaw {yaw:.0f}: settled err (last 1 s) {err[m].mean():.3f} m, "
                      f"yaw err {np.mean(np.abs(yaw_err)):.1f} deg")
        else:
            m = t > 3.0
            print(f"[raptor]   tracking RMS after 3 s: {np.sqrt(np.mean(err[m] ** 2)):.3f} m, max {err[m].max():.3f} m")

    if args.keep_open and not args.headless:
        print("[raptor] paused; close the Isaac Sim window to exit")
        while simulation_app.is_running():
            simulation_app.update()
    simulation_app.close()


if __name__ == "__main__":
    # Kit swallows uncaught exceptions and exits 0, so fail loudly ourselves
    try:
        main()
    except Exception:
        import traceback

        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(1)
