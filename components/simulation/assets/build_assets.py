"""Generate the S500 vehicle USD and a standalone payload-rig assembly from the YAML configs.

    source env/env.sh
    cd components/simulation/assets
    python build_assets.py                                   # config/payload_rig.yaml as is
    python build_assets.py --set num_drones=4 --set cable.model=rope --pin_drones
    python build_assets.py --vset mass.total=1.62            # vehicle override (measured all-up mass)

Writes generated/s500.usd and generated/rig_<vehicle>_n<N>_<cable>.usd (or --out). Open the rig file in Isaac Sim
(File > Open) and press Play: with --pin_drones each drone is fixed to the world and the payload hangs; without it
nothing drives the rotors, so everything falls. To fly it, use components/simulation/tests/raptor/raptor_payload.py.
"""

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from castor_assets import config as C  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--rig", default="payload_rig.yaml", help="rig config (path, or a name in config/)")
parser.add_argument("--vehicle", default=None, help="vehicle config; default: the rig's vehicle_config")
parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE", help="rig override, repeatable")
parser.add_argument("--vset", action="append", default=[], metavar="KEY=VALUE", help="vehicle override, repeatable")
parser.add_argument("--out", default=None, help="rig USD path; default generated/rig_<vehicle>_n<N>_<cable>.usd")
parser.add_argument("--pin_drones", action="store_true", help="fix each drone to the world so the payload hangs")
parser.add_argument("--vehicle_only", action="store_true", help="only write the vehicle USD")
args = parser.parse_args()

# fail on config mistakes before paying for an Isaac Sim start-up
rig = C.load_rig(args.rig, args.set)
vehicle = C.load_vehicle(args.vehicle or rig.vehicle_config, args.vset)

sys.stdout.reconfigure(line_buffering=True)
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})


def main():
    from castor_assets import usd_build as U

    os.makedirs(C.GENERATED_DIR, exist_ok=True)
    iris = os.path.join(HERE, "../pegasus_simulator/extensions/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd")
    vinfo = U.vehicle_info(rig, vehicle, C.GENERATED_DIR, os.path.abspath(iris))
    if vinfo.kind == "s500":
        print(f"[assets] vehicle {vinfo.usd_path}")
        print(json.dumps(vinfo.summary, indent=2))
    if args.vehicle_only:
        return
    out = args.out or os.path.join(C.GENERATED_DIR, f"rig_{rig.vehicle}_n{rig.num_drones}_{rig.cable.model}.usd")
    handles, warnings = U.build_rig_file(rig, vinfo, out, pin_drones=args.pin_drones)
    lay = handles.layout
    print(f"[assets] rig {out}")
    print(f"[assets]   {rig.num_drones} x {rig.vehicle}, cable {rig.cable.model} {rig.cable.length} m, "
          f"anchor radius {lay.anchor_radius:.3f} m, drones at {lay.horizontal_distance:.3f} m, "
          f"cable angle {lay.cable_angle * 57.29578:.1f} deg, rotor clearance "
          f"{lay.rotor_clearance if lay.n > 1 else float('nan'):.3f} m")
    print(f"[assets]   static tension {lay.static_tension:.2f} N per cable, hover needs "
          f"{100 * lay.hover_throttle_share:.0f}% of full thrust per drone")
    for w in warnings:
        print(f"[assets] WARNING {w}")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback

        traceback.print_exc()
        sys.stdout.flush()
        os._exit(1)
    app.close()
