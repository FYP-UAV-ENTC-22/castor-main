"""castor-config: check robot.yaml and derive what the containers need from it.

    castor-config check                 validate; exit 2 with every problem listed
    castor-config env                   print `export CASTOR_...=` lines for the entrypoints
    castor-config show                  print the parsed config, defaults filled in
    castor-config zenoh-bridge --out F  write the zenoh bridge config for this host
"""

from __future__ import annotations

import argparse
import dataclasses
import shlex
import sys

import yaml

from . import bridge_config
from .robot_config import ConfigError, config_path, load


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="castor-config", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", help="robot.yaml path (default: $CASTOR_ROBOT_CONFIG or /etc/castor/robot.yaml)")
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check")
    sub.add_parser("env")
    sub.add_parser("show")
    zb = sub.add_parser("zenoh-bridge")
    zb.add_argument("--out", required=True)
    args = parser.parse_args(argv)

    try:
        cfg = load(args.config)
    except ConfigError as e:
        print(f"castor-config: {e}", file=sys.stderr)
        return 2

    if args.cmd == "check":
        print(f"{config_path(args.config)} OK: robot {cfg.robot_id} '{cfg.namespace}' on {cfg.hardware}, "
              f"team slot {cfg.team_index}/{cfg.team_size}, fc {'on' if cfg.fc.enabled else 'off'}")
    elif args.cmd == "env":
        for key, value in cfg.env().items():
            print(f"export {key}={shlex.quote(value)}")
    elif args.cmd == "show":
        d = dataclasses.asdict(cfg)
        d["one_hot"] = cfg.one_hot()
        print(yaml.safe_dump(d, sort_keys=False), end="")
    elif args.cmd == "zenoh-bridge":
        bridge_config.write(cfg, args.out)
        print(f"wrote {args.out} (namespace '{cfg.namespace}', connect {list(cfg.zenoh.connect) or 'none'})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
