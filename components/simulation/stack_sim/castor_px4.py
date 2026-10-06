"""PX4 SITL launching for stack_sim, without patching the Pegasus fork.

Pegasus' PX4LaunchTool runs every PX4 instance with Isaac Sim's own environment
(it even mutates os.environ), from build/px4_sitl_default, in a throwaway temp
directory. In stack_sim each instance needs its own environment (ROS_DOMAIN_ID,
PX4_UXRCE_DDS_PORT, PX4_UXRCE_DDS_NS: see stack_sim.sh), may need the RAPTOR build,
and RAPTOR loads ./raptor/policy.tar from the working directory. This module
swaps in a launch tool that does that:

    import castor_px4
    castor_px4.install(px4_env_file=".stack_sim/px4.env", build="px4_sitl_raptor")
    # ...then create Pegasus vehicles with PX4MavlinkBackend as usual
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

_instances: dict[int, dict[str, str]] = {}
_build = "px4_sitl_default"
_workdir_root = Path("/tmp/castor-px4")
_extra: dict[str, str] = {}
_fresh_params = True


def read_env_file(path: str) -> dict[int, dict[str, str]]:
    """stack_sim.sh's px4.env: '<instance> KEY=value KEY=value ...' per line."""
    out: dict[int, dict[str, str]] = {}
    for line in Path(path).read_text().splitlines():
        if not line.strip():
            continue
        idx, *pairs = line.split()
        out[int(idx)] = dict(p.split("=", 1) for p in pairs)
    return out


class CastorPX4LaunchTool:
    """Drop-in for pegasus' PX4LaunchTool (same constructor and methods)."""

    def __init__(self, px4_dir, vehicle_id: int = 0, px4_model: str = "gazebo-classic_iris"):
        self.px4_dir = px4_dir
        self.vehicle_id = vehicle_id
        self.rc_script = f"{px4_dir}/ROMFS/px4fmu_common/init.d-posix/rcS"
        self.px4_process = None
        # A copy: never touch Isaac Sim's own environment.
        self.environment = dict(os.environ)
        self.environment["PX4_SIM_MODEL"] = px4_model
        self.environment.update(_extra)
        self.environment.update(_instances.get(vehicle_id, {}))
        # Persistent per-instance working dir (RAPTOR finds its policy there). Saved parameters are dropped at
        # launch unless install(fresh_params=False): PX4_PARAM_* is applied after they load, but anything changed
        # by hand in an earlier run (QGC, pxh) would otherwise carry over.
        self.workdir = _workdir_root / f"instance_{vehicle_id}"
        self.workdir.mkdir(parents=True, exist_ok=True)
        policy = Path(px4_dir) / "src/modules/mc_raptor/blob/policy.tar"
        if _build == "px4_sitl_raptor" and policy.exists():
            (self.workdir / "raptor").mkdir(exist_ok=True)
            shutil.copy2(policy, self.workdir / "raptor" / "policy.tar")

    def launch_px4(self):
        binary = f"{self.px4_dir}/build/{_build}/bin/px4"
        if not os.path.exists(binary):
            raise FileNotFoundError(f"{binary} not found: build it with `make sim-px4`")
        self._kill_stale()
        if _fresh_params:
            for saved in ("parameters.bson", "parameters_backup.bson"):
                (self.workdir / saved).unlink(missing_ok=True)
        log = open(self.workdir / "px4.log", "ab")
        self.px4_process = subprocess.Popen(
            [binary, f"{self.px4_dir}/ROMFS/px4fmu_common/", "-s", self.rc_script, "-i", str(self.vehicle_id), "-d"],
            cwd=self.workdir, env=self.environment, stdout=log, stderr=subprocess.STDOUT)
        (self.workdir / "px4.pid").write_text(str(self.px4_process.pid))

    def _kill_stale(self):
        """A PX4 left behind by a simulator that died without stopping it would keep this instance's ports."""
        pidfile = self.workdir / "px4.pid"
        try:
            pid = int(pidfile.read_text())
            cmdline = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
        except (OSError, ValueError):
            return
        if cmdline and cmdline[0].endswith(b"/bin/px4") and str(self.vehicle_id).encode() in cmdline:
            os.kill(pid, 9)
            print(f"[castor_px4] killed a stale PX4 instance {self.vehicle_id} (pid {pid})")

    def kill_px4(self):
        if self.px4_process is not None:
            self.px4_process.kill()
            self.px4_process.wait(timeout=10)
            self.px4_process = None

    def __del__(self):
        self.kill_px4()


def stack_env(instance: int) -> dict[str, str]:
    """What stack_sim.sh up writes for PX4 instance i (robot i + 1): its domain, its agent port, no /fmu namespace."""
    return {"ROS_DOMAIN_ID": str(21 + instance), "PX4_UXRCE_DDS_PORT": str(8888 + instance), "PX4_UXRCE_DDS_NS": ""}


def install(px4_env_file: str | None = None, build: str = "px4_sitl_default",
            workdir_root: str | None = None, extra_env: dict[str, str] | None = None,
            instances: dict[int, dict[str, str]] | None = None, fresh_params: bool = True) -> None:
    """Make Pegasus' PX4 backend use CastorPX4LaunchTool. Call before vehicles start.

    instances: per-instance environment, used instead of px4_env_file.
    fresh_params: start every instance from its airframe defaults plus extra_env, dropping saved parameters."""
    global _instances, _build, _workdir_root, _extra, _fresh_params
    from pegasus.simulator.logic.backends import px4_mavlink_backend

    if instances is not None:
        _instances = {i: dict(env) for i, env in instances.items()}
    else:
        _instances = read_env_file(px4_env_file) if px4_env_file else {}
    _build = build
    _extra = dict(extra_env or {})
    _fresh_params = fresh_params
    if workdir_root:
        _workdir_root = Path(workdir_root)
    px4_mavlink_backend.PX4LaunchTool = CastorPX4LaunchTool
