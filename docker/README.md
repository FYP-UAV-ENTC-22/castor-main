# CASTOR containers

One image per component, all built from [Dockerfile](Dockerfile) with a
`COMPONENT` build argument. Every container runs ROS 2 Jazzy on the host
network, Fast DDS, `ROS_DOMAIN_ID=20`. DDS never leaves the host: the default
Fast DDS profile [fastdds_localhost.xml](fastdds_localhost.xml) allows only
shared memory and UDP on 127.0.0.1, for the ROS nodes and the XRCE agent alike,
and the bridge's Cyclone DDS is pinned to 127.0.0.1 in the compose file. The
zenoh bridge is the only path between hosts, and it carries only allow-listed
topics.

Don't set `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` in these containers. In that
mode Fast DDS and the bridge each advertise a non-loopback address, and the
bridge never discovers a node that starts after it (the XML has the detail).

For the ROS CLI use `docker compose exec <service> bash`, a `docker run` of any
CASTOR image, or a ROS 2 install on the host itself (Humble or Jazzy, Fast DDS)
after `source docker/host_ros_env.sh`. Measured 2026-10-04, host Humble against
the Jazzy containers and the simulator:

| Host settings | Discovery | Data |
|---|---|---|
| ROS defaults | no | no |
| `fastdds_localhost.xml` (the containers' profile) | yes | no: Fast DDS picks shared memory, and the root-owned segments are closed to you |
| `fastdds_host.xml` (what `host_ros_env.sh` sets) | yes | yes, both ways |

With host networking the host and every container share the `ros2` daemon's
port, so a daemon started by another distro or other settings answers with
`!rclpy.ok()` faults; `host_ros_env.sh` stops it. Containers that are killed
rather than stopped leave their shared-memory segments in `/dev/shm` (277 had
piled up by 2026-10-04); `make dds-shm-clean` removes the ones nothing uses.

Restarts need care because of how the bridge works. It tracks the publishers on
its host by node name, so a node that comes back under the same name before its
crashed predecessor's DDS lease runs out loses its route when that lease expires,
and its topics silently stop crossing to other hosts. Hence:

- Containers stop with **SIGINT** (`STOPSIGNAL` in the image, `stop_signal` in
  compose), the only signal on which `ros2 launch` shuts its nodes down cleanly.
- The Fast DDS lease is 5 s.
- Respawning nodes wait `RESPAWN_DELAY_S` (6 s, `castor_common/launch_helpers.py`).
- A container that Docker restarts waits 6 s before starting ROS.

Keep those numbers in step if you change one.

| Image | Contains | Runs |
|---|---|---|
| `castor-vehicle` | Micro XRCE-DDS Agent v2.4.3, `px4_msgs` from the PX4 submodule, `castor_vehicle_interface`, mavlink-router, QGroundControl 5.1.4, openocd/st-flash | `vehicle_launch vehicle.launch.py` |
| `castor-localization` | heartbeat only (empty for now) | `localization_launch localization.launch.py` |
| `castor-planning` | `castor_policy` + ONNX Runtime 1.30.0 | `planning_launch planning.launch.py` |
| `castor-system` | `castor_supervisor` (YASMIN 6 + BehaviorTree.CPP 4), rosbag2 + MCAP | `system_launch system.launch.py` |

Each is published for `linux/amd64` and `linux/arm64` under one tag:
`ghcr.io/fyp-uav-entc-22/castor-<component>:{main,sha-<rev>,v*}`. An amd64
image does not run on a Pi (`exec format error`); the multi-arch tag makes
each machine pull its own variant.

## Layout (follows ~/rad_autonomy)

```
Makefile                      make <component>-<target>, from the repo root
make/component_colcon.mk      colcon targets shared by every component
docker/
  Dockerfile                  <component>-dev, <component>-final, component-test
  docker-compose.prod.yml     the onboard stack (Pis, or a laptop next to the simulator)
  docker-compose.dev.yml      dev containers with the repo mounted at /home/ws
  build_runtime.sh            build/test/push the -final images (local and CI)
  build_dev.sh                build the -dev images
  castor_env.sh               sourced by every entrypoint: ROS + robot.yaml
.devcontainer/<component>/
  devcontainer.json           VS Code attaches to that compose service
  deps/apt-dependencies.txt   build/dev apt packages rosdep can't express
  deps/apt-runtime-dependencies.txt   runtime apt packages rosdep can't express
  deps/pip-requirements.txt
  deps/install-source-deps.sh source/binary installs into /opt/castor/third_party
components/common/            castor_interfaces, castor_common (in every image)
components/<component>/       entrypoint.sh, Makefile, <component>_launch/, packages, submodules
```

ROS dependencies are never listed by hand. rosdep reads them from each
`package.xml`: all of them for the dev and build stages, only the exec
dependencies for the runtime image (`resolve_runtime_deps.sh`). Submodules are
never part of a build context ([../.dockerignore](../.dockerignore)) or a colcon
workspace ([list_component_packages.sh](list_component_packages.sh)).

## Developing

```bash
make vehicle-dev-image          # castor-vehicle:dev, with your uid
make vehicle-dev-up
make vehicle-dev-shell
# inside, at /home/ws:
make vehicle-build              # -> .component_workspaces/vehicle/install
make vehicle-test
source .component_workspaces/vehicle/install/setup.bash
ros2 launch vehicle_launch vehicle.launch.py
```

Or open `.devcontainer/<component>/` in VS Code ("Reopen in Container").
Dev containers mount `deploy/robot.laptop.yaml` unless `CASTOR_ROBOT_CONFIG`
points elsewhere.

## Running the stack on a laptop

```bash
make images                     # castor-<component>:local for this machine's arch
make images-test                # the same, after running every component's colcon tests
make stack-up                   # docker-compose.prod.yml with :local and robot.laptop.yaml
docker compose -f docker/docker-compose.prod.yml exec system bash
ros2 topic echo /drone1/system/state
make vehicle-qgc                # QGroundControl from the vehicle container, on your display
make stack-down
```

## Building for the Pis

CI does this on every merge to `main` (see
[../.github/workflows/images.yml](../.github/workflows/images.yml)): native
amd64 and arm64 runners, colcon tests first, then a multi-arch tag. To build
arm64 on an x86 machine yourself you need QEMU (`docker run --privileged --rm
tonistiigi/binfmt --install arm64`); expect C++ builds to be several times
slower than native.

```bash
docker/build_runtime.sh --platform linux/arm64 --push --tag test-mine vehicle   # after docker login ghcr.io
```

## Images are always tagged

No build leaves an untagged image behind. Onboard images are built by the
`castor` buildx builder, so base images stay in its cache, not in Docker's image
list; a rebuilt `:local` or `:dev` removes the image it replaced, by ID, once
that image is dangling; `castor-update` on a Pi does the same for what it
replaced. The zenoh bridge comes from our mirror,
`ghcr.io/fyp-uav-entc-22/zenoh-bridge-ros2dds:1.10.1` (the upstream image,
unchanged, under a tag). CI pushes per-arch `sha-<rev>-<arch>` tags and the
multi-arch `main` / `sha-<rev>`, with no attestations; its build cache lives in
the GitHub Actions cache, not on GHCR.

Containers share the host's PID namespace (`pid: host`), so `pkill -f` on the
host matches processes in every container. Stop things by exact PID.

The runtime images still carry about 135 MB of `-dev` packages. They don't come
from our rosdep keys: `ros-core` itself depends on 19 of them, and the system
image adds 18 through upstream Debian `Depends` of behaviortree-cpp, yasmin and
the rosbag2 vendor packages (libzmq3-dev pulls libicu-dev, 49 MB). Removing them
means force-removing packages apt considers required, so they stay.

## Simulation image

One image for everything that needs the GPU: Isaac Sim 5.1, Isaac Lab, Pegasus,
PX4 SITL, ROS 2 Jazzy and training (`simulation` target, `castor-simulation:local`).
It is built locally and never pushed: it contains NVIDIA's Isaac Sim layers, and
NVIDIA's licence does not allow redistributing them. Each machine pulls
`nvcr.io/nvidia/isaac-sim:5.1.0` itself (the build script checks its digest).

```bash
make sim-image                  # docker/build_simulation.sh; 27.4 GB
make sim-up                     # headless container, repo at /home/ws, GPU, host network
make sim-shell
make sim-train-smoke            # 3 MAPPO iterations on the flycrane hover task
make sim-px4                    # builds px4_sitl_default and px4_sitl_raptor into the PX4 checkout
make sim-pegasus-ros2 DRONES=2  # Pegasus S500s publishing drone<i>/state/*, /sensors/* into the graph
make sim-gui                    # Isaac Sim on your display
make sim-own                    # give files the container wrote into the repo back to you
make sim-down
```

- The container runs as root, like the onboard ones (Fast DDS shared memory
  between them needs the same uid). The `sim-*` targets that write into the
  repo (training logs, PX4 builds) end with `sim-own`.
- Python packages from the repo (Isaac Lab, Pegasus, skrl, the MARL ext) are not
  in the image: a `.pth` file points Isaac's Python at `/home/ws`, so a code
  change needs no rebuild. Their dependencies are baked in, resolved from their
  metadata files only, so a source edit doesn't invalidate those layers either.
- Isaac's own processes (`/isaac-sim/python.sh`, `isaac-sim.sh`) use the ROS 2
  Jazzy that ships with the Isaac ROS bridge (Python 3.11). It is added to
  `LD_LIBRARY_PATH` only for them, by `setup_python_env.sh`; set globally, its
  libcrypto breaks apt and curl. `ros2` in the container is the system Jazzy
  (Python 3.12). Both are Fast DDS 2.14 on domain 20 with the localhost profile,
  so the simulator and the onboard containers share one graph.
- Assets come from the local pack (`CASTOR_ASSET_PACK`, default
  `/mnt/isaac/isaacsim_assets`), mounted read-only; the entrypoint points Kit's
  asset root at it.
- `ACCEPT_EULA=Y` is set in the compose file, so starting the container means
  you accept NVIDIA's EULA. `PRIVACY_CONSENT` is not set.

### Software in the loop

```bash
components/simulation/sil/sil.sh up --drones 3      # one onboard stack per drone + a ground-station bridge
docker compose -f docker/docker-compose.sim.yml exec simulation \
  /isaac-sim/python.sh components/simulation/sil/sil_pegasus.py --headless --drones 3
components/simulation/sil/sil.sh status | down
```

Each drone's stack is the Pi's compose file under its own project
(`castor-sil-drone<i>`), on its own ROS domain (20 + i), with its own XRCE agent
port (8887 + i) and zenoh bridge (127.0.0.1:7447 + i) connected to the ground
station's (domain 20). Robots never share a DDS domain, so everything between
them crosses zenoh with the flight allow-lists. `sil_pegasus.py` flies one S500
per stack, each with its own PX4 SITL instance pointed at that stack's domain and
agent port (`castor_px4.py`), and holds the simulation to real time. The XRCE
agent binds UDP on all interfaces (v2.4.3 has no bind option), so on a shared
network firewall ports 8888-8899.

## Adding things

- **A ROS package:** put it under `components/<component>/` with a
  `package.xml`; it is picked up automatically. Add it to that component's
  launch file.
- **A ROS dependency:** add it to `package.xml`. Nothing else.
- **A system library or tool:** `.devcontainer/<component>/deps/apt-dependencies.txt`
  (build) and, if needed at run time, `apt-runtime-dependencies.txt`.
- **Something built from source:** `.devcontainer/<component>/deps/install-source-deps.sh`,
  installing under `/opt/castor/third_party`. Put PATH or library settings in
  `/opt/castor/third_party/env.d/<name>.sh`.
- **A topic other drones or the ground station need:** add it to the allow-list
  in `components/common/castor_common/castor_common/bridge_config.py`.
