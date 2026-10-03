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
bridge never discovers a node that starts after it (the XML has the detail). For
the ROS CLI use `docker compose exec <service> bash` or a `docker run` of any
CASTOR image; a ROS install on the host itself (another distro, other DDS
settings) is not guaranteed to see the containers' graph.

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
