# syntax=docker/dockerfile:1.7
#
# CASTOR images: one Dockerfile, one COMPONENT build argument.
#
#   <component>-dev    build tools and every dependency; the repo is mounted at
#                      /home/ws by docker-compose.dev.yml and built with make
#   <component>-final  the runtime image pushed to GHCR and run on the Pis
#   component-test     builds the component with tests on and runs colcon test
#
# Components: vehicle, localization, planning, system. Build through
# docker/build_runtime.sh and docker/build_dev.sh rather than by hand: they pass
# COMPONENT, copy px4_msgs in from the PX4 submodule, and tag the result.
#
# Every image runs ROS 2 Jazzy on Fast DDS, domain 20, kept on the host by
# docker/fastdds_localhost.xml (shared memory + UDP on 127.0.0.1); traffic between
# hosts goes through the zenoh bridge only. ROS_AUTOMATIC_DISCOVERY_RANGE stays at
# its default on purpose: the XML explains why LOCALHOST breaks the bridge.

ARG RUNTIME_IMAGE=ros:jazzy-ros-core-noble@sha256:8bbec5839a5258d43c8ff8d44c79c87a5c5a929ad419b22d0fa6cd5f196c7818
ARG SIM_BASE_IMAGE=nvcr.io/nvidia/isaac-sim:5.1.0
ARG BUILD_IMAGE=ros:jazzy-ros-base-noble@sha256:066420e07f60aa18262f2479981def87ebcfcec42eefb0c0c57c4a46098348ca

# ===================================================================== runtime-base
FROM ${RUNTIME_IMAGE} AS runtime-base
ENV ROS_DOMAIN_ID=20 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
    FASTRTPS_DEFAULT_PROFILES_FILE=/opt/castor/fastdds_localhost.xml \
    FASTDDS_DEFAULT_PROFILES_FILE=/opt/castor/fastdds_localhost.xml \
    CASTOR_ROBOT_CONFIG=/etc/castor/robot.yaml \
    LANG=C.UTF-8 \
    PYTHONUNBUFFERED=1
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
         python3-yaml ros-jazzy-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

# ===================================================================== base (build tools)
FROM ${BUILD_IMAGE} AS base
ENV ROS_DOMAIN_ID=20 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
    FASTRTPS_DEFAULT_PROFILES_FILE=/opt/castor/fastdds_localhost.xml \
    FASTDDS_DEFAULT_PROFILES_FILE=/opt/castor/fastdds_localhost.xml \
    CASTOR_ROBOT_CONFIG=/etc/castor/robot.yaml \
    LANG=C.UTF-8 \
    PYTHONUNBUFFERED=1
# Layer order everywhere below: slow and rarely changing first (apt, toolchains,
# third-party builds), then dependency lists, then CASTOR sources, then small
# config files. An edit rebuilds, and a Pi re-downloads, only the layers after it.
COPY docker/install_component_deps.sh /opt/castor/docker/
COPY .devcontainer/common/deps /opt/castor/deps/common
RUN /opt/castor/docker/install_component_deps.sh common

# ===================================================================== common-build
# castor_interfaces and castor_common, shared by every component.
FROM base AS common-build
COPY components/common /opt/castor/src/common
RUN apt-get update \
    && rosdep install --rosdistro jazzy --from-paths /opt/castor/src/common --ignore-src -y \
    && rm -rf /var/lib/apt/lists/* \
    && . /opt/ros/jazzy/setup.sh \
    && colcon build --merge-install \
         --base-paths /opt/castor/src/common \
         --build-base /tmp/build/common --install-base /opt/castor/common \
         --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf /tmp/build

# ===================================================================== manifests
# Only the package.xml files, so rosdep in component-deps re-runs when a
# dependency changes, not on every source edit.
FROM base AS manifests
COPY components /tmp/components
RUN mkdir -p /manifests \
    && cd /tmp/components \
    && find . -name package.xml -exec cp --parents {} /manifests/ \;

# ===================================================================== component-deps
FROM base AS component-deps
ARG COMPONENT
ARG TARGETARCH
RUN test -n "${COMPONENT}" || { echo "build with --build-arg COMPONENT=<vehicle|localization|planning|system>" >&2; exit 1; }
# Only the build-time inputs of the expensive step (XRCE agent, mavlink-router,
# QGC, ONNX Runtime); the runtime apt list is read later, by component-runtime.
COPY .devcontainer/${COMPONENT}/deps/apt-dependencies.txt .devcontainer/${COMPONENT}/deps/pip-requirements.txt \
     .devcontainer/${COMPONENT}/deps/install-source-deps.sh /opt/castor/deps/${COMPONENT}/
RUN TARGETARCH="${TARGETARCH}" /opt/castor/docker/install_component_deps.sh "${COMPONENT}"
COPY --from=manifests /manifests/${COMPONENT} /opt/castor/manifests/${COMPONENT}
RUN apt-get update \
    && rosdep install --rosdistro jazzy --from-paths /opt/castor/manifests/${COMPONENT} --ignore-src -y \
         --skip-keys "castor_interfaces castor_common" \
    && rm -rf /var/lib/apt/lists/*
COPY --from=common-build /opt/castor/common /opt/castor/common
RUN ln -sf /opt/castor/common/lib/castor_common/castor-config /usr/local/bin/castor-config

# ===================================================================== component-dev
FROM component-deps AS component-dev
ARG COMPONENT
ARG USERNAME=castor
ARG USER_UID=1000
ARG USER_GID=1000
# The base image ships `ubuntu` as uid 1000; turn it into the dev user with the
# host user's ids, so files written to the mounted repo belong to the host user.
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
         sudo gdb less vim-tiny bash-completion python3-colcon-mixin ccache \
    && rm -rf /var/lib/apt/lists/* \
    && groupmod -n "${USERNAME}" -g "${USER_GID}" ubuntu \
    && usermod -l "${USERNAME}" -u "${USER_UID}" -g "${USER_GID}" -d "/home/${USERNAME}" -m ubuntu \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/castor-dev \
    && chmod 0440 /etc/sudoers.d/castor-dev \
    && usermod -aG dialout "${USERNAME}"
COPY docker/fastdds_localhost.xml docker/castor_env.sh docker/dev_entrypoint.sh /opt/castor/
RUN echo 'CASTOR_ENV_LENIENT=1 source /opt/castor/castor_env.sh; ws=/home/ws/.component_workspaces/$CASTOR_COMPONENT/install; [ -f "$ws/setup.bash" ] && source "$ws/setup.bash"' \
      >> "/home/${USERNAME}/.bashrc"
ENV CASTOR_COMPONENT=${COMPONENT}
USER ${USERNAME}
WORKDIR /home/ws
ENTRYPOINT ["/opt/castor/dev_entrypoint.sh"]
CMD ["sleep", "infinity"]

FROM component-dev AS vehicle-dev
FROM component-dev AS localization-dev
FROM component-dev AS planning-dev
FROM component-dev AS system-dev

# ===================================================================== component-build
FROM component-deps AS component-build
ARG COMPONENT
COPY docker/resolve_runtime_deps.sh /opt/castor/docker/
COPY components/common /opt/castor/src/common
COPY components/${COMPONENT} /opt/castor/src/${COMPONENT}
RUN . /opt/ros/jazzy/setup.sh \
    && . /opt/castor/common/setup.sh \
    && colcon build --merge-install \
         --base-paths /opt/castor/src/${COMPONENT} \
         --build-base /tmp/build/${COMPONENT} --install-base /opt/castor/${COMPONENT} \
         --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf /tmp/build \
    && /opt/castor/docker/resolve_runtime_deps.sh /opt/castor/src/${COMPONENT} /opt/castor/src/common \
         > /opt/castor/${COMPONENT}/runtime-apt.txt \
    && install -m 755 /opt/castor/src/${COMPONENT}/entrypoint.sh /opt/castor/${COMPONENT}/entrypoint.sh \
    && if [ -d /opt/castor/src/${COMPONENT}/scripts ]; then \
         install -D -m 755 -t /opt/castor/${COMPONENT}/bin /opt/castor/src/${COMPONENT}/scripts/*; \
       fi

# ===================================================================== component-test
FROM component-deps AS component-test
ARG COMPONENT
COPY docker/fastdds_localhost.xml /opt/castor/fastdds_localhost.xml
COPY components/common /opt/castor/src/common
COPY components/${COMPONENT} /opt/castor/src/${COMPONENT}
RUN . /opt/ros/jazzy/setup.sh \
    && cd /tmp \
    && colcon build --base-paths /opt/castor/src/common /opt/castor/src/${COMPONENT} \
         --build-base /tmp/tb --install-base /tmp/ti \
         --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
    && colcon test --base-paths /opt/castor/src/common /opt/castor/src/${COMPONENT} \
         --build-base /tmp/tb --install-base /tmp/ti --event-handlers console_direct+ \
    && colcon test-result --test-result-base /tmp/tb --verbose

# ===================================================================== component-runtime
FROM runtime-base AS component-runtime
ARG COMPONENT
ARG CASTOR_REVISION=unknown
COPY .devcontainer/${COMPONENT}/deps/apt-runtime-dependencies.txt /tmp/apt-runtime.txt
COPY --from=component-build /opt/castor/${COMPONENT}/runtime-apt.txt /tmp/ros-runtime.txt
RUN pkgs=$(cat /tmp/apt-runtime.txt /tmp/ros-runtime.txt | grep -vE '^[[:space:]]*(#|$)' | sort -u | tr '\n' ' ') \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends ${pkgs} \
    && rm -rf /var/lib/apt/lists/* /tmp/*.txt
COPY --from=component-deps /opt/castor/third_party /opt/castor/third_party
COPY --from=component-build /opt/castor/common /opt/castor/common
COPY --from=component-build /opt/castor/${COMPONENT} /opt/castor/${COMPONENT}
COPY docker/fastdds_localhost.xml docker/castor_env.sh /opt/castor/
RUN ln -sf /opt/castor/common/lib/castor_common/castor-config /usr/local/bin/castor-config \
    && ln -sf /opt/castor/${COMPONENT}/entrypoint.sh /opt/castor/entrypoint.sh \
    && if [ -d /opt/castor/${COMPONENT}/bin ]; then ln -sf /opt/castor/${COMPONENT}/bin/* /usr/local/bin/; fi \
    && echo 'CASTOR_ENV_LENIENT=1 source /opt/castor/castor_env.sh' >> /etc/bash.bashrc
ENV CASTOR_COMPONENT=${COMPONENT} \
    CASTOR_IMAGE_REVISION=${CASTOR_REVISION}
LABEL org.opencontainers.image.source="https://github.com/FYP-UAV-ENTC-22/castor-main" \
      org.opencontainers.image.revision="${CASTOR_REVISION}" \
      org.opencontainers.image.title="castor-${COMPONENT}" \
      org.opencontainers.image.licenses="BSD-3-Clause"
ENTRYPOINT ["/opt/castor/entrypoint.sh"]
# ros2 launch shuts its nodes down cleanly only on SIGINT. On SIGTERM (Docker's
# default) it exits at once and the nodes are killed without leaving DDS, which
# stalls the zenoh bridge's routes and leaves MCAP recordings unfinished.
STOPSIGNAL SIGINT

# ===================================================================== finals
FROM component-runtime AS vehicle-final
RUN [ "${CASTOR_COMPONENT}" = vehicle ] || { echo "vehicle-final needs --build-arg COMPONENT=vehicle" >&2; exit 1; }
CMD ["ros2", "launch", "vehicle_launch", "vehicle.launch.py"]

FROM component-runtime AS localization-final
RUN [ "${CASTOR_COMPONENT}" = localization ] || { echo "localization-final needs --build-arg COMPONENT=localization" >&2; exit 1; }
CMD ["ros2", "launch", "localization_launch", "localization.launch.py"]

FROM component-runtime AS planning-final
RUN [ "${CASTOR_COMPONENT}" = planning ] || { echo "planning-final needs --build-arg COMPONENT=planning" >&2; exit 1; }
# The policy model packages (models/README.md), last: a new model rebuilds only this layer, and a code change leaves
# it byte-identical. /var/lib/castor/models, mounted at run time, overrides them.
COPY models/ /opt/castor/models/
CMD ["ros2", "launch", "planning_launch", "planning.launch.py"]

FROM component-runtime AS system-final
RUN [ "${CASTOR_COMPONENT}" = system ] || { echo "system-final needs --build-arg COMPONENT=system" >&2; exit 1; }
CMD ["ros2", "launch", "system_launch", "system.launch.py"]

# ===================================================================== simulation
# One image for everything that needs the GPU: Isaac Sim 5.1, Isaac Lab, Pegasus,
# PX4 SITL (toolchain; built inside the container from the mounted checkout),
# ROS 2 Jazzy, and the MARL training stack (skrl, MARL ext). Never pushed: NVIDIA's
# licence forbids redistributing the base, so every machine builds it locally
# with docker/build_simulation.sh. The repo is mounted at /home/ws at run time.
#
# Layers go from slowest-changing to fastest: OS packages, torch, Python
# dependencies (resolved from package metadata only), small config files, and
# last the editable installs, which only point at the mounted repo. Editing
# code needs no rebuild at all.
#
# ROS 2: Isaac processes (python.sh, isaac-sim.sh) load the ROS 2 Jazzy build
# bundled with the Isaac ROS 2 bridge (Python 3.11 rclpy, Fast DDS 2.14), via
# docker/simulation/isaac_ros_env.sh hooked into Isaac's setup_python_env.sh.
# System Jazzy (Python 3.12) is installed for the CLI and
# colcon; the `ros2` wrapper sources it for itself, so it never leaks into Isaac.
FROM ${SIM_BASE_IMAGE} AS simulation
USER root
ENV ISAACSIM_PATH=/isaac-sim \
    OMNI_KIT_ALLOW_ROOT=1 \
    ROS_DOMAIN_ID=20 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    FASTRTPS_DEFAULT_PROFILES_FILE=/opt/castor/fastdds_localhost.xml \
    FASTDDS_DEFAULT_PROFILES_FILE=/opt/castor/fastdds_localhost.xml \
    PIP_ROOT_USER_ACTION=ignore \
    PYTHONUNBUFFERED=1 \
    LANG=C.UTF-8

# OS packages: ROS 2 Jazzy, Isaac Lab's build deps, PX4 SITL toolchain, X11 auth.
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends curl ca-certificates \
    && curl -fsSL -o /tmp/ros2-apt-source.deb \
         https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.2.0/ros2-apt-source_1.2.0.noble_all.deb \
    && apt-get install -y /tmp/ros2-apt-source.deb && rm /tmp/ros2-apt-source.deb \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
         ros-jazzy-ros-base ros-jazzy-rmw-fastrtps-cpp \
         build-essential cmake ninja-build ccache git gdb make file rsync unzip zip \
         libglib2.0-0 ncurses-term wget xauth less vim-tiny \
         python3-dev python3-pip python3-venv libxml2-utils \
    && rm -rf /var/lib/apt/lists/*

# torch for Isaac Sim's Python (it ships none); same pins as the castor conda env.
RUN /isaac-sim/python.sh -m pip install --no-cache-dir \
        torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128 \
    && /isaac-sim/python.sh -m pip freeze | grep -E '^torch(vision)?==' > /opt/torch-constraints.txt

# PX4's build-time Python packages, in their own venv (PX4 pins versions that
# would replace Debian's numpy and friends under ROS 2's system Python).
# `make sim-px4` activates it.
COPY --from=simmeta px4-requirements.txt /tmp/px4-requirements.txt
RUN python3 -m venv /opt/px4-venv \
    && /opt/px4-venv/bin/pip install --no-cache-dir -r /tmp/px4-requirements.txt \
    && rm /tmp/px4-requirements.txt

# Python dependencies of Isaac Lab, Pegasus, skrl and the MARL ext, resolved from
# their metadata only (setup.py, extension.toml, stub packages): this layer only
# rebuilds when a dependency list changes, never for a code edit.
COPY --from=simmeta pkgs /opt/castor/sim-meta
RUN cd /opt/castor/sim-meta \
    && export PIP_CONSTRAINT=/opt/torch-constraints.txt \
    && /isaac-sim/python.sh -m pip install --no-cache-dir toml "setuptools<81" \
    && /isaac-sim/python.sh -m pip install --no-cache-dir --no-build-isolation flatdict==4.0.1 \
    && for p in isaaclab isaaclab_assets isaaclab_tasks isaaclab_rl isaaclab_mimic; do \
         /isaac-sim/python.sh -m pip install --no-cache-dir "./IsaacLab/source/$p" || exit 1; done \
    && /isaac-sim/python.sh -m pip install --no-cache-dir "./IsaacLab/source/isaaclab_rl[none]" \
         ./skrl ./MARL_mav_carry_ext numpy scipy pyyaml pymavlink \
    && /isaac-sim/python.sh -m pip uninstall -y isaaclab isaaclab_assets isaaclab_tasks isaaclab_rl \
         isaaclab_mimic skrl MARL_mav_carry_ext \
    && /isaac-sim/python.sh -m pip uninstall -y quadprog || true

# Small config files and helpers.
COPY docker/fastdds_localhost.xml /opt/castor/fastdds_localhost.xml
COPY docker/simulation/ /opt/castor/simulation/
RUN install -m 755 /opt/castor/simulation/ros2 /usr/local/bin/ros2 \
    && echo 'source /opt/castor/simulation/isaac_ros_env.sh' >> /isaac-sim/setup_python_env.sh \
    && install -m 755 /opt/castor/simulation/entrypoint.sh /opt/castor/entrypoint.sh

# Last: editable installs that point into the mounted repo (/home/ws). They are
# path entries, so the code is whatever the mount holds; no rebuild on edits.
RUN site=$(/isaac-sim/python.sh -c 'import site; print(site.getsitepackages()[0])') \
    && { for p in isaaclab isaaclab_assets isaaclab_tasks isaaclab_rl isaaclab_mimic; do \
           echo "/home/ws/components/simulation/IsaacLab/source/$p"; done; \
         echo /home/ws/components/simulation/pegasus_simulator/extensions/pegasus.simulator; \
         echo /home/ws/components/planning/skrl; \
         echo /home/ws/components/planning/MARL_cooperative_aerial_manipulation_ext/exts/MARL_mav_carry_ext; \
       } > "$site/castor-workspace.pth" \
    && mkdir -p /home/ws
WORKDIR /home/ws
ENTRYPOINT ["/opt/castor/entrypoint.sh"]
CMD ["bash"]
