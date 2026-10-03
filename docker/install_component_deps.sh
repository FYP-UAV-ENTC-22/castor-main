#!/bin/bash
# Install one component's non-ROS dependencies from .devcontainer/<component>/deps/:
#
#   apt-dependencies.txt     apt packages for building and developing
#   pip-requirements.txt     pip packages that apt/rosdep can't provide
#   install-source-deps.sh   source/binary installs into /opt/castor/third_party
#
# ROS dependencies are not listed there: rosdep reads them from package.xml.
# Runs inside the image build (docker/Dockerfile), as root.
set -euo pipefail

component="${1:?usage: install_component_deps.sh <component>}"
deps="${2:-/opt/castor/deps/$component}"

list() { [ -f "$1" ] && grep -vE '^[[:space:]]*(#|$)' "$1" | tr '\n' ' ' || true; }

mkdir -p /opt/castor/third_party/env.d

apt_pkgs=$(list "$deps/apt-dependencies.txt")
if [ -n "$apt_pkgs" ]; then
    echo "==> [$component] apt: $apt_pkgs"
    apt-get update
    # shellcheck disable=SC2086
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends $apt_pkgs
    rm -rf /var/lib/apt/lists/*
fi

if [ -n "$(list "$deps/pip-requirements.txt")" ]; then
    echo "==> [$component] pip"
    # Ubuntu 24.04 marks the system Python as externally managed (PEP 668).
    python3 -m pip install --no-cache-dir --break-system-packages -r "$deps/pip-requirements.txt"
fi

if [ -f "$deps/install-source-deps.sh" ]; then
    echo "==> [$component] source deps"
    bash "$deps/install-source-deps.sh"
fi
