#!/bin/bash
# Entrypoint for the <component>-dev images. The repo is mounted at /home/ws;
# build inside with `make <component>-build`, which installs into
# /home/ws/.component_workspaces/<component>/install.
set -e

CASTOR_ENV_LENIENT=1 source /opt/castor/castor_env.sh
ws="/home/ws/.component_workspaces/${CASTOR_COMPONENT}/install"
# shellcheck disable=SC1091
[ -f "$ws/setup.bash" ] && source "$ws/setup.bash"

exec "$@"
