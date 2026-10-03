#!/bin/bash
# Entrypoint for the localization container.
set -e

# ROS, the install spaces, and robot.yaml (validated; CASTOR_* exported).
source /opt/castor/castor_env.sh

# After a crash restart, let the old DDS participants expire first (castor_env.sh).
castor_wait_out_old_participants "${1:-}"

exec "$@"
