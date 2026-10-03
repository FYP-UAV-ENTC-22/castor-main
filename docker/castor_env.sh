# shellcheck shell=bash
# Sourced by every component entrypoint, and by interactive shells in the images.
#
# Sets up ROS 2, the CASTOR install spaces and third-party tools, then validates
# robot.yaml and exports CASTOR_ROBOT_ID, CASTOR_NS, CASTOR_HARDWARE,
# CASTOR_TEAM_SIZE and CASTOR_TEAM_INDEX.
#
# An invalid or missing robot.yaml stops the container with the reason, unless
# CASTOR_ENV_LENIENT=1 (interactive shells), which only warns.

# shellcheck disable=SC1090,SC1091
source /opt/ros/jazzy/setup.bash
for _castor_ws in /opt/castor/common "/opt/castor/${CASTOR_COMPONENT:-none}"; do
    [ -f "$_castor_ws/setup.bash" ] && source "$_castor_ws/setup.bash"
done
for _castor_f in /opt/castor/third_party/env.d/*.sh; do
    [ -f "$_castor_f" ] && source "$_castor_f"
done
unset _castor_ws _castor_f

if _castor_env=$(castor-config env); then
    eval "$_castor_env"
else
    if [ "${CASTOR_ENV_LENIENT:-0}" = 1 ]; then
        echo "castor: robot.yaml problem (above); CASTOR_* variables are not set" >&2
    else
        echo "castor: refusing to start ${CASTOR_COMPONENT:-this component} until robot.yaml is fixed" >&2
        exit 2
    fi
fi
unset _castor_env

# Called by each entrypoint just before it starts ROS. A container Docker restarts
# (a crash under `restart: unless-stopped`, or `docker restart`) keeps its own
# filesystem, so the marker tells a restart from a first start. Nodes that crashed
# never said goodbye on DDS; the zenoh bridge tracks local publishers by node name,
# so when the dead participants' lease runs out it retires the routes the new nodes
# (same names) are using, and their topics stop crossing to other hosts. Waiting
# longer than the lease (5 s, docker/fastdds_localhost.xml) first avoids that. A
# clean stop (SIGINT, the images' stop signal) doesn't need this, but costs only
# the wait. Commands other than `ros2 ...` (castor-config, a shell) start no nodes
# and skip it.
castor_wait_out_old_participants() {
    [ "${1:-}" = ros2 ] || return 0
    local marker=/var/tmp/castor-container-started
    if [ -e "$marker" ]; then
        echo "castor: container restarted; waiting ${CASTOR_RESTART_WAIT_S:-6} s for its previous DDS participants to expire" >&2
        sleep "${CASTOR_RESTART_WAIT_S:-6}"
    fi
    touch "$marker"
}
