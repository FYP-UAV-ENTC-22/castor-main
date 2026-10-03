#!/bin/bash
# One-time setup of a Raspberry Pi (4 or 5, 64-bit OS) to run the CASTOR stack.
# Run it on the Pi, with sudo:
#
#   curl -fsSL https://raw.githubusercontent.com/FYP-UAV-ENTC-22/castor-main/main/deploy/pi/install.sh -o install.sh
#   sudo bash install.sh
#
# What it does (each step is skipped if already done):
#   1. installs Docker Engine + the compose and buildx plugins from Docker's apt repo
#   2. adds you to the docker group, so castor-update runs without sudo
#   3. clones castor-main, without submodules, to /opt/castor/src
#   4. creates /etc/castor/robot.yaml and stack.env from the examples (never overwrites)
#   5. creates /run/castor (tmpfiles), /var/log/castor and /var/lib/castor/models
#   6. installs castor-stack.service (enabled: starts local images at boot)
#      and castor-update.service (manual only; there is no timer)
#
# It does not pull images, start the stack, or touch the flight controller.
set -euo pipefail

BRANCH="${CASTOR_BRANCH:-main}"
REPO="${CASTOR_REPO:-https://github.com/FYP-UAV-ENTC-22/castor-main.git}"
SRC=/opt/castor/src
USER_NAME="${SUDO_USER:-}"

die()  { printf '\033[31minstall: %s\033[0m\n' "$*" >&2; exit 1; }
say()  { printf '\n\033[1m==> %s\033[0m\n' "$*"; }

[ "$(id -u)" -eq 0 ] || die "run with sudo"
[ -n "$USER_NAME" ] && [ "$USER_NAME" != root ] || die "run with sudo from your normal account (SUDO_USER is not set)"
[ "$(uname -m)" = aarch64 ] || echo "warning: $(uname -m) is not aarch64; CASTOR images for Pis are linux/arm64" >&2
[ "$(getconf LONG_BIT)" = 64 ] || die "this OS is 32-bit; CASTOR needs a 64-bit OS (Ubuntu 24.04 arm64 or Raspberry Pi OS 64-bit)"

# shellcheck disable=SC1091
. /etc/os-release

say "Docker Engine"
if command -v docker >/dev/null && docker compose version >/dev/null 2>&1; then
    echo "    already installed: $(docker --version)"
else
    case "$ID" in
        ubuntu|debian) ;;
        *) die "unsupported OS '$ID'; install Docker Engine by hand (https://docs.docker.com/engine/install/)" ;;
    esac
    apt-get update
    apt-get install -y ca-certificates curl git
    install -m 0755 -d /etc/apt/keyrings
    curl -fsSL "https://download.docker.com/linux/$ID/gpg" -o /etc/apt/keyrings/docker.asc
    chmod a+r /etc/apt/keyrings/docker.asc
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/$ID $VERSION_CODENAME stable" \
        > /etc/apt/sources.list.d/docker.list
    apt-get update
    apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
    systemctl enable --now docker
fi
command -v git >/dev/null || apt-get install -y git

say "docker group"
if id -nG "$USER_NAME" | tr ' ' '\n' | grep -qx docker; then
    echo "    $USER_NAME is already in it"
else
    usermod -aG docker "$USER_NAME"
    echo "    added $USER_NAME (log out and back in for it to apply)"
fi

say "castor-main checkout ($SRC)"
if [ -d "$SRC/.git" ]; then
    echo "    exists: $(git -c safe.directory="$SRC" -C "$SRC" log -1 --format='%h %s')"
else
    mkdir -p "$(dirname "$SRC")"
    git clone --quiet --depth 1 --branch "$BRANCH" "$REPO" "$SRC"
    echo "    cloned $BRANCH (no submodules: the Pi only needs docker/ and deploy/)"
fi
chown -R "$USER_NAME:$USER_NAME" "$SRC"

say "/etc/castor"
mkdir -p /etc/castor
if [ -f /etc/castor/robot.yaml ]; then
    echo "    robot.yaml exists, left as is"
else
    install -m 0644 "$SRC/deploy/robot.example.yaml" /etc/castor/robot.yaml
    echo "    created robot.yaml from the example: EDIT robot.id, robot.namespace, robot.hardware and team.index"
fi
if [ -f /etc/castor/stack.env ]; then
    echo "    stack.env exists, left as is"
else
    install -m 0644 "$SRC/deploy/pi/stack.env.example" /etc/castor/stack.env
    echo "    created stack.env (CASTOR_TAG=main)"
fi

say "Runtime directories"
echo "d /run/castor 0775 root docker -" > /etc/tmpfiles.d/castor.conf
systemd-tmpfiles --create /etc/tmpfiles.d/castor.conf
install -d -m 2775 -g docker /var/log/castor
install -d -m 0755 /var/lib/castor/models
echo "    /run/castor, /var/log/castor, /var/lib/castor/models (put policy.onnx here)"

say "systemd units"
install -m 0644 "$SRC/deploy/pi/castor-stack.service" /etc/systemd/system/castor-stack.service
install -m 0644 "$SRC/deploy/pi/castor-update.service" /etc/systemd/system/castor-update.service
ln -sf "$SRC/deploy/pi/castor-update.sh" /usr/local/bin/castor-update
systemctl daemon-reload
systemctl enable castor-stack.service >/dev/null
echo "    castor-stack enabled (starts local images at boot, never pulls)"
echo "    castor-update installed, not enabled (run it by hand)"

say "Done. Next:"
cat <<EOF
    1. sudo nano /etc/castor/robot.yaml        set this drone's id, namespace, hardware, team.index
    2. castor-update                           first pull (needs internet; see deploy/network-gateway/)
    3. sudo systemctl start castor-stack       start it; it also starts at every boot
    4. docker compose -f $SRC/docker/docker-compose.prod.yml ps
EOF
