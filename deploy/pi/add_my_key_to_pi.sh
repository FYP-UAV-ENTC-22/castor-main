#!/bin/bash
# Put your SSH key on a Pi so later ssh/update commands need no login password.
# sudo on the Pi still asks for the account password; key auth only covers login.
#
# Safe to re-run, and each teammate runs it for their own key: ssh-copy-id only
# appends to authorized_keys.
#
# Usage: deploy/pi/add_my_key_to_pi.sh <pi-host> <pi-user>
#
# Copied from the retired drone-ops repo. The password is no longer an
# argument (it ended up in shell history); ssh-copy-id asks for it once.
set -euo pipefail

PI_HOST="${1:?usage: $0 <pi-host> <pi-user>}"
PI_USER="${2:?usage: $0 <pi-host> <pi-user>}"

KEY_PATH="$HOME/.ssh/id_ed25519"
if [ ! -f "$KEY_PATH" ]; then
    echo "No SSH key at $KEY_PATH, generating one..."
    ssh-keygen -t ed25519 -f "$KEY_PATH" -N "" -C "$(whoami)@$(hostname)"
else
    echo "Using existing key: $KEY_PATH"
fi

echo "Copying ${KEY_PATH}.pub to $PI_USER@$PI_HOST (asks for the Pi password once)..."
ssh-copy-id -o StrictHostKeyChecking=accept-new -i "${KEY_PATH}.pub" "$PI_USER@$PI_HOST"

echo
echo "Verifying passwordless login..."
if ssh -o BatchMode=yes -o ConnectTimeout=5 "$PI_USER@$PI_HOST" 'echo OK' | grep -q OK; then
    echo "Done: ssh $PI_USER@$PI_HOST now works without a password."
else
    echo "Key-based login isn't working yet." >&2
    exit 1
fi
