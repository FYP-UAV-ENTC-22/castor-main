#!/bin/bash
# Print the directories of a component's own ROS packages, one per line.
#
#   list_component_packages.sh <component>
#
# Submodules are skipped: PX4-Autopilot alone contains 25 package.xml files that
# must never end up in a CASTOR colcon build.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
component="${1:?usage: list_component_packages.sh <component>}"
dir="$ROOT/components/$component"
[ -d "$dir" ] || { echo "list_component_packages: no component '$component'" >&2; exit 1; }

prune=()
while read -r _ path; do
    case "$path" in
        components/"$component"/*) prune+=(-path "$ROOT/$path" -prune -o) ;;
    esac
done < <(git -C "$ROOT" config -f .gitmodules --get-regexp '^submodule\..*\.path$' 2>/dev/null || true)

find "$dir" "${prune[@]}" -name .git -prune -o -name package.xml -printf '%h\n' | sort
