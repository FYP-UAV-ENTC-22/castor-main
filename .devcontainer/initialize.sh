#!/bin/sh
# Runs on the host before VS Code starts a CASTOR dev container.
# Lets the container open windows (QGC, rqt) on the host display.
command -v xhost >/dev/null 2>&1 && xhost +local: >/dev/null 2>&1 || true
exit 0
