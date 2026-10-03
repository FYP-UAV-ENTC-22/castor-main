#!/bin/sh
# First start of the system dev container: build the component once.
set -e
cd /home/ws
make system-build
