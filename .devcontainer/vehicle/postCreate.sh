#!/bin/sh
# First start of the vehicle dev container: build the component once.
set -e
cd /home/ws
make vehicle-build
