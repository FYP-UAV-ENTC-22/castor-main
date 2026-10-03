#!/bin/sh
# First start of the localization dev container: build the component once.
set -e
cd /home/ws
make localization-build
