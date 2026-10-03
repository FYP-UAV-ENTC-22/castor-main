# Colcon targets for the vehicle component. From the repo root: make vehicle-<target>.
# Run inside the vehicle dev container (make vehicle-dev-shell).

MAKEFILE_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
COMPONENT_NAME := vehicle
# px4_msgs/msg and srv come from the PX4 submodule at its pinned commit.
PREPARE_CMD := $(MAKEFILE_DIR)../../docker/sync_px4_msgs.sh

include $(MAKEFILE_DIR)../../make/component_colcon.mk
