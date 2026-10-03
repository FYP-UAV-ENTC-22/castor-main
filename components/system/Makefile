# Colcon targets for the system component. From the repo root: make system-<target>.
# Run inside the system dev container (make system-dev-shell).

MAKEFILE_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
COMPONENT_NAME := system

include $(MAKEFILE_DIR)../../make/component_colcon.mk
