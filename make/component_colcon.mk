# Shared colcon targets for one component, included by components/<c>/Makefile.
# Run them inside that component's dev container (make <c>-dev-shell), where the
# repo is mounted at /home/ws and the component's dependencies are installed.
#
# Each component builds in its own workspace, .component_workspaces/<c>/, with
# components/common built alongside it, and never looks inside submodules.

SHELL := /bin/bash
WS_ROOT := $(abspath $(MAKEFILE_DIR)/../..)
BUILD_ROOT := $(WS_ROOT)/.component_workspaces/$(COMPONENT_NAME)
PKG_DIRS = $(shell $(WS_ROOT)/docker/list_component_packages.sh common) \
           $(shell $(WS_ROOT)/docker/list_component_packages.sh $(COMPONENT_NAME))
COLCON_PATHS = --base-paths $(PKG_DIRS) \
               --build-base $(BUILD_ROOT)/build --install-base $(BUILD_ROOT)/install
# Non-interactive shells (VS Code postCreate, `docker compose exec -T`) never
# read .bashrc, so every colcon call sets up ROS itself.
ROS_ENV := source /opt/ros/jazzy/setup.bash && \
           { [ ! -f /opt/castor/common/setup.bash ] || source /opt/castor/common/setup.bash; } && \
           { compgen -G '/opt/castor/third_party/env.d/*.sh' >/dev/null && for f in /opt/castor/third_party/env.d/*.sh; do source "$$f"; done; true; }
BUILD_TYPE ?= RelWithDebInfo
PKGS ?=

.PHONY: build build-pkg test clean list-pkgs source help check-ros prepare

check-ros:
	@command -v colcon >/dev/null || { echo "colcon not found: run this inside the dev container (make $(COMPONENT_NAME)-dev-shell)"; exit 1; }

# Components set PREPARE_CMD for generated inputs (vehicle: px4_msgs).
prepare: check-ros
	@$(if $(PREPARE_CMD),$(PREPARE_CMD),true)

build: prepare
	$(ROS_ENV) && cd $(WS_ROOT) && colcon build $(COLCON_PATHS) --symlink-install \
	    --cmake-args -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
	@echo "now: source $(BUILD_ROOT)/install/setup.bash"

build-pkg: prepare
	@[ -n "$(PKGS)" ] || { echo "usage: make $(COMPONENT_NAME)-build-pkg PKGS='pkg1 pkg2'"; exit 1; }
	$(ROS_ENV) && cd $(WS_ROOT) && colcon build $(COLCON_PATHS) --symlink-install \
	    --packages-up-to $(PKGS) --cmake-args -DCMAKE_BUILD_TYPE=$(BUILD_TYPE)

test: build
	$(ROS_ENV) && cd $(WS_ROOT) && colcon test $(COLCON_PATHS) --event-handlers console_direct+ \
	    && colcon test-result --test-result-base $(BUILD_ROOT)/build --verbose

clean:
	rm -rf $(BUILD_ROOT)

list-pkgs: check-ros
	@$(ROS_ENV) && cd $(WS_ROOT) && colcon list --base-paths $(PKG_DIRS)

source:
	@echo "source $(BUILD_ROOT)/install/setup.bash"

help:
	@echo "$(COMPONENT_NAME): build | build-pkg PKGS=... | test | clean | list-pkgs | source"
