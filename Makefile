# CASTOR workspace commands. Every target is <component>-<target> or a stack-wide one.
#
#   make help

SHELL := /bin/bash
COMPONENTS := vehicle localization planning system
COLCON_TARGETS := build build-pkg test clean list-pkgs source
DEV_COMPOSE := docker compose -f docker/docker-compose.dev.yml
PROD_COMPOSE := docker compose -f docker/docker-compose.prod.yml
# `make stack-up` runs your local builds against the laptop robot.yaml.
STACK_ENV := CASTOR_TAG=$${CASTOR_TAG:-local} CASTOR_ROBOT_CONFIG=$${CASTOR_ROBOT_CONFIG:-$(CURDIR)/deploy/robot.laptop.yaml}

.DEFAULT_GOAL := help

.PHONY: help
help:
	@echo "CASTOR workspace"
	@echo ""
	@echo "Inside a component's dev container (repo at /home/ws):"
	@echo "  make <component>-build | -test | -clean | -list-pkgs | -source | -build-pkg PKGS='a b'"
	@echo ""
	@echo "On the host:"
	@echo "  make <component>-dev-image    build castor-<component>:dev"
	@echo "  make <component>-dev-up       start its dev container"
	@echo "  make <component>-dev-shell    shell in it"
	@echo "  make <component>-dev-logs | -dev-down"
	@echo "  make <component>-image        build the runtime image (castor-<component>:local)"
	@echo "  make images                   all four runtime images"
	@echo "  make images-test              colcon tests for all four, then the images"
	@echo "  make stack-up | stack-down | stack-logs | stack-ps"
	@echo "                                the production compose stack on this machine, with"
	@echo "                                local images and deploy/robot.laptop.yaml"
	@echo "  make vehicle-px4-msgs         copy PX4's msg/srv into components/vehicle/px4_msgs"
	@echo "  make vehicle-qgc              open QGroundControl from the running vehicle container"
	@echo "  make sim-image                build castor-simulation:local (Isaac Sim, Isaac Lab, Pegasus, PX4, ROS 2, training)"
	@echo "  make sim-up | sim-shell | sim-down | sim-gui | sim-px4 | sim-train-smoke | sim-pegasus-ros2 | sim-own"
	@echo ""
	@echo "Components: $(COMPONENTS)"

# ------------------------------------------------------------- colcon, in a dev container
define colcon_rules
.PHONY: $(1)-$(2)
$(1)-$(2):
	@$$(MAKE) --no-print-directory -C components/$(1) $(2)
endef
$(foreach c,$(COMPONENTS),$(foreach t,$(COLCON_TARGETS),$(eval $(call colcon_rules,$(c),$(t)))))

# ------------------------------------------------------------- dev containers, on the host
define dev_rules
.PHONY: $(1)-dev-image $(1)-dev-up $(1)-dev-shell $(1)-dev-logs $(1)-dev-down $(1)-image
$(1)-dev-image:
	docker/build_dev.sh $(1)
$(1)-dev-up:
	$(DEV_COMPOSE) up -d $(1)
$(1)-dev-shell:
	$(DEV_COMPOSE) exec $(1) bash
$(1)-dev-logs:
	$(DEV_COMPOSE) logs -f $(1)
$(1)-dev-down:
	$(DEV_COMPOSE) rm -sf $(1)
$(1)-image:
	docker/build_runtime.sh $(1)
endef
$(foreach c,$(COMPONENTS),$(eval $(call dev_rules,$(c))))

.PHONY: images images-test
images:
	docker/build_runtime.sh
images-test:
	docker/build_runtime.sh --test

# ------------------------------------------------------------- the production stack, locally
.PHONY: stack-up stack-down stack-logs stack-ps
stack-up:
	$(STACK_ENV) $(PROD_COMPOSE) up -d
stack-down:
	$(STACK_ENV) $(PROD_COMPOSE) down
stack-logs:
	$(STACK_ENV) $(PROD_COMPOSE) logs -f
stack-ps:
	$(STACK_ENV) $(PROD_COMPOSE) ps

# ------------------------------------------------------------- vehicle extras
.PHONY: vehicle-px4-msgs vehicle-qgc
vehicle-px4-msgs:
	docker/sync_px4_msgs.sh
vehicle-qgc:
	@command -v xhost >/dev/null && xhost +local: >/dev/null || echo "xhost not found; QGC may not be allowed on the display"
	$(STACK_ENV) $(PROD_COMPOSE) exec vehicle castor-qgc

# ------------------------------------------------------------- simulation (GPU machine only)
# NVIDIA's asset pack is mounted only when it is readable (an unplugged drive must not stop the container).
SIM_ASSET_PACK ?= /mnt/isaac/isaacsim_assets
CASTOR_ASSET_PACK ?= $(shell test -r $(SIM_ASSET_PACK)/Assets 2>/dev/null && echo $(SIM_ASSET_PACK))
export CASTOR_ASSET_PACK
SIM_COMPOSE := docker compose -f docker/docker-compose.sim.yml
SIM_EXEC := $(SIM_COMPOSE) exec simulation
MARL_DIR := components/planning/MARL_cooperative_aerial_manipulation_ext
.PHONY: sim-image sim-up sim-shell sim-down sim-gui sim-px4 sim-train-smoke sim-pegasus-ros2 sim-own dds-shm-clean
sim-image:
	docker/build_simulation.sh
sim-up:
	$(SIM_COMPOSE) up -d
sim-shell: sim-up
	$(SIM_EXEC) bash; $(MAKE) --no-print-directory sim-own
sim-down:
	-$(MAKE) --no-print-directory sim-own
	$(SIM_COMPOSE) down
sim-gui: sim-up
	@command -v xhost >/dev/null && xhost +local: >/dev/null || echo "xhost not found; the GUI may not be allowed on the display"
	$(SIM_EXEC) /isaac-sim/isaac-sim.sh; $(MAKE) --no-print-directory sim-own
# PX4 SITL, compiled in the simulation container into the PX4 checkout's build/.
sim-px4: sim-up
	$(SIM_EXEC) bash -c 'source /opt/px4-venv/bin/activate && cd components/vehicle/PX4-Autopilot && git config --global --add safe.directory "*" && make px4_sitl_default && make px4_sitl_raptor'; \
	  s=$$?; $(MAKE) --no-print-directory sim-own; exit $$s
# Three MAPPO iterations on the flycrane hover task (about a minute once shaders are cached).
sim-train-smoke: sim-up
	$(SIM_COMPOSE) exec -w /home/ws/$(MARL_DIR) simulation /isaac-sim/python.sh scripts/skrl/train.py \
	  --task=Isaac-flycrane-payload-decentralized-hovering-v0 --headless --num_envs=8 --max_iterations=3 --seed=42 --algorithm=MAPPO; \
	  s=$$?; $(MAKE) --no-print-directory sim-own; exit $$s
# The rig (components/simulation/assets/config, default payload_rig_marl.yaml: the default model's) on the ground,
# one disarmed PX4 SITL per drone, until the window closes. PX4 i waits for robot i+1's stack
# (components/simulation/sil/sil.sh up). Ground truth on sim/*.
# Options: RIG=<file>, DRONES=N (default: the rig's), HEADLESS=1, DURATION=s, PX4_BUILD=px4_sitl_default (default:
# the RAPTOR build). Needs make sim-px4 once.
sim-pegasus-ros2: sim-up
	@[ -n "$(HEADLESS)" ] || { command -v xhost >/dev/null && xhost +local: >/dev/null || echo "xhost not found; the GUI may not be allowed on the display"; }
	$(SIM_EXEC) /isaac-sim/python.sh components/simulation/sil/sil_pegasus.py --duration $(or $(DURATION),0) \
	  $(if $(RIG),--rig $(RIG)) $(if $(DRONES),--drones $(DRONES)) $(if $(HEADLESS),--headless) \
	  $(if $(PX4_BUILD),--build $(PX4_BUILD)); \
	  s=$$?; $(MAKE) --no-print-directory sim-own; exit $$s
# Fast DDS shared-memory segments left in /dev/shm by killed containers (root-owned); only unused ones go.
dds-shm-clean:
	docker run --rm --ipc host --entrypoint bash ghcr.io/fyp-uav-entc-22/castor-system:$(or $(CASTOR_TAG),local) -c 'source /opt/ros/jazzy/setup.bash && fastdds shm clean'
# The container runs as root: give whatever it wrote into the checkout back to you.
sim-own:
	$(SIM_COMPOSE) exec -T simulation find /home/ws -xdev -user 0 -exec chown -h $(shell id -u):$(shell id -g) {} +

# ------------------------------------------------------------- end-to-end tests
.PHONY: test-zenoh test-zenoh-remote
test-zenoh:
	deploy/tests/zenoh_bridge_test.sh --local
test-zenoh-remote:
	@[ -n "$(HOST)" ] || { echo "usage: make test-zenoh-remote HOST=user@pi [MULTICAST=1]"; exit 2; }
	deploy/tests/zenoh_bridge_test.sh --remote $(HOST) $(if $(MULTICAST),--multicast)
