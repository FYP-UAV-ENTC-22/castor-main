# Deploying CASTOR to the drones

Each drone's Raspberry Pi runs the four onboard containers (vehicle,
localization, planning, system) plus the zenoh bridge, from images CI publishes
to GHCR. A Pi **never updates by itself**: no timer, no pull at boot. Images
change only when someone runs `castor-update`.

```
deploy/
├── robot.example.yaml     template for /etc/castor/robot.yaml (who this Pi is)
├── robot.laptop.yaml      the same, for a laptop with no flight controller
├── pi/
│   ├── install.sh         one-time Pi setup (run with sudo)
│   ├── castor-update.sh   pull + restart changed containers (manual; installed as `castor-update`)
│   ├── castor-stack.service   starts the stack from local images at boot
│   ├── castor-update.service  `sudo systemctl start castor-update` (manual, never enabled)
│   ├── stack.env.example  template for /etc/castor/stack.env (which image tag)
│   ├── add_my_key_to_pi.sh    put your SSH key on a Pi
│   └── fetch_logs.sh      copy /var/log/castor back to a laptop
├── fleet/update_all.sh    run castor-update on every Pi over SSH
└── network-gateway/       a laptop as internet gateway for Pis on the FFT Wi-Fi
```

## Which Pis

Any Raspberry Pi 4 or 5 running a **64-bit** OS: Ubuntu 24.04 arm64, or
Raspberry Pi OS (Bookworm) 64-bit. Both models run the same `linux/arm64`
images. A 32-bit OS cannot run them; `install.sh` refuses.

Things that differ per model live in robot.yaml, never in the images:

| | Pi 5 | Pi 4 |
|---|---|---|
| FC UART on GPIO 14/15 | `/dev/ttyAMA0` after adding `dtoverlay=uart0-pi5` to `/boot/firmware/config.txt` | `/dev/ttyAMA0` with `dtoverlay=disable-bt` (otherwise the mini UART, `/dev/ttyS0`) |
| `robot.hardware` | `rpi5` | `rpi4` |

Free disk matters on small SD cards: check `df -h /` before the first pull and
`docker system df` after.

## First-time setup of a Pi

From your laptop:

```bash
deploy/pi/add_my_key_to_pi.sh <pi-host> <pi-user>       # once per person per Pi
ssh -t <pi-user>@<pi-host>
```

On the Pi:

```bash
curl -fsSL https://raw.githubusercontent.com/FYP-UAV-ENTC-22/castor-main/main/deploy/pi/install.sh -o install.sh
sudo bash install.sh
sudo nano /etc/castor/robot.yaml     # robot.id, robot.namespace, robot.hardware, team.index, fc.*
exit                                  # log out and back in so the docker group applies
```

Then, with internet on the Pi (on FFT, see [network-gateway/](network-gateway/)):

```bash
castor-update                         # first pull; no gate check while nothing is running
sudo systemctl start castor-stack     # also starts at every boot
docker compose -f /opt/castor/src/docker/docker-compose.prod.yml ps
```

## robot.yaml

`/etc/castor/robot.yaml` is the only per-robot file. Every container mounts it
read-only, so changing it and running `sudo systemctl restart castor-stack`
moves every node to the new namespace, changes the policy's one-hot slot, and
switches device paths. See [robot.example.yaml](robot.example.yaml) for every
key. Unknown keys and bad values stop the containers at start with a message
naming each problem; check a file without starting anything with:

```bash
docker run --rm -v /etc/castor/robot.yaml:/etc/castor/robot.yaml:ro \
    ghcr.io/fyp-uav-entc-22/castor-system:main castor-config check
```

`robot.id` should equal the flight controller's `MAV_SYS_ID`, and `team.size`
must equal the number of agents the deployed policy was trained for. The
planning container reports a model whose input width doesn't fit.

## Between drones and the ground station

Inside a host, containers talk Fast DDS over localhost only. Between hosts,
each host's zenoh bridge carries only what its allow-lists name, chosen by
`zenoh.role`:

| role | sends | receives |
|---|---|---|
| `drone` | its own `/<ns>/*/heartbeat`, `/<ns>/vehicle/{odom,state}` (odom at most 20 Hz), `/<ns>/system/state`, `/<ns>/planning/status` | `/team/*` |
| `ground_station` | `/team/*` | the same topics from every drone |

Raw PX4 topics (`/fmu/*`) and all services and actions stay on the drone.
Bridges on one network find each other by multicast; list peers in
`zenoh.connect` when multicast is blocked. The ground station config is
[robot.ground-station.yaml](robot.ground-station.yaml). To widen what crosses,
edit `components/common/castor_common/castor_common/bridge_config.py`.

## Updating

```bash
castor-update --check                 # what would change
castor-update                         # pull and restart only what changed
deploy/fleet/update_all.sh            # every Pi in deploy/fleet/hosts.txt, from a laptop
```

`castor-update` refuses while the drone may be flying. The system container
writes `/run/castor/update_gate` every second, and only a fresh `allow` lets an
update through: supervisor in BOOT, WAIT_COMPONENTS or IDLE, and the vehicle
disarmed (or `fc.enabled: false`). `--force` skips the gate, for recovering a
broken stack when you know the drone is on the ground.

Every update appends the image revision and digest of each container to
`/var/log/castor/deployments.log`, so any recording can be tied to the exact
images that produced it.

**Flight days.** All drones in a team must run the same build. Pin them:

```bash
deploy/fleet/update_all.sh --tag sha-<12-char git revision>
```

or set `CASTOR_TAG=sha-...` in each Pi's `/etc/castor/stack.env`.

## Flight controller settings (not applied by anything here)

For the uXRCE-DDS link the FC needs, on the port wired to the Pi (example
TELEM2): `UXRCE_DDS_CFG` = that port, its `SER_TELx_BAUD` = `fc.baud` in
robot.yaml (921600), and `UXRCE_DDS_DOM_ID` = 20. Leave `UXRCE_DDS_PTCFG` at 0
(default participant) or set 2 (`px4_participant`): both come from
[docker/fastdds_localhost.xml](../docker/fastdds_localhost.xml) and keep PX4's
topics on the Pi. Changing FC parameters is a deliberate manual step. Nothing
in this repository writes to the flight controller.

The vehicle container never arms and never changes modes. It forwards
setpoints to PX4 only when launched with `enable_setpoint_output:=true`.

## Safety

No flight-ready aircraft exists yet. Anything that arms a vehicle or spins
motors is props-off until a written bench procedure exists for the Pixhawk 6C
mini.
