# FFT Drone Internet Gateway

> Copied from the retired `drone-ops` repo (2026-10-02). For CASTOR it is how
> Pis on the field network get the internet access `deploy/pi/castor-update.sh`
> needs to pull images from GHCR. One change from the original:
> `pi_fft_gateway.sh` no longer takes the Pi's password as an argument; sudo
> prompts for it on the Pi over an interactive `ssh -t` session.

Lets Raspberry Pi 5 drone companion computers on the long-range **FFT** wifi
network (which has no internet, no working default gateway) reach the
internet through a teammate's laptop acting as a NAT gateway on its second
(internet-connected) interface — without breaking that laptop's own normal
internet use.

**Portable by design**: everyone on the team is often connected to FFT at
once, but usually only one person's laptop has actual internet at any given
moment (dongle, hotspot, ethernet — whatever). Both scripts auto-detect
interfaces at run time (by SSID / by testing real connectivity, not by
assumed device names or "has a default route"), so:
- `base_station_setup.sh` is safe for **any teammate** to run on **any
  laptop**. If that laptop currently has no working internet, it detects
  that and aborts cleanly — it does not guess or half-configure anything.
- `pi_fft_gateway.sh` points a drone at *whoever runs it* by default (your
  own FFT IP), or at an explicit gateway IP if you pass one.
- `teammate_fft_client.sh` does the same thing `pi_fft_gateway.sh` does for
  a Pi, but for a **teammate's own laptop** - with an extra safety net,
  since a laptop (unlike a Pi) might already have its own real internet
  that must never be demoted. See its own section below.

## Topology

```
Internet ── <auto-detected WAN iface> ── [ TEAMMATE'S LAPTOP ] ── <auto-detected FFT iface> ── drone Pis
                                          (whoever currently has internet)     192.168.1.0/24
```

- **Gateway machine**: whichever teammate's laptop currently has internet.
  Runs `base_station_setup.sh`, which finds the FFT interface (by SSID) and
  the internet interface (by testing connectivity), then NATs between them.
- **Drones**: Raspberry Pi 5s running Ubuntu 24.04, on FFT at
  `192.168.1.17x`, reached over SSH at `<hostname>.local`.
  - drone1: `drone1-pi.local`, user `drone1`
  - drone2/drone3: same pattern, added later

FFT's own DHCP advertises a dead-end gateway (`192.168.1.1`, unreachable) on
every machine — it's already at a low route priority everywhere, so it
doesn't need to be removed, just out-prioritized.

## Usage

### 1. Become the gateway (whoever currently has internet)

```
sudo ./base_station_setup.sh
```

Auto-detects your FFT interface and your internet interface, sets up
IP forwarding + NAT/FORWARD rules (persisted across reboot), and prints your
FFT-side IP at the end. If you don't have internet right now, it says so and
exits without touching anything — just have someone else run it instead.

Safe to re-run (e.g. after switching from ethernet to a phone hotspot): it
removes its own previously-added rules before adding fresh ones, so nothing
stale piles up.

### 2. Point a drone at the gateway

```
./pi_fft_gateway.sh <pi-host> <pi-user> [gateway-ip]
```

Examples:
```
# Point drone1 at ME (auto-detects my own FFT IP - I just ran base_station_setup.sh)
./pi_fft_gateway.sh drone1-pi.local drone1

# Point drone2 at a specific teammate's gateway IP instead
./pi_fft_gateway.sh drone2-pi.local drone2 192.168.1.42
```

Sets DNS (`8.8.8.8`) and a default route via the gateway IP on the drone's
`FFT` NetworkManager profile, applied live (`nmcli device reapply`, no wifi
drop) and persisted via netplan (survives reboot — confirmed by an actual
reboot test on drone1).

Re-running is safe / repointing is clean: it clears any previously-set
custom route on the Pi before adding the new one, so switching which
teammate a drone routes through never leaves stale routes behind.

Prereq on the Pi: already joined to FFT with a saved NetworkManager profile
literally named `FFT` (check with `nmcli connection show` on the Pi if
unsure).

### 3. Give a teammate's own laptop internet via FFT (optional)

If a teammate has no internet source of their own right now, they can
borrow whoever's currently running `base_station_setup.sh` - **as a
fallback only**, never overriding their own real internet if they have one.
Run this locally (with `sudo`) on your own laptop, not over SSH:

```
sudo ./teammate_fft_client.sh on <gateway-ip>
sudo ./teammate_fft_client.sh off      # revert any time
```

Unlike `pi_fft_gateway.sh`, this doesn't just add a low-metric route and
trust it - a laptop might already have real internet that must not be
demoted, and NetworkManager's automatic route metrics don't always match
what you ask for (observed on this project: requesting metric `150` on one
machine actually resulted in kernel metric `20150`, because NM added its
own ~20000 baseline to that connection - the number you pass is a request,
not a guarantee). So the script:
1. Requests a route metric based on this machine's *currently observed*
   best real-internet metric + 50.
2. Applies it, then **re-reads the actual kernel route table** and checks
   the fallback route really does have a worse (higher) metric than this
   machine's own best real default route.
3. If that check fails for any reason, it **automatically rolls back** -
   clears the route/DNS it just added - rather than risk leaving a
   misconfigured route that silently steals traffic from real internet.

Verified on this project's base station laptop (which has its own real
internet): enabling the fallback left the real connection's route
unchanged and lowest-metric throughout, confirmed by testing actual
internet reachability before/after, then cleanly reverted with `off`.

## What each script changes

### `base_station_setup.sh` (on the gateway machine)
- `net.ipv4.ip_forward=1`, persisted in `/etc/sysctl.conf`
- iptables (tagged with comment `drone-ops-fft-gateway` for clean re-runs):
  - `POSTROUTING -o <WAN_IF> -j MASQUERADE`
  - `FORWARD -i <FFT_IF> -o <WAN_IF> -j ACCEPT`
  - `FORWARD -i <WAN_IF> -o <FFT_IF> -m state --state ESTABLISHED,RELATED -j ACCEPT`
- Persisted via `iptables-persistent` / `netfilter-persistent` →
  `/etc/iptables/rules.v4`
- Note: installing `iptables-persistent` on a machine that has `ufw`
  installed but *inactive* will remove the `ufw` package (apt dependency
  conflict). No functional change if ufw wasn't enabled; reinstall with
  `sudo apt-get install ufw` if you want it back. If `ufw` is *active* when
  you run this script, it adjusts `DEFAULT_FORWARD_POLICY` and adds a route
  rule instead of colliding with it.

### `pi_fft_gateway.sh` (on the drone Pi, via SSH)
On the Pi's `FFT` NetworkManager connection profile:
- `ipv4.dns=8.8.8.8`, `ipv4.ignore-auto-dns=yes` (FFT provides no DNS)
- `ipv4.routes` cleared, then set to `0.0.0.0/0 <gateway-ip> 100` (metric
  100 beats the dead-end DHCP default's higher metric)

### `teammate_fft_client.sh` (on a teammate's own laptop, local)
On that laptop's `FFT` NetworkManager connection profile:
- `ipv4.dns=8.8.8.8`, `ipv4.ignore-auto-dns=yes` (scoped to the FFT
  connection only - does not touch DNS on your other connections)
- `ipv4.routes` cleared, then set to `0.0.0.0/0 <gateway-ip> <metric>`,
  where `<metric>` is picked and *verified* to lose to your own real
  internet (see "Give a teammate's own laptop internet via FFT" above) -
  auto-rolled-back if verification fails

## Verifying

From a drone Pi:
```
ip route                    # default via <gateway-ip> dev wlan0 metric 100
resolvectl dns wlan0        # 8.8.8.8
ping -c 2 <gateway-ip>      # gateway over FFT
ping -c 2 8.8.8.8           # raw internet
ping -c 2 google.com        # DNS + internet
```

From the gateway machine:
```
sudo iptables -L FORWARD -n -v   # packet counters on the tagged rules should increase
ip route get 8.8.8.8             # should go out your WAN interface, not FFT
```

## Ending a session / reversing

**None of this is required for safety** — everything here fails closed by
design. Unplug your internet source, or just walk away, and drones/
teammates pointed at you simply lose internet with no error and no side
effect; reconnecting resumes everything automatically (verified: pulled a
gateway's rules mid-session, confirmed the Pi failed closed with clean SSH/
FFT connectivity but no internet, then restored the gateway and the Pi
regained internet on its own with zero Pi-side changes). The commands below
are for actively tidying up (handing off a laptop, not wanting to share
anymore), not something you must remember to run.

**Stop one drone from routing through a gateway:**
```
./pi_fft_gateway.sh <pi-host> <pi-user> off
```
Falls back to FFT-only (no internet), as before.

**Stop a teammate's laptop from using the FFT fallback** (run on that
laptop):
```
sudo ./teammate_fft_client.sh off
```

**Stop a gateway machine from acting as a gateway:**
```
sudo ./stop_gateway.sh
```
Removes the NAT/FORWARD rules this project added (by comment tag) and
persists the removal. Leaves `net.ipv4.ip_forward=1` in place — harmless
with no NAT/FORWARD rules to act on.

**Just unplugging the internet source:** no action needed, see above.

## Files

- `lib/detect_fft.sh` — shared detection functions (source, don't execute)
- `base_station_setup.sh` — run with `sudo` on whichever laptop currently
  has internet
- `pi_fft_gateway.sh` — run from that laptop against a drone Pi:
  `./pi_fft_gateway.sh <pi-host> <ssh-user> <ssh-password> [gateway-ip]`
- `teammate_fft_client.sh` — run locally (with `sudo`) on a teammate's own
  laptop: `./teammate_fft_client.sh on <gateway-ip>` /
  `./teammate_fft_client.sh off`
- `stop_gateway.sh` — run with `sudo` on a gateway machine to stop it
  acting as one (optional tidy-up, not required for safety)
