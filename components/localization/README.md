# localization

State estimation for each drone and the payload, in the one world frame the team works in.

| Path | What it is |
|---|---|
| [`castor_localization/`](castor_localization/) | `world_frame`: places the vehicle's own odometry in the world frame. |
| [`localization_launch/`](localization_launch/) | The component's launch file. |

## The world frame

East-north-up, with its origin at the **datum**: one geodetic point the whole team is given
(`datum:=<latitude>,<longitude>,<altitude>` on the launch file). The policy was trained with the payload's goals
within a metre or so of the origin, so the datum belongs where the payload is flown, not at the base station.

PX4 keeps each drone's position in its own local frame, whose origin is wherever its estimator started. The vehicle
component publishes that estimate (`<ns>/vehicle/odom`) and where the frame's origin is on the globe
(`<ns>/vehicle/local_origin`). `world_frame` turns the two into:

| Topic | |
|---|---|
| `<ns>/localization/odom` | `nav_msgs/Odometry`: the drone's pose in the world frame, twist in the body frame. Planning flies on it. |
| `<ns>/localization/map_to_odom` | `geometry_msgs/TransformStamped`, latched: PX4's local frame in the world frame. Planning turns its world-frame setpoints back into PX4's frame with it. |

Without a datum, or before PX4 has a global position, nothing is published and planning does not fly.

All receivers must use the same base station, so their errors relative to each other stay at the centimetre level.
In stack_sim the datum is the simulated world's origin (`components/simulation/assets/config/px4_sitl.yaml`).

## Not here yet

The payload's pose (from its own flight controller, over Wi-Fi) and UWB ranging. Raw UWB ranges come from the
vehicle component, which owns the UWB hardware and its firmware ([`../vehicle/uwb_firmware`](../vehicle/uwb_firmware)).
