# system

The supervisor: a YASMIN state machine for the vehicle's modes, with behaviour
trees for task sequencing inside them, plus safety supervision and
mission-resilience (MRM) behaviour. Runs on every drone's Raspberry Pi.

The flight controller handles emergencies first and directly; this component
follows it into its emergency states and is never in the emergency path itself.

Two nodes:

- `supervisor`: the state machine above. It follows the FC and owns the update
  gate; it commands nothing.
- `mission` (temporary, the simplest form until the supervisor's mission trees
  replace it): take off, wait for the team and a goal, fly the planning policy,
  hold, land. On `/team/command` `takeoff` it sends `<ns>/vehicle/command` arm
  and takeoff, then switches PX4 to RAPTOR at `takeoff_height`. In HOVER it waits
  until every teammate is at height (it reads only their `<ns>/system/mission`)
  and a goal arrives on `/team/goal` (`geometry_msgs/PoseStamped`, payload pose,
  world frame); then it enables the policy on `<ns>/planning/command`. When the
  planning status reports the goal reached it goes to HOLD with the policy still
  running. `/team/command` `land` stops the policy and sends land. If PX4 goes
  into failsafe or leaves RAPTOR, the policy stops and PX4 stays in charge
  (FC_OVERRIDE). The commands reach PX4 only if the vehicle component was
  launched with `enable_commands:=true` (SIL does that; a Pi does not).
