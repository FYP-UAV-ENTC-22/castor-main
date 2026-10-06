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
  replace it): take off, lift the payload with the team, wait for a goal, fly
  the planning policy, hold, land. The sequence is the one the PX4 run in
  `components/simulation/tests/marl_raptor` flew with one process for all three
  drones, split across the drones:
  1. `/team/command` `takeoff`: PX4 Takeoff mode first, then arm (PX4 boots in
     Position mode, which needs sticks, and refuses to arm there without RC).
     PX4 climbs to `takeoff_height`, below where the cables go taut.
  2. At height, PX4 switches to RAPTOR (HOVER). Once every teammate is in RAPTOR
     (it reads only their `<ns>/system/mission`), planning lifts: every drone
     climbs straight up at 0.15 m/s until the payload hangs at the model's lift
     height (LIFT).
  3. LIFTED: planning holds the setpoint; after 4 s the planning status must
     pass the hand-over check (own cable taut, payload 0.2 m clear of the
     ground), the state the policy was trained from (READY).
  4. With every teammate READY and a goal on `/team/goal`
     (`geometry_msgs/PoseStamped`, payload pose, world frame), the policy flies
     (MARL); when the planning status reports the goal reached it goes to HOLD
     with the policy still running.

  `/team/command` `land` stops planning and sends land. If PX4 goes into
  failsafe or leaves RAPTOR, planning stops and PX4 stays in charge
  (FC_OVERRIDE). The commands reach PX4 only if the vehicle component was
  launched with `enable_commands:=true` (stack_sim does that; a Pi does not).
