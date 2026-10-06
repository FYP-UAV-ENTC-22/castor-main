# The system mission node's state (<ns>/system/mission). Drones read each
# other's to know when the whole team is at take-off height.

string WAIT_VEHICLE=WAIT_VEHICLE   # no fresh FC link yet
string IDLE=IDLE                   # on the ground, disarmed, waiting for /team/command "takeoff"
string ARMING=ARMING               # PX4 Takeoff mode first, then arm (PX4 climbs once armed)
string TAKING_OFF=TAKING_OFF       # PX4 Takeoff mode
string HANDOVER=HANDOVER           # switching PX4 to RAPTOR at take-off height
string HOVER=HOVER                 # RAPTOR holding position, cables slack; waiting for the team to get here
string LIFT=LIFT                   # the team climbs together: the cables go taut, the payload lifts
string LIFTED=LIFTED               # holding the lifted payload; hand-over check
string READY=READY                 # hand-over passed; waiting for the team and a goal on /team/goal
string MARL=MARL                   # the planning policy is flying the team to the goal
string HOLD=HOLD                   # goal reached; the policy keeps running to hold the payload there
string LANDING=LANDING             # PX4 Land mode
string FC_OVERRIDE=FC_OVERRIDE     # PX4 left the expected mode or is in failsafe; planning stopped

std_msgs/Header header
uint32 robot_id
uint32 team_index
string state
bool team_ready                    # every teammate is ready for the policy (READY, MARL or HOLD)
geometry_msgs/Pose goal            # payload goal in the world frame, valid in MARL and HOLD
