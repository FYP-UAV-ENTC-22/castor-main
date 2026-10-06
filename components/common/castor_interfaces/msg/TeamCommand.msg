# Team-level command from the ground station, on /team/command.
# The only topic family the zenoh bridge lets into a drone by default.

string PREFLIGHT=preflight   # IDLE -> PREFLIGHT
string ABORT=abort           # PREFLIGHT/READY -> IDLE
string RESET=reset           # clears a latched EMERGENCY once the vehicle is disarmed
string TAKEOFF=takeoff       # mission: arm, take off to the take-off height, hand over to RAPTOR
string LAND=land             # mission: stop the policy and land

builtin_interfaces/Time stamp
string command
uint32[] robot_ids           # empty = every robot
