# Team-level command from the ground station, on /team/command.
# The only topic family the zenoh bridge lets into a drone by default.

string PREFLIGHT=preflight   # IDLE -> PREFLIGHT
string ABORT=abort           # PREFLIGHT/READY -> IDLE
string RESET=reset           # clears a latched EMERGENCY once the vehicle is disarmed

builtin_interfaces/Time stamp
string command
uint32[] robot_ids           # empty = every robot
