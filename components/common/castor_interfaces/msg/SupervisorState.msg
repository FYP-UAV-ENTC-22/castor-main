# The system supervisor's state machine, on <ns>/system/state.

std_msgs/Header header
string state                 # BOOT, WAIT_COMPONENTS, IDLE, PREFLIGHT, READY, MISSION, LANDING, EMERGENCY
bool emergency
string emergency_reason
bool update_allowed          # mirrors /run/castor/update_gate
string[] components_up       # components whose heartbeat is fresh
