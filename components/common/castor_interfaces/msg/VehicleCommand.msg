# A flight-controller command from the system layer to the vehicle component
# (<ns>/vehicle/command). The vehicle component turns it into PX4's
# vehicle_command; nothing else talks to the FC. It is dropped unless the
# vehicle component was launched with enable_commands:=true.

string ARM=arm
string DISARM=disarm
string TAKEOFF=takeoff       # PX4 Takeoff mode to `altitude` above home; PX4 holds (Loiter) when it gets there
string RAPTOR=raptor         # switch to the RAPTOR external mode
string LAND=land             # PX4 Land mode at the current position; PX4 disarms on touchdown

builtin_interfaces/Time stamp
string command
float32 altitude             # [m] above home, TAKEOFF only
