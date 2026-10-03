# Flight controller status as seen by the vehicle component (<ns>/vehicle/state).
# Translated from PX4's vehicle_status; nothing here is commanded, only reported.

std_msgs/Header header
bool fc_connected            # a PX4 status message arrived within the link timeout
bool armed
uint8 nav_state              # PX4 vehicle_status.nav_state, passed through unchanged
bool failsafe                # PX4 is in failsafe; the system supervisor follows into EMERGENCY
bool preflight_checks_pass
float32 seconds_since_fc_message   # -1 if nothing was ever received
