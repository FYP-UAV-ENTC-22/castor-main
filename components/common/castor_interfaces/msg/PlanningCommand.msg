# What the system layer asks of the planning runner (<ns>/planning/command).
# Sent continuously (10 Hz) while the mode is not OFF; the runner falls back to
# OFF when the command goes stale, and RAPTOR then holds the vehicle where it is.

string OFF=off         # publish nothing: RAPTOR holds by itself
string STEADY=steady   # stream a fixed setpoint: the last one flown, or the drone's position
string LIFT=lift       # climb straight up until the payload hangs at the model's lift height
string POLICY=policy   # the MARL policy flies the payload to `goal`

std_msgs/Header header
string mode
geometry_msgs/Pose goal      # payload goal in the world frame (ENU), POLICY only
