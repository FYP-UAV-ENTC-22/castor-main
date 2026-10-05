# What the system layer asks of the planning policy (<ns>/planning/command).
# Sent continuously while the policy should fly; the policy stops publishing
# setpoints when enable is false or the command goes stale, and RAPTOR then
# holds the vehicle where it is.

std_msgs/Header header
bool enable
geometry_msgs/Pose goal      # payload goal in the world frame (ENU)
