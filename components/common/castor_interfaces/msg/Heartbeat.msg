# Published once a second by every CASTOR component on <ns>/<component>/heartbeat.
# The system supervisor uses it to tell which components are up.

builtin_interfaces/Time stamp
string component        # vehicle | localization | planning | system
uint32 robot_id         # robot.id from robot.yaml
string image_revision   # git revision the running image was built from
uint64 sequence
