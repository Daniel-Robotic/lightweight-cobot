# Actual TCP pose; enabled is false while any higher-priority motion owns control.
geometry_msgs/PoseStamped actual
bool enabled
# Advances on rejection, controller completion, or higher-priority ownership.
uint64 epoch
string reason
