# Whether a grip press will engage the arm, and how far the controller still is from the tool
# pose it has to match. The tolerances travel with the errors so a client can shade an indicator
# without holding its own copy of the thresholds, which could disagree with the gate.

std_msgs/Header header

# release_grip, calibration_required, tracking_lost, align_pose, ready, waiting_delta, engaged,
# unreachable. The client needs only `ready`; this says why when it is false.
string state

# True when a press would engage: both errors are inside their tolerances and tracking is live.
bool ready

float32 position_error_m
float32 rotation_error_rad
float32 position_tolerance_m
float32 rotation_tolerance_rad
