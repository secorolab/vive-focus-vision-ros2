# Eye tracking from the headset, one sample for both eyes.
#
# There is no standard gaze message in ROS 2 (nothing in sensor_msgs, geometry_msgs or
# vision_msgs covers it), which is why this type exists. It mirrors what the runtime actually
# reports rather than inventing a tidier shape: per eye, each quantity carries its own validity,
# because the tracker can lose one eye, or hold the pose while dropping the pupil measurement.
#
# Poses are rays, not points: position is the eye, orientation points along the gaze. Expressed
# in header.frame_id - vr_origin as published by the headset, world after InputNode calibrates.

std_msgs/Header header

geometry_msgs/Pose left
geometry_msgs/Pose right
bool left_valid
bool right_valid

# Pupil diameter in millimetres, as the runtime reports it.
float32 left_pupil_diameter_mm
float32 right_pupil_diameter_mm
bool left_pupil_valid
bool right_pupil_valid
