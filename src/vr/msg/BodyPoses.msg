# Poses of the bodies that moved, rather than of every body.
#
# A furnished world is mostly scenery: a kitchen is 194 bodies, of which six are the objects
# anyone picks up. Sending all of them so one bottle can move costs 16 KB of rosbridge JSON per
# frame, which is what made a grabbed object track the ray slowly. ids[i] names the body that
# poses[i] belongs to, so a frame carries only what changed.
std_msgs/Header header
int32[] ids
geometry_msgs/Pose[] poses
