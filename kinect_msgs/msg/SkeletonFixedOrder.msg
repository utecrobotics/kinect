# This message should contain the data in the following order:
#   right_shoulder, right_elbow, right_wrist, ...
#   left_shoulder, left_elbow, left_wrist, ...
#   head (optional), neck (optional), torso (optional)
geometry_msgs/Vector3[] position
geometry_msgs/Quaternion[] orientation
# Indications for opening or closing the hand
std_msgs/Bool left_hand
std_msgs/Bool right_hand
