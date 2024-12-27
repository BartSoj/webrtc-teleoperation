# State reference for the robot to track. References are expressed in the frame_id that is specified in the header.
# The mode of the state reference can be independent for orientation and translation (e.g. position reference with yaw-rate).

uint8 MODE_UNKNOWN = 0

uint8 ORIENTATION_MODE_ATTITUDE = 1
uint8 ORIENTATION_MODE_VELOCITY = 2
uint8 ORIENTATION_MODE_ACCELERATION = 3

uint8 TRANSLATION_MODE_POSITION = 101
uint8 TRANSLATION_MODE_VELOCITY = 102
uint8 TRANSLATION_MODE_ACCELERATION = 103

# Header containing a timestamp and frame id.
std_msgs/Header header

# Modes of the state reference.
uint8 translation_mode
uint8 orientation_mode

# Pose reference containing position and orientation.
geometry_msgs/Pose pose

# Velocity reference, both linear and angular.
geometry_msgs/Twist velocity

# Acceleration reference, both linear and angular.
geometry_msgs/Twist acceleration
