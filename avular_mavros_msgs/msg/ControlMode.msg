# ControlMode message

std_msgs/Header header

# Type of control mode used.
uint8 CONTROL_MODE_NONE = 0
uint8 CONTROL_MODE_POSITION = 1        # position control (position, velocity, acceleration)
uint8 CONTROL_MODE_VELOCITY = 2        # velocity control (velocity, acceleration)
uint8 CONTROL_MODE_ACCELERATION = 3    # acceleration control (acceleration)
uint8 CONTROL_MODE_ATTITUDE_THRUST = 4 # attitude control (attitude, thrust)
uint8 CONTROL_MODE_WAYPOINT = 5        # waypoint control (waypoint)

uint8 control_mode

# Type of heading control mode used.
uint8 HEADING_MODE_NONE = 20
uint8 HEADING_MODE_ANGLE = 21
uint8 HEADING_MODE_RATE = 22

uint8 heading_mode
