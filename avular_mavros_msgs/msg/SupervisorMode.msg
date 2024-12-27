# Vertex Supervisor Mode

# Header containing a timestamp
std_msgs/Header header

#Main mode of the supervisor
uint8 SUPERVISOR_MODE_NONE       = 0
uint8 SUPERVISOR_INIT            = 1     # Flight Controller initializing
uint8 SUPERVISOR_BOOT_OK         = 2     # Flight Controller booted up succesfully
uint8 SUPERVISOR_CALIBRATING     = 3     # Flight Controller calibrating sensors
uint8 SUPERVISOR_READY           = 4     # Flight Controller ready 
uint8 SUPERVISOR_E_STOP          = 5     # Flight Controller in emergency stop state, reboot required

uint8   mode_supervisor  

# Armed mode of the supervisor 
# Supervisor mode inside the flight mode 'ARMED' otherwise value is 'ARMED_MODE_NONE'
uint8   ARMED_MODE_NONE         = 10
uint8   ARMED_MODE_IDLE 	    = 11     # Flight Controller in idle mode
uint8   ARMED_MODE_TAKE_OFF     = 12     # Flight Controller in take-off mode
uint8   ARMED_MODE_IN_FLIGHT    = 13     # Take-off completed, in flight
uint8   ARMED_MODE_LANDING      = 14     # Flight Controller in landing mode 
uint8   ARMED_MODE_FAIL_SAFE    = 15     # Flight Controller in emergency landing

uint8   mode_armed 

# Supervisor mode of operation
# Mode of operation when main mode is 'READY' otherwise value is 'MODE_N0NE'
uint8   OPERATION_MODE_NONE                 = 20
uint8   OPERATION_MODE_ATTITUDE_MANUAL      = 21     # Manual RC control over attitude angles, altitude velocity and yaw velocity
uint8   OPERATION_MODE_POSITION_MANUAL      = 22     # Manual RC control over horizontal velocity in body frame, altitude velocity and yaw velocity
uint8   OPERATION_MODE_POSITION_EXTERNAL    = 23     # External control while following waypoints or commands.

uint8   mode_of_operation

# Supervisor flight mode
# Supervisor mode inside the main 'READY' mode  otherwise value is 'MODE_N0NE'
uint8   FLIGHT_MODE_NONE                = 30
uint8   FLIGHT_MODE_PRE_FLIGHT_CHECK    = 31     # Basic sensor and RC checks
uint8   FLIGHT_MODE_DISARMED            = 32     # Flight Controller ready, motors disarmed
uint8   FLIGHT_MODE_ARMED               = 33     # Flight Controller ready, motors armed
uint8   FLIGHT_MODE_EMERGENCY_STOP      = 34     # Flight Controller in emergency stop mode, motors stopped

uint8   mode_of_flight

# External supervisor mode
uint8   EXTERNAL_MODE_NONE              = 40
uint8   EXTERNAL_MODE_UNDEFINED         = 41     # Undefined external control
uint8   EXTERNAL_MODE_POSITION          = 42     # External control over position
uint8   EXTERNAL_MODE_VELOCITY          = 43     # External control over velocity
uint8   EXTERNAL_MODE_ACCELERATION      = 44     # External control over acceleration
uint8   EXTERNAL_MODE_ATTITUDE_THRUST   = 45     # External control over attitude and thrust
uint8   EXTERNAL_MODE_WAYPOINT          = 46     # External control over waypoints

uint8   mode_external_control
