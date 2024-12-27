# Define a command request for the flight controller supervisor.

uint8 VERTEX_COMMAND_NONE           = 0
uint8 VERTEX_COMMAND_ARM            = 1
uint8 VERTEX_COMMAND_DISARM         = 2

uint8 VERTEX_COMMAND_TAKEOFF        = 10
uint8 VERTEX_COMMAND_LAND           = 11
uint8 VERTEX_COMMAND_ABORT_LAND     = 12

uint8 VERTEX_COMMAND_EMERGENCY_LAND = 100
uint8 VERTEX_COMMAND_KILLSWITCH     = 254

uint8 command
