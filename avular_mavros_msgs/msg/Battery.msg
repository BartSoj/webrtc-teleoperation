# Battery information message
# For all field holds, if the field is unavailable the max value will be provided (i.e. UINT32_MAX)

# Battery alert constants
uint32 BATTERY_ALERT_CELL_UNDERVOLTAGE = 1
uint32 BATTERY_ALERT_CELL_OVERVOLTAGE  = 2
uint32 BATTERY_ALERT_OVERCURRENT_CHARGE = 4
uint32 BATTERY_ALERT_OVERCURRENT_DISCHARGE = 8
uint32 BATTERY_ALERT_OVERLOAD_DISCHARGE = 16
uint32 BATTERY_ALERT_OVERLOAD_DISCHARGE_LATH = 32
uint32 BATTERY_ALERT_SHORT_CIRCUIT_DISCHARGE = 64
uint32 BATTERY_ALERT_SHORT_CIRCUIT_DISCHARGE_LATCH = 128
uint32 BATTERY_ALERT_OVERTEMPERATURE_CHARGE = 256
uint32 BATTERY_ALERT_OVERTEMPERATURE_DISCHARGE = 512
uint32 BATTERY_ALERT_UNDERTEMPERATURE_CHARGE = 1024
uint32 BATTERY_ALERT_UNDERTEMPERATURE_DISCHARGE = 2048
uint32 BATTERY_ALERT_AFE_ALERT = 4096
uint32 BATTERY_ALERT_RESERVED_1 = 32768
uint32 BATTERY_ALERT_PRECHARGE_TIMEOUT_SUSPEND = 262144
uint32 BATTERY_ALERT_RESERVED_2 = 131072
uint32 BATTERY_ALERT_CHARGE_TIMEOUT_SUSPEND = 262144
uint32 BATTERY_ALERT_RESERVED_3 = 524288
uint32 BATTERY_ALERT_OVERCHARGE = 1048576

# Battery state constants
uint8 BATTERY_STATE_READY = 0
uint8 BATTERY_STATE_PREDISCHARGING = 1
uint8 BATTERY_STATE_BALANCING = 2
uint8 BATTERY_STATE_ONE_MODULE_FAILED = 3
uint8 BATTERY_STATE_COMMUNICATION_ERROR = 4

std_msgs/Header header      

uint32      voltage             # mV
uint32      current             # positive on discharge (mA)
int16       temperature         # degree celcius
uint8       state_of_charge     # 0-100%
uint16      time_remaining      # Estimated time remaining until emergency behavior (s)
uint8       state_of_health     # 0-100%
uint32      alert               # Battery alert status
uint8       state               # Battery state