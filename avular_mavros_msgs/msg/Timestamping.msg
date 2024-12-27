# Timestamping message
std_msgs/Header header

uint64 nanoseconds_since_boot_prime
uint64 nanoseconds_since_boot_jetson

int64 timestamp_offset  # Offset from FCU to Jetson
uint64 offset_utc_boot_time # Used for conversion from boot time to UTC