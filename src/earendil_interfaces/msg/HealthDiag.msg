# HealthDiag — single component health report
# Published by: each node periodically

std_msgs/Header header
string component_name          # e.g. "gps_adapter", "motor_driver"

uint8 LEVEL_OK=0
uint8 LEVEL_WARN=1
uint8 LEVEL_ERROR=2
uint8 LEVEL_STALE=3

uint8 level
string message
float64 uptime_sec
float64 last_update_age_ms     # ms since this component last published
