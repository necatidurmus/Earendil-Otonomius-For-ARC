# StmStatus — Status of the STM32H723 middle-brain
# Published by: earendil_control/stm_bridge on /stm/status

std_msgs/Header header

# Operating mode (mirrors H723 RoverMode_t)
uint8 MODE_DISARMED=0
uint8 MODE_MANUAL=1
uint8 MODE_AUTONOMOUS=2

uint8 operating_mode
bool link_active                 # true if H723 telemetry received within timeout
float64 link_age_sec             # seconds since last H723 telemetry
uint32 uptime_sec                # H723 uptime (if reported)
uint32 commands_sent             # commands sent to H723
uint32 acks_received             # ACKs received from H723
uint32 timeouts                  # ACK timeout count
