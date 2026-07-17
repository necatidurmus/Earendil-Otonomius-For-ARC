# RscpStatus — Status of the RSCP bridge node
# Published by: earendil_rscp_bridge on /rscp/status

std_msgs/Header header

# Connection state
uint8 STATE_DISCONNECTED=0
uint8 STATE_CONNECTING=1
uint8 STATE_CONNECTED=2
uint8 STATE_ERROR=3

uint8 connection_state
string serial_port
uint32 frames_received
uint32 frames_sent
uint32 parse_errors
uint32 cobs_errors
float64 last_command_age_sec     # seconds since last RSCP command received
