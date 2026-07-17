# RscpCommand — Internal representation of an RSCP request command
# Published by: earendil_rscp_bridge on /rscp/command
# Subscribed by: earendil_safety (arm/disarm gate), earendil_navigation (mission manager)

std_msgs/Header header

# Command type constants
uint8 CMD_SET_STAGE=0
uint8 CMD_ARM=1
uint8 CMD_DISARM=2
uint8 CMD_NAVIGATE_TO_GPS=3
uint8 CMD_SEARCH_AREA=4
uint8 CMD_START_EXPLORATION=5

uint8 command_type

# Payload (used depending on command_type)
uint32 stage_value                          # CMD_SET_STAGE: 1-4
float64 latitude                            # CMD_NAVIGATE_TO_GPS, CMD_SEARCH_AREA
float64 longitude                           # CMD_NAVIGATE_TO_GPS, CMD_SEARCH_AREA
float32 altitude                            # CMD_NAVIGATE_TO_GPS, CMD_SEARCH_AREA (EGM96)
float32 search_radius                       # CMD_SEARCH_AREA (metre)
