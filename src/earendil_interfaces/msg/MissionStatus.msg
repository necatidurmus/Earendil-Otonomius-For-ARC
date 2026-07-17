# MissionStatus — progress of an active waypoint mission
# Published by: earendil_navigation / mission_manager

std_msgs/Header header

uint8 STATUS_IDLE=0
uint8 STATUS_RUNNING=1
uint8 STATUS_PAUSED=2
uint8 STATUS_SUCCEEDED=3
uint8 STATUS_FAILED=4
uint8 STATUS_CANCELLED=5

uint8 status
string mission_name
uint32 total_waypoints
uint32 current_waypoint_index

# Current target
geometry_msgs/PoseStamped current_target

# Timing
float64 elapsed_time_sec
float64 estimated_remaining_sec

# Results
uint32 succeeded_count
uint32 failed_count
string last_error

# Lifecycle (dahili, RSCP'den bağımsız)
#   0=RUNNING, 1=PAUSED, 2=CANCELLED
uint8 lifecycle_state

# Localization mode (GPS↔SLAM trigger)
#   "gps" veya "slam". SLAM kaynağı M8 sonrası bağlanır.
string localization_mode
