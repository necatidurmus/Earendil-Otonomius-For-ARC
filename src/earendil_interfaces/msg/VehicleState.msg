# VehicleState — combined status of the vehicle
# Published by: earendil_safety / safety_node

std_msgs/Header header

# Motion state
float32 linear_velocity      # m/s (signed: positive=forward)
float32 angular_velocity     # rad/s
float32 heading              # rad (from odometry)

# Position (from localization)
float64 position_x           # m (map frame)
float64 position_y           # m (map frame)

# Safety state
bool e_stop_active           # true = emergency stop engaged
bool deadman_active          # true = deadman switch held
bool nav_timeout             # true = navigation command timed out
bool sensor_fault            # true = critical sensor data lost

# Operational mode
uint8 MODE_IDLE=0
uint8 MODE_MANUAL=1
uint8 MODE_NAVIGATION=2
uint8 MODE_MISSION=3
uint8 current_mode

# Speed limits
float32 max_linear_speed     # m/s (current enforced limit)
float32 max_angular_speed    # rad/s (current enforced limit)
