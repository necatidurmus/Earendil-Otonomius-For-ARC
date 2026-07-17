# SafetyStatus — detailed safety system state
# Published by: earendil_safety / safety_node

std_msgs/Header header

# E-stop sources
bool estop_hardware_pin      # GPIO pin state
bool estop_software          # software e-stop (web/service)
bool estop_active            # combined: ANY source triggers

# Watchdog
bool watchdog_ok             # false = safety gate locked
float64 watchdog_age_ms      # ms since last valid command
float64 watchdog_timeout_ms  # configured timeout

# Deadman
bool deadman_held            # physical/web deadman switch
float64 deadman_age_ms       # ms since last deadman heartbeat

# Sensor health
bool gps_ok
bool imu_ok
bool lidar_ok
bool odom_ok
float64 gps_age_ms
float64 imu_age_ms
float64 lidar_age_ms
float64 odom_age_ms

# Speed limits enforced
float32 current_linear_limit
float32 current_angular_limit
