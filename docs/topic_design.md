# Topic Design — Earendil RPi5 Vehicle

> All topics, message types, QoS settings, and publishers/subscribers for the Earendil rover.
> Reference: CLAUDE.md §7, ROADMAP.md §6.

## RSCP Bridge Topics

| Topic | Type | Publisher | Subscriber | QoS | Description |
|---|---|---|---|---|---|
| `/rscp/current_stage` | `std_msgs/UInt32` | rscp_bridge | mission_manager | reliable, volatile | Current ARC stage (1-4, 0=idle) |
| `/rscp/command` | `earendil_interfaces/RscpCommand` | rscp_bridge | safety_mux, mission_manager | reliable, volatile | Parsed RSCP command (type + payload) |
| `/rscp/status` | `earendil_interfaces/RscpStatus` | rscp_bridge | watchdog, diagnostics | best_effort, volatile | Bridge health (connection, errors, counters) |

## Mission Topics

| Topic | Type | Publisher | Subscriber | QoS | Description |
|---|---|---|---|---|---|
| `/mission/command` | `earendil_interfaces/RscpCommand` | rscp_bridge | mission_manager | reliable, volatile | Mission-level command (navigate, search, explore) |
| `/mission/status` | `earendil_interfaces/MissionStatus` | mission_manager | rscp_bridge, web | reliable, volatile | Mission progress (state, waypoint index) |
| `/mission/result` | `std_msgs/String` | mission_manager | rscp_bridge | reliable, volatile | Mission result signal ("task_finished", "gps_coordinate", "distance") |

## Sensor Topics

| Topic | Type | Publisher | Subscriber | QoS | Description |
|---|---|---|---|---|---|
| `/gps/fix` | `sensor_msgs/NavSatFix` | gps_adapter | mission_manager, rscp_bridge, localization | best_effort, volatile | RTK GPS position |
| `/rtk/status` | `std_msgs/String` | gps_adapter | mission_manager, rscp_bridge | best_effort, volatile | RTK fix quality (FIXED/FLOAT/DGPS/SPS/NO_FIX) |
| `/scan` | `sensor_msgs/LaserScan` | lidar_adapter | Nav2, obstacle_detector | best_effort, volatile | LiDAR scan data |
| `/stm/imu/data` | `sensor_msgs/Imu` | stm_bridge | localization, rscp_bridge | best_effort, volatile | STM MPU9250 IMU (ENU frame) |
| `/stm/magnetic_field` | `sensor_msgs/MagneticField` | stm_bridge | localization | best_effort, volatile | STM QMC5883P magnetometer |
| `/stm/wheel_odom` | `nav_msgs/Odometry` | stm_bridge | localization, rscp_bridge | best_effort, volatile | Wheel odometry from Hall sensors (odom→base_link) |
| `/stm/status` | `earendil_interfaces/StmStatus` | stm_bridge | watchdog, web, diagnostics | best_effort, volatile | STM H723 link + operating mode |
| `/stm/fault_flags` | `earendil_interfaces/StmFaultFlags` | stm_bridge | watchdog, web, diagnostics | best_effort, volatile | F411 motor fault flags |

## Vision Topics (Jetson)

| Topic | Type | Publisher | Subscriber | QoS | Description |
|---|---|---|---|---|---|
| `/jetson/aruco_detections` | `earendil_interfaces/JetsonArucoDetections` | jetson_bridge | mission_manager | best_effort, volatile | ArUco tag detections (id, distance, position) |
| `/jetson/obstacles` | custom | jetson_bridge | Nav2 | best_effort, volatile | Obstacle list from stereo depth |
| `/jetson/depth_summary` | custom | jetson_bridge | web | best_effort, volatile | Summary depth info |
| `/jetson/visual_odom` | `nav_msgs/Odometry` | jetson_bridge | localization | best_effort, volatile | Visual odometry (optional, default off) |
| `/jetson/imu/data` | `sensor_msgs/Imu` | jetson_bridge | — | best_effort, volatile | Camera IMU (aux only, NOT primary IMU) |

## Motion Topics

| Topic | Type | Publisher | Subscriber | QoS | Description |
|---|---|---|---|---|---|
| `/cmd_vel_nav` | `geometry_msgs/Twist` | Nav2 | safety_mux | reliable, volatile | Navigation velocity command |
| `/cmd_vel_manual` | `geometry_msgs/Twist` | web_server | safety_mux | reliable, volatile | Manual teleop (deadman required) |
| `/cmd_vel_safe` | `geometry_msgs/Twist` | safety_mux | stm_bridge | reliable, volatile | **Single safe output** — only safety_mux writes here |

## Safety Topics

| Topic | Type | Publisher | Subscriber | QoS | Description |
|---|---|---|---|---|---|
| `/e_stop` | `std_msgs/Bool` | web_server, GPIO | safety_mux | reliable, volatile | Emergency stop (latching) |
| `/deadman` | `std_msgs/Bool` | web_server | safety_mux | reliable, volatile | Deadman heartbeat |
| `/safety/status` | `earendil_interfaces/SafetyStatus` | safety_mux | web, diagnostics | best_effort, volatile | Safety gate state |

## TF Frames (REP-105)

```
map → odom → base_link → imu_link
                        → lidar_link
                        → gps_link
                        → wheel_FL
                        → wheel_FR
                        → wheel_RL
                        → wheel_RR
```

- `map → odom`: from `robot_localization` (global EKF)
- `odom → base_link`: from `robot_localization` (local EKF) or stm_bridge wheel odom
- Static transforms: `base_link → imu_link, lidar_link, gps_link, wheel_*`

## QoS Notes

- **Sensor topics**: `best_effort, volatile` — matches typical sensor publishers, no persistence needed
- **Command topics**: `reliable, volatile` — commands must be delivered, no need for durability
- **Safety topics**: `reliable, volatile` — e-stop/deadman must be delivered
