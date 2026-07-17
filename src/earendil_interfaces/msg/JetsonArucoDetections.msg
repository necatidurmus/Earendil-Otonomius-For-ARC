# JetsonArucoDetections — ArUco tag detection results from Jetson stereo camera
# Published by: Jetson-side aruco_detector_node on /jetson/aruco_detections
# Subscribed by: earendil_navigation (mission manager), earendil_jetson_bridge

std_msgs/Header header

# Single detection
uint32[] ids                     # ArUco tag IDs detected
float64[] distances              # estimated distance to each tag (metres)
float64[] position_x             # x position in camera frame (metres)
float64[] position_y             # y position in camera frame (metres)
float64[] position_z             # z position in camera frame (metres)
float64[] yaw                    # yaw angle of each tag relative to camera (radians)
