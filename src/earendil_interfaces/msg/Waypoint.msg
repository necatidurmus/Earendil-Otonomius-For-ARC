# Waypoint — single waypoint with position and optional heading

geometry_msgs/Point position      # x, y, z (map frame)
float32 yaw_deg                     # desired heading in degrees
float32 speed_limit                  # m/s, 0 = use default
float32 arrival_radius               # m, 0 = use default
string label                         # human-readable name (optional)
