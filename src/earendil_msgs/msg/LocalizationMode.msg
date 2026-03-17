# LocalizationMode.msg
# Lokalizasyon modu durum mesajı
# Topic: /localization/mode_status

uint8 UNKNOWN    = 0
uint8 GPS        = 1
uint8 SLAM       = 2
uint8 HYBRID     = 3
uint8 DEAD_RECK  = 4

std_msgs/Header header
uint8 mode                      # Mevcut mod
string mode_name                # "gps" | "slam" | "hybrid" | "dead_reckoning"
float32 gps_quality             # 0.0–1.0
bool slam_active                # SLAM Toolbox aktif mi
bool ukf_healthy                # UKF Local/Global sağlıklı mı
