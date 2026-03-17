# MissionState.msg
# Rover'ın anlık görev durumunu yayınlar
# Topic: /mission/state

# Görev durumları
uint8 IDLE         = 0
uint8 LOCALIZING   = 1
uint8 GPS_NAV      = 2
uint8 SLAM_NAV     = 3
uint8 TASK_EXEC    = 4
uint8 RETURNING    = 5
uint8 COMPLETE     = 6
uint8 EMERGENCY    = 7
uint8 PAUSED       = 8

std_msgs/Header header
uint8 state                     # Mevcut durum (yukarıdaki sabitlerden biri)
string state_name               # İnsan okunabilir durum adı
string active_task              # Aktif görev adı ("aruco", "rock", "peak", "tunnel", "")
float32 mission_elapsed_secs    # Görev başından beri geçen süre
float32 mission_progress_pct    # Tamamlanan yüzde (0.0–100.0)
string localization_mode        # "gps" | "slam" | "unknown"
float32 gps_quality             # 0.0–1.0
bool e_stop_active              # E-stop aktif mi
string last_error               # Son hata mesajı (varsa)
