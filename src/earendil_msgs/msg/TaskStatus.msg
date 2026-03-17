# TaskStatus.msg
# Görev yürütme durumu
# Topic: /mission/task_status

uint8 PENDING    = 0
uint8 RUNNING    = 1
uint8 SUCCESS    = 2
uint8 FAILED     = 3
uint8 CANCELLED  = 4

std_msgs/Header header
string task_name                # Görev adı ("aruco", "rock", "peak", "tunnel")
uint8 status                    # Durum
string result_data              # Sonuç verisi (JSON, marker pozisyonu vb.)
float32 elapsed_secs            # Görev süresi
string error_msg                # Hata mesajı (varsa)
