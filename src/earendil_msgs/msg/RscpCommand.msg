# RscpCommand.msg
# Uzak istasyondan gelen RSCP komut mesajı
# Topic: /rscp/command

# Komut tipleri
uint8 START        = 0
uint8 STOP         = 1
uint8 PAUSE        = 2
uint8 RESUME       = 3
uint8 E_STOP       = 4
uint8 GO_HOME      = 5
uint8 TASK_SELECT  = 6
uint8 MANUAL_CTRL  = 7

std_msgs/Header header
uint8 command_type              # Komut tipi (yukarıdaki sabitlerden biri)
string command_name             # İnsan okunabilir komut adı
string payload                  # Ek veri (JSON string, task parametreleri vb.)
string source                   # Komut kaynağı ("rscp_remote" | "matlab" | "local")
uint32 sequence_id              # Komut sıra numarası (tekrar algılama için)
