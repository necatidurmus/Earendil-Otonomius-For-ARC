# MissionCommand — dahili/operatör kökenli lifecycle komutları
# RSCP kökenli DEĞIL. RscpCommand.msg'den tamamen bağımsız.
# RSCP bridge bu topic'i subscribe etmez. Web + earendil.py + testtalker üretir.
# Topic: /mission/control (QoS 10). Subscribed by: earendil_navigation / mission_manager

std_msgs/Header header

# Command type constants
uint8 CMD_PAUSE=0
uint8 CMD_RESUME=1
uint8 CMD_CANCEL=2
uint8 CMD_RESET=3
uint8 CMD_SKIP=4
uint8 CMD_LOAD_PRESET=5
uint8 CMD_LOAD_CSV=6
uint8 CMD_START_SWEEP=7

uint8 command_type

# Payload (komut tipine göre):
#   CMD_LOAD_PRESET -> preset adı (conservative/aggressive/test)
#   CMD_LOAD_CSV    -> CSV dosya yolu
#   CMD_START_SWEEP -> JSON sweep config: {"param":"<name>","values":[...]}
string payload
