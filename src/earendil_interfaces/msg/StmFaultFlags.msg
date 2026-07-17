# StmFaultFlags — F411 motor controller fault flags (14 faults)
# Published by: earendil_control/stm_bridge on /stm/fault_flags

std_msgs/Header header

# Motor IDs
uint8 MOTOR_FL=0
uint8 MOTOR_FR=1
uint8 MOTOR_RL=2
uint8 MOTOR_RR=3

# Fault flags per motor (true = fault active)
bool[4] overcurrent
bool[4] overtemperature
bool[4] hall_error
bool[4] undervoltage
bool[4] overvoltage
bool[4] driver_error
bool[4] stall

# Aggregate
bool any_fault
uint8 fault_count
