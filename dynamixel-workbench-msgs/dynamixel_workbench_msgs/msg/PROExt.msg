# This message is compatible with control table of Dynamixel PRO Series
# (L54-30-S500-R, L54-30-S400-R,  L54-50-S500-R, L54-50-S290-R,
#  M42-10-S260-R, M54-40-S250-R,  M54-60-S250-R,
#  H42-20-S300-R, H54-100-S500-R, H54-200-S500-R)
# If you want to specific information about control table, please follow the link (http://emanual.robotis.com/)

uint16 Model_Number
uint8  Firmware_Version
uint8  ID
uint8  Baud_Rate
uint8  Return_Delay_Time
uint8  Operating_Mode
int32  Homing_Offset
uint32 Moving_Threshold
uint8  Temperature_Limit
uint16 Max_Voltage_Limit
uint16 Min_Voltage_Limit
uint32 Acceleration_Limit
uint16 Torque_Limit
uint32 Velocity_Limit
int32  Max_Position_Limit
int32  Min_Position_Limit
uint8  External_Port_Mode_1
uint8  External_Port_Mode_2
uint8  External_Port_Mode_3
uint8  External_Port_Mode_4
uint8  Shutdown

uint8  Torque_Enable
uint8  LED_RED
uint8  LED_GREEN
uint8  LED_BLUE
uint16 Velocity_I_Gain
uint16 Velocity_P_Gain
uint16 Position_P_Gain
int32  Goal_Position
uint32 Goal_Velocity
uint16 Goal_Torque
int32  Goal_Acceleration
uint8  Moving
int32  Present_Position
uint32 Present_Velocity
uint16 Present_Current
uint16 Present_Input_Voltage
uint8  Present_Temperature
uint8  Registered_Instruction
uint8  Status_Return_Level
uint8  Hardware_Error_Status


