# This message is compatible with control table of Dynamixel MX(2.0) Series (MX-12W(2.0), MX-28T/MX-28R/MX-28AT/MX-28AR(2.0))
# If you want to specific information about control table, please follow the link (http://emanual.robotis.com/)

uint16 Model_Number
uint8  Firmware_Version
uint8  ID
uint8  Baud_Rate
uint8  Return_Delay_Time
uint8  Drive_Mode
uint8  Operating_Mode
uint8  Secondary_ID
uint8  Protocol_Version
int32  Homing_Offset
uint32 Moving_Threshold
uint8  Temperature_Limit
uint16 Max_Voltage_Limit
uint16 Min_Voltage_Limit
uint16 PWM_Limit
uint32 Acceleration_Limit
uint32 Velocity_Limit
uint32 Max_Position_Limit
uint32 Min_Position_Limit
uint8  Shutdown

uint8  Torque_Enable
uint8  LED
uint8  Status_Return_Level
uint8  Registered_Instruction
uint8  Hardware_Error_Status
uint16 Velocity_I_Gain
uint16 Velocity_P_Gain
uint16 Position_D_Gain
uint16 Position_I_Gain
uint16 Position_P_Gain
uint16 Feedforward_2nd_Gain
uint16 Feedforward_1st_Gain
uint8  Bus_Watchdog
int16  Goal_PWM
int32  Goal_Velocity
uint32 Profile_Acceleration
uint32 Profile_Velocity
uint32 Goal_Position
uint16 Realtime_Tick
uint8  Moving
uint8  Moving_Status
int16  Present_PWM
int16  Present_Load
int32  Present_Velocity
int32  Present_Position
uint32 Velocity_Trajectory
uint32 Position_Trajectory
uint16 Present_Input_Voltage
uint8  Present_Temperature

