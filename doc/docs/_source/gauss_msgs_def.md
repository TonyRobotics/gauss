# 自定义消息

RobotMoveAction 的内容：
```
# goal
gauss_msgs/RobotMoveCommand cmd
---
# result
int32 status
string message
---
# feedback
gauss_msgs/RobotState state
```

RobotMoveCommand 的内容：
```
int32 cmd_type

float64[] joints
geometry_msgs/Point position
gauss_msgs/RPY rpy
gauss_msgs/ShiftPose shift
gauss_msgs/TrajectoryPlan Trajectory
geometry_msgs/Pose pose_quat
string  saved_position_name
int32 saved_trajectory_id 

gauss_msgs/ToolCommand tool_cmd

# In the future, allow a tool command to be launched at the same time as an Arm command
# 3 choices : arm only, arm + tool, tool only
# bool use_tool 
```

RobotMoveCommand 的cmd_type:
```
JOINTS     = 1
POSE       = 2
POSITION   = 3
RPY        = 4
SHIFT_POSE = 5
TOOL       = 6
EXECUTE_TRAJ = 7
POSE_QUAT  = 8
SAVED_POSITION =9
SAVED_TRAJECTORY = 10  
```

ToolCommand 的内容：
```
uint8 tool_id
uint8 cmd_type

# if gripper close
uint16 gripper_close_speed

# if gripper open
uint16 gripper_open_speed

# if vacuum pump or electromagnet grove
bool activate

# if tool is set by digital outputs (electromagnet)
uint8 gpio
```

RobotState 的内容：
```
geometry_msgs/Point position
gauss_msgs/RPY rpy
```