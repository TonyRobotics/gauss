
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
