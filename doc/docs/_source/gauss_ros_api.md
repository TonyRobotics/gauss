# ros 接口列表

## Ｔopic

### 机械臂的关节状态接口

- 接口： /joint_states
- ROS 接口类型： Topic
- 接口内容： sensor_msgs/JointState

### 硬件信息接口

- 接口： /gauss/hardware_status
- ROS 接口类型： Topic
- 接口内容： gauss_msgs/HardwareStatus

### 机器人的空间状态接口

- 接口： /gauss/robot_state
- ROS 接口类型： Topic
- 接口内容： gauss_msgs/RobotState

### 示教模式状态接口

- 接口： /gauss/learning_mode
- ROS 接口类型： Topic
- 接口内容：　std_msgs/Bool

## Service 

### 示教模式切换接口
- 接口： /gauss/activate_learning_mode
- ROS接口类型： Service
- 接口内容： gauss_msgs/SetInt 

### 校准接口

- 接口： /gauss/calibrate_motors
- ROS 接口类型： Service
- 接口内容： gauss_msgs/SetInt

### 重新校准

- 接口：　/gauss/request_new_calibration
- ROS 接口类型： Service
- 接口内容： gauss_msgs/SetInt

### 机械臂停止接口

- 接口： /gauss/commander/stop_command
- ROS接口类型： Service
- 接口内容： ｓtd_srvs/SetBool

### 更换工具接口

- 接口： /gauss/change_tool
- ROS接口类型： Service
- 接口内容： gauss_msgs/SetInt

## Action

### 运动规划接口

- 接口： /gauss/commander/robot_action
- ROS 接口类型： Action
- 接口内容： gauss_msgs/RobotMoveAction

### 工具控制接口

- 接口： /gauss/commander/robot_action
- ROS 接口类型： Action
- 接口内容： gauss_msgs/RobotMoveAction



