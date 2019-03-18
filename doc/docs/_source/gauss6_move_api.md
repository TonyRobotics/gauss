# 运动规划接口

接口文件： gauss/gauss_python_api/src/gauss_python_api/gauss_api.py

- 接口： gauss/commander/robot_action
- ROS 接口类型： Action
- 接口内容： RobotMoveAction

具体使用请参考 code 目录下的 `move_joint.py` 和 `move_pose.py` 文件。

move接口可以完成如下的规划任务：
```
JOINTS     = 1
POSE       = 2
POSITION   = 3
RPY        = 4
SHIFT_POSE = 5
# TOOL       = 6
# EXECUTE_TRAJ = 7
# POSE_QUAT  = 8
# SAVED_POSITION =9
# SAVED_TRAJECTORY = 10 
```

注意：Gauss机械臂提供了关节坐标系、直角坐标系的接口，暂未提供笛卡尔路径规划接口。
POSE、POSITION、RPY、SHIFT_POSE 等规划依赖于规划器和逆解器算法。如有发现规划失败，则需要考虑：

1. 是否超出了机械臂的可达范围
2. 更换OMPL所使用的插件： 配置文件在 ompl_planning.yaml
3. 考虑是否需要更换moveit规划器：https://moveit.ros.org/documentation/planners/
4. 考虑是否需要更换逆解器：ikfast、trac_ik等


参考：
1、 https://bitbucket.org/traclabs/trac_ik/src/HEAD/trac_ik_kinematics_plugin/