
# 工具控制接口

注意：请在 studio 界面选择已安装的末端执行器后再使用调用接口控制末端执行器。

接口文件： gauss/gauss_python_api/src/gauss_python_api/gauss_api.py

- 接口： gauss/commander/robot_action
- ROS 接口类型： Action
- 接口内容： RobotMoveAction

工具接口支持： gripper、air_vacuum、electromagnet、DCMOTOR、laser 等工具

- gripper 为电机控制
- electromagnet、DCMOTOR、laser、air_vacuum 为 digital_io 控制

end_effectors.yaml 文件列出了各种工具的参数等信息：

```
command_list:
    # Gripper
    open_gripper: 1
    close_gripper: 2
    # Vacuump pump
    pull_air_vacuum_pump: 10
    push_air_vacuum_pump: 11
    # Tools controlled by digital I/Os
    setup_digital_io: 20
    activate_digital_io: 21
    deactivate_digital_io: 22

tool_list:
    - name: "Gripper 1"
      type: "gripper"
      id: 11
      available_commands:
          - open_gripper
          - close_gripper
      specs:
          open_position: 600
          open_hold_torque: 404
          close_position: 350
          close_hold_torque: 404
          close_max_torque: 1023
          open_speed: 300 
          close_speed: 300
    - name: "Gripper 2"
      type: "gripper"
      id: 12
      available_commands:
          - open_gripper
          - close_gripper
      specs:
          open_position: 640
          open_hold_torque: 128
          close_position: 400
          close_hold_torque: 128
          close_max_torque: 1023
          open_speed: 300 
          close_speed: 300
    - name: "Gripper 3"
      type: "gripper"
      id: 13
      available_commands:
          - open_gripper
          - close_gripper
      specs:
          open_position: 450
          open_hold_torque: 128
          close_position: 220
          close_hold_torque: 160
          close_max_torque: 1023
          open_speed: 300 
          close_speed: 300
    - name: "Electromagnet 1"
      type: "electromagnet"
      id: 30
      available_commands:
          - activate_digital_io
          - deactivate_digital_io
          - setup_digital_io
      specs:
          []
    - name: "Vacuum Pump 1"
      type: "vacuum_pump"
      id: 31
      available_commands:
          - pull_air_vacuum_pump
          - push_air_vacuum_pump
      specs:
          []
    - name: "Laser 1"
      type: "laser"
      id: 32
      available_commands:
          - activate_digital_io
          - deactivate_digital_io
          - setup_digital_io
      specs:
          []
    - name: "DC Motor 1"
      type: "dc_motor"
      id: 33
      available_commands:
          - activate_digital_io
          - deactivate_digital_io
          - setup_digital_io
      specs:
          []
```

## 可选接口（一）

- gauss/tool_action  控制末端执行器执行动作
- gauss/change_tool  改变末端的工具
- /gauss/current_tool_id  

## 可选接口（二）

注意：该接口为最底层的接口，一般只用于调试。
- gauss/tools/open_gripper
- gauss/tools/close_gripper