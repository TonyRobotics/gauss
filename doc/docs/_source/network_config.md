# 网络配置

一般情况下，直接通过本机ssh 登录到机械臂的树莓派主控板卡，然后调试和修改代码。另外，为了调试方便，也可以通过PC机和机械臂通信。

如需要PC机器和机械臂通信，则需要配置PC机和机械臂通信。

假设机械臂的 ip 为：192.168.0.116， PC机的 ip 为: 192.168.0.120

那么需要把机械臂的所有节点都 kill 掉，然后重新启动ros：

1. 在机械臂上操作

`rosnode kill -a`

修改 ~/.bashrc，修改环境变量：

```
export ROS_MASTER_URI=http://192.168.0.116:11311
export ROS_HOSTNAME=192.168.0.116
```
然后执行： `source ~/.bashrc`

最后在机械臂上启动ros： `gauss-ros-start`

2. 在PC机上操作

```
export ROS_MASTER_URI=http://192.168.0.116:11311
export ROS_HOSTNAME=192.168.0.120
```

在 PC 机上测试：
```
rostopic echo /gauss/robot_state
```
