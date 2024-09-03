# 功能介绍

该功能包通过接收物体识别节点的消息，控制机器人自动踢球

# 使用方法

## 准备工作

1. 具备TonyPi机器人，包含机器人本体、相机及RDK套件，并且能够正常运行。
2. 具备小球等相关道具

## 编译与运行

**1.编译**

启动机器人后，通过终端SSH或者VNC连接机器人，打开终端拉取相应代码并编译安装

```bash
# 拉取人形机器人SDK并安装
mkdir -p /home/pi && cd /home/pi

# RDK X5
git clone https://github.com/wunuo1/TonyPi.git -b feature-humble-x5
# RDK X3
git clone https://github.com/wunuo1/TonyPi.git -b feature-foxy-x3

cd /home/pi/TonyPi/HiwonderSDK
pip install .


# 拉取踢球控制代码并编译
mkdir -p ~/tonypi_ws/src && cd ~/tonypi_ws/src

# RDK X5
git clone https://github.com/wunuo1/tonypi_kickball_control.git -b feature-humble-x5
# RDK X3
git clone https://github.com/wunuo1/tonypi_kickball_control.git -b feature-foxy-x3

cd ..
source /opt/tros/setup.bash
colcon build --packages-select tonypi_kickball_control
```

**2.运行自动踢球功能**

```shell
source ~/tonypi_ws/install/setup.bash

ros2 launch tonypi_kickball_control kick_ball_control.launch.py

```


# 接口说明

## 话题

### Sub话题
| 名称                          | 消息类型                                                     | 说明                                                   |
| ----------------------------- | ------------------------------------------------------------ | ------------------------------------------------------ |
| robot_target_detection      | ai_msgs::msg::PerceptionTargets        | 目标物体的位置消息                  |

## 参数

| 参数名                | 类型        | 说明   |
| --------------------- | ----------- | ----------------------------------------------------- |
| sub_topic    | string |    接收的目标物体的消息名称，请根据实际发布的话题名称配置，默认值为/robot_target_detection |
| target_type    | string |    识别目标的类型，默认值为red_ball |