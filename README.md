# tonypi_kickball_control
# Function Introduction

This package controls the robot to automatically kick the ball by receiving messages from the object recognition node.

# Usage

## Preparations

1. Have the TonyPi robot, including the robot body, camera, and RDK suite, and ensure it runs normally.
2. Have related props such as small balls.

## Compile and Runs

**1. Compile**

After starting the robot, connect to it via SSH or VNC on the terminal, open the terminal, pull the corresponding code, and compile and install it.

```bash
# Pull and install the robot SDK
mkdir -p /home/pi && cd /home/pi
git clone https://github.com/wunuo1/TonyPi.git -b feature-humble-x5
cd /home/pi/TonyPi/HiwonderSDK
pip install .

# Pull the kick control code and compile it
mkdir -p ~/tonypi_ws/src && cd ~/tonypi_ws/src
git clone https://github.com/wunuo1/tonypi_kickball_control.git -b feature-humble-x5
cd ..
source /opt/tros/setup.bash
colcon build --packages-select tonypi_kickball_control
```
**2. Run the Task Decomposition Function**

```shell
source ~/tonypi_ws/install/setup.bash

ros2 launch tonypi_kickball_control kick_ball_control.launch.py
```

# Interface Description

## Topics

### Subscribed Topics

|Name  | Type                                  |  Description           |
|------| --------------------------------------| --------------------------------|
|robot_target_detection |ai_msgs::msg::PerceptionTargets | Position messages of the target object|

## Parameters
| Parameter Name             | Type       | Description  |
| --------------------- | ----------- | ----------------------------------------------------- |
| sub_topic    | string |   Name of the subscribed topic for receiving messages about target objects. Configure according to the actual published topic name. Default value is /robot_target_detection |
| target_type    | string |    Type of the recognized target. Default value is red_ball. |