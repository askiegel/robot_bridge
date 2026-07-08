# Mini Pupper Robot Bridge

Robot-side bridge server for the Mini Pupper 2 Cognitive Robotics Platform.

This service runs on the Mini Pupper and exposes a small HTTP API for the Ubuntu PC cognitive platform.

## Purpose

Keep ROS2 isolated on the robot.

The Ubuntu PC should not depend on cross-machine ROS2 discovery.

## Confirmed Robot Runtime

Workspace:

~/ros2_ws

Bringup:

ros2 launch mini_pupper_bringup bringup.launch.py

Motion topic:

/cmd_vel

Message type:

geometry_msgs/msg/Twist

Controller:

/quadruped_controller_node

## Initial API

GET  /status
POST /motion
POST /stop

## Development Rule

One feature per commit.
