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

## Bounded lateral motion

The deployed entry point is `app.py`. Its existing `POST /motion` contract now
accepts optional `linear_y` (m/s, default `0.0`) alongside `linear_x`, `angular_z`,
`duration`, `streaming`, and `watchdog_timeout`. A nonzero lateral request must
have zero forward and yaw axes and is limited to `abs(linear_y) <= 0.10`.
Existing forward/yaw limits, duration bounds, streaming cadence and deadman
semantics remain unchanged. Initial Cognitive avoidance requests use only
`linear_y = +/-0.08` for at most `0.50` seconds.

The same validated endpoint publishes `Twist.linear.y` through the existing ROS
publisher. Bounded requests automatically STOP; explicit STOP and streaming
watchdog expiration publish zero x/y/yaw and clear all motion state. `/status`
exposes `motion.linear_y` and `motion_capabilities` so a client can reject a
lateral request before contacting a Bridge without lateral support. Omitted
`linear_y` preserves older clients' motion requests.
