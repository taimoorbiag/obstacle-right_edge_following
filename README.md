# ROS 2 Fuzzy Wall Following and Obstacle Avoidance

A Python mobile robot controller that combines right-wall following
and obstacle avoidance using laser scan data and fuzzy logic.

## Overview

The controller receives laser measurements from `/scan` and publishes
movement commands to `/cmd_vel`.

Two behaviours calculate the robot's speed and steering:

- **Right-wall following:** uses front-right and back-right distances.
- **Obstacle avoidance:** uses front-right, front, and front-left distances.

Distance thresholds determine which behaviour supplies the final command.

## Features

- Near, medium, and far fuzzy distance categories.
- Nine rules for right-wall following.
- Twenty-seven rule combinations for obstacle avoidance.
- Weighted-average defuzzification.
- Velocity commands published every 0.2 seconds (5 Hz).
- Stop command on keyboard interruption.

## Technologies

- Python 3
- ROS 2
- rclpy
- LaserScan messages
- Twist velocity commands

## Main File

`combination.py` — contains both fuzzy controllers, behaviour selection,
laser processing, and velocity publishing.

## Main Functions

| Function | Purpose |
|----------|---------|
| `right_edge()` | Calculates wall-following velocity |
| `obstacle_avoid()` | Calculates obstacle-avoidance velocity |
| `clbk_laser()` | Extracts distances from laser scan sectors |
| `find_nearest()` | Finds the nearest positive distance, capped at 1 metre |
| `movement()` | Selects the behaviour and creates a velocity command |
| `timer_callback()` | Publishes the latest command |
| `stop()` | Publishes zero velocity |

## ROS Topics

| Topic | Message Type | Direction |
|-------|--------------|-----------|
| `/scan` | `sensor_msgs/msg/LaserScan` | Input |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Output |

## Behaviour Selection

The current implementation uses the following selection order:

1. Select wall following when the minimum front-right/back-right
   distance is greater than 0.6 m and at most 0.7 m.
2. Otherwise, select obstacle avoidance when the minimum
   front-right/front/front-left distance is at most 0.6 m.
3. Otherwise, default to wall following.

Although the code contains a weighted combination formula, its binary
weights and `if/elif` selection activate only one behaviour at a time.

## Requirements

- A configured ROS 2 environment.
- Python 3.
- Packages providing `rclpy`, `sensor_msgs`, `geometry_msgs`,
  `nav_msgs`, and `tf2_ros`.
- A robot or simulation publishing `/scan` and accepting `/cmd_vel`.

## Running

Start the robot or simulation first.

In a terminal with the ROS 2 environment sourced, open the directory
containing the script and run:

```bash
python3 combination.py
```

Press `Ctrl+C` to stop.
