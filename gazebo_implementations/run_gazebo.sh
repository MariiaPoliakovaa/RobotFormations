#!/bin/bash

# Check if an argument is provided
if [ $# -eq 0 ]; then
    echo "Usage: Provide a number of robots"
    exit 1
fi

source install/setup.bash
source setup.sh 4

ros2 launch multi_turtlebot_gazebo multi_turtlebot_gazebo.launch.py
