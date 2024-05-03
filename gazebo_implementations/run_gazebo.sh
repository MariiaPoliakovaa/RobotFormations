#!/bin/bash

# Check if an argument is provided
if [ $# -eq 0 ]; then
    echo "Usage: Provide a number of robots"
    exit 1
fi

# exports for remote ros messages
export ROS_DOMAIN_ID=1

source install/setup.bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
source setup.sh $1

ros2 launch multi_turtlebot_gazebo multi_turtlebot_gazebo.launch.py
