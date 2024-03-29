#!/bin/bash

# Check if an argument is provided
if [ $# -eq 0 ]; then
    echo "Usage: Provide a number of robots"
    exit 1
fi

source install/setup.sh
source setup.sh $1

ros2 run target_assignement_controller target_assignement
