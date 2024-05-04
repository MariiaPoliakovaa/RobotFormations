#!/bin/bash

# Check if an argument is provided
if [ $# -eq 0 ]; then
    echo "Usage: Provide a number of robots"
    exit 1
fi

# Read the json from the file
num_robots=$1
folder="./robots"
file_name="${num_robots}robots.json"
file_path="${folder}/${file_name}"

if [ -f "$file_path" ]; then
	robots=$(<"$file_path")
	# Export the string to an environment variable
	export ROBOTS="$robots"
else
	echo "No json file for ${num_robots} robots"
fi


# export goal positions
patterns_file="${num_robots}robots_patterns.json"
patterns_path="pattern_gen/patterns/${patterns_file}"
target_pos=$(<"$patterns_path")
export TARGET_POSITIONS="$target_pos"
export ROS_DOMAIN_ID=1
