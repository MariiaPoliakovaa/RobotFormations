#!/bin/bash

# Read the json from the file
robots=$(<robots.json)

# Export the string to an environment variable
export ROBOTS="$robots"
