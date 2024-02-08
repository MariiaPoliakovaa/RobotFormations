# Masterarbeit

Source ROS2
	source /opt/ros/humble/setup.bash
Source gazebo
	source /usr/share/gazebo/setup.sh

## build a package
I one terminal:
colcon build --packages-select multi_turtlebot_gazebo

## Run a package
In another terminal than the one the package was built in:
ros2 launch multi_turtlebot_gazebo multi_turtlebot_gazebo.launch.py


## Ideen
- target assignement: Statt die Zuweisung und Prio nur Anhand vom Abstand zum naechsten Ziel zu berechnen koennte man noch die notwendige Rotation mit hinein berechnen, da diese oft sehr lange braucht
