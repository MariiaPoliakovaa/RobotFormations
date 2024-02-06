import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from typing import List
from pathlib import Path


# TODO: relative paths
custom_models_path = "/home/mariia/Documents/Masterarbeit/Implementation_example/gazebo_implementations/src/multi_turtlebot_gazebo/models"
world_path = '/home/mariia/Documents/Masterarbeit/Implementation_example/gazebo_implementations/src/multi_turtlebot_gazebo/worlds/empty.world'


def generate_launch_description():

    ld = LaunchDescription()

    TURTLEBOT3_MODEL = "burger"

    enable_drive = LaunchConfiguration("enable_drive", default="true")
    declare_enable_drive = DeclareLaunchArgument(
        name="enable_drive", default_value="true", description="Enable robot drive node"
    )

    multi_turtlebot_gazebo = get_package_share_directory("multi_turtlebot_gazebo")
    launch_file_dir = os.path.join(multi_turtlebot_gazebo, "launch")

    world = os.path.join(
        multi_turtlebot_gazebo, "worlds", "empty.world"
    )

    urdf_file_name = "turtlebot3_" + TURTLEBOT3_MODEL + ".urdf"
    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(
        multi_turtlebot_gazebo, "urdf", urdf_file_name
    )

    gazebo = Node(
        package='gazebo_ros',
        executable='/usr/bin/gazebo',
        name='gazebo',
        output='screen',
        arguments=[
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            '--verbose',
            world_path
        ],
    )

    ld.add_action(declare_enable_drive)
    ld.add_action(gazebo)

    last_action = None

    # Remapping is required for state publisher otherwise /tf and /tf_static will get be published on root '/' namespace
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Spawn turtlebot3 instances in gazebo
    robots = ['Tony', 'Jacky']
    for i, robot in enumerate(robots):
        # Construct a unique name and namespace
        name = robot
        namespace = robot

        # Create state publisher node for that instance
        turtlebot_state_publisher = Node(
            package="robot_state_publisher",
            namespace=namespace,
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"use_sim_time": False,
                         "publish_frequency": 10.0}],
            remappings=remappings,
            arguments=[urdf],
        )

        # Create spawn call
        spawn_turtlebot3_burger = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-file",
                os.path.join(multi_turtlebot_gazebo,'models', 'turtlebot3_' + TURTLEBOT3_MODEL, 'model.sdf'),
                "-entity",
                name,
                "-robot_namespace",
                namespace,
                "-x",
                str(i),
                "-y",
                str(i),
                "-z",
                "0.01",
                "-Y",
                "3.14159",
                "-unpause",
            ],
            output="screen",
        )

        if last_action is None:
            # Call add_action directly for the first robot to facilitate chain instantiation via RegisterEventHandler
            ld.add_action(turtlebot_state_publisher)
            ld.add_action(spawn_turtlebot3_burger)

        else:
            # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
            # Simply calling ld.add_action for spawn_entity introduces issues due to parallel run.
            spawn_turtlebot3_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[spawn_turtlebot3_burger,
                             turtlebot_state_publisher],
                )
            )
            ld.add_action(spawn_turtlebot3_event)

        # Save last instance for next RegisterEventHandler
        last_action = spawn_turtlebot3_burger

    return ld
