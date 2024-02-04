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


"""
def generate_robots(n: int) -> List[Node]:
    # Remapping is required for state publisher otherwise /tf and /tf_static will get be published on root '/' namespace
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    for i in range(n):
        name = 'turtlebot_' + str(i)

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

        return robots

def generate_launch_description():
    ld = LaunchDescription([
        # Set logging level
        DeclareLaunchArgument(
            'log_level',
            default_value='debug',
            description='Set the ROS log level (debug, info, warn, error, fatal)'
        ),

        # Set path to models
        DeclareLaunchArgument(
            'model_path',
            default_value=custom_models_path,
            description='Path to the directory containing the TurtleBot3 model'
        ),

        # Launch Gazebo
        Node(
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
        ),
    ])
    robots = generate_robots(1)
    ld.add_action(*robots)

    return ld
"""



def generate_launch_description():
    num_robots = 2

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

    """
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")
        ),
    )
    """
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
    #ld.add_action(gzserver_cmd)
    #ld.add_action(gzclient_cmd)
    ld.add_action(gazebo)

    last_action = None

    # Remapping is required for state publisher otherwise /tf and /tf_static will get be published on root '/' namespace
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Spawn turtlebot3 instances in gazebo
    for i in range(num_robots):
        # Construct a unique name and namespace
        name = "turtlebot" + str(i)
        namespace = "/tb" + str(i)

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
