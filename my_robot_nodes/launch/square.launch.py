import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    diffdrive_share = get_package_share_directory("diffdrive_bot")

    side_length = ParameterValue(LaunchConfiguration("side_length"), value_type=float)

    # Includes diffdrive_bot control.launch.py (which includes Gazebo simulation,
    # obstacle_detector, drive_controller, and drive_distance_server)
    control_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(diffdrive_share, "launch", "control.launch.py")
        ),
    )

    rotate_server = Node(
        package="my_robot_nodes",
        executable="rotate_server",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    square_driver = Node(
        package="my_robot_nodes",
        executable="square_driver",
        parameters=[{"side_length": side_length, "use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "side_length",
            default_value="0.5",
            description="Side length of the square trajectory in meters",
        ),
        control_simulation,
        rotate_server,
        square_driver,
    ])

