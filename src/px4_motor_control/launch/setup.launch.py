import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Include the iris_sitl.launch.py launch file
    iris_sitl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('px4_motor_control'),
                'launch',
                'iris_sitl.launch.py'
            )
        )
    )

    # Define the px4_detector node
    px4_detector_node = Node(
        package='px4_detector',
        executable='detector',
        name='px4_detector'
    )

    return LaunchDescription([
        iris_sitl_launch,
        px4_detector_node
    ])