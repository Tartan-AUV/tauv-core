import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    xsens_params_path = os.path.join(
        get_package_share_directory('xsens_mti_ros2_driver'),
        'param',
        'xsens_calibrate.yaml'
    )

    return LaunchDescription([
        # Global Logging Settings for Jetson Orin Performance
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # IMU Calibration Node
        Node(
            package='xsens_mti_ros2_driver',
            executable='xsens_mti_node',
            name='xsens_mti_node',
            output='screen',
            parameters=[xsens_params_path]
        ),
    ])