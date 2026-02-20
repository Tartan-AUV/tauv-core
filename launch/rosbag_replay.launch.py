from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    common_share_dir = Path(get_package_share_directory("tauv_core"))
    common_ekf_file = common_share_dir / "config" / "ekf.yaml"

    ssd_rosbag = "/mnt/peely/rosbags/rosbag_osprey_2026.02.13_16.58.47"

    # Timestamped bag name
    timestamp = "2026.02.13_16.58.47"
    bag_name = f"rosbag_replay_{timestamp}"
    bag_name_latest = "latest"

    local_ekf_record_file = (
        Path("src") / "tauv_core" / "odometry_visualization" / "rosbags" / bag_name
    )
    local_ekf_record_file_latest = (
        Path("src") / "tauv_core" / "odometry_visualization" / "rosbags" / bag_name_latest
    )
    print(f"Recording local EKF data to: {local_ekf_record_file}")

    ssd_ekf_record_file = f"/mnt/peely/rosbags/rosbag_ekf_{timestamp}"
    print(f"Recording SSD EKF data to: {ssd_ekf_record_file}")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'saveLocal', default_value='true', description='Record locally'
            ),
            DeclareLaunchArgument(
                'saveSSD', default_value='true', description='Record on SSD'
            ),
            ExecuteProcess(
                cmd=['ros2', 'bag', 'play', ssd_rosbag, '--clock'],
                output='screen',
            ),
            Node(
                package="tauv_core",
                executable="dvl_converter",
                name="dvl_converter",
                output="screen",
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                parameters=[str(common_ekf_file)],
                output="screen",
            ),
            ExecuteProcess(
                condition=IfCondition(LaunchConfiguration('saveLocal')),
                cmd=[
                    'ros2',
                    'bag',
                    'record',
                    '/odometry/filtered',
                    '-o',
                    str(local_ekf_record_file),
                ],
                output='screen',
            ),
            ExecuteProcess(
                condition=IfCondition(LaunchConfiguration('saveLocal')),
                cmd=[
                    'ros2',
                    'bag',
                    'record',
                    '/odometry/filtered',
                    '-o',
                    str(local_ekf_record_file_latest),
                ],
                output='screen',
            ),
            ExecuteProcess(
                condition=IfCondition(LaunchConfiguration('saveSSD')),
                cmd=[
                    'ros2',
                    'bag',
                    'record',
                    '/odometry/filtered',
                    '-o',
                    str(ssd_ekf_record_file),
                ],
                output='screen',
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_imu',
                # Arguments: x y z yaw pitch roll frame_id child_frame_id
                arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
                output='screen'
            )
        ]
    )
