import shutil
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnShutdown, OnProcessExit
from launch_ros.actions import Node
from launch.events import Shutdown


def generate_launch_description():
    common_share_dir = Path(get_package_share_directory("tauv_core"))
    common_ekf_file = common_share_dir / "config" / "ekfFUNNY.yaml"

    timestamp = "2026.02.24_16.06.11"
    ssd_rosbag = f"/mnt/peely/rosbags/rosbag_osprey_{timestamp}"

    # Timestamped bag name
    bag_name = f"rosbag_replay_{timestamp}"
    bag_name_latest = "rosbag_replay_latest"

    local_ekf_record_file = (
        Path("/tauv-mono/ros_ws/bags") / bag_name
    )
    local_ekf_record_file_latest = (
        Path("/tauv-mono/ros_ws/bags") / bag_name_latest
    )
    ssd_ekf_record_file = Path(f"/mnt/peely/rosbags/rosbag_ekf_{timestamp}")

    # --- 1. OVERWRITE FIX: Delete target directories at launch if they exist ---
    # ros2 bag record will fail if the directory already exists. 
    for bag_path in [local_ekf_record_file, ssd_ekf_record_file]:
        if bag_path.exists() and bag_path.is_dir():
            print(f"Removing existing bag to overwrite: {bag_path}")
            shutil.rmtree(bag_path)

    print(f"Recording local EKF data to: {local_ekf_record_file}")
    print(f"Recording SSD EKF data to: {ssd_ekf_record_file}")

    # --- 2. POST-RUN SCRIPT: Handle "latest" copy on shutdown ---
    # When you stop the launch file (Ctrl+C), this event handler triggers the copy.
    post_run_latest_script = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    condition=IfCondition(LaunchConfiguration('saveLocal')),
                    cmd=['rm', '-rf', str(local_ekf_record_file_latest)],
                    output='log'
                ),
                ExecuteProcess(
                    condition=IfCondition(LaunchConfiguration('saveLocal')),
                    # cp -r is for smaller bags (recursive copy), ln -sfn is for larger bags (symlink update)
                    cmd=['ln', '-sfn', str(local_ekf_record_file), str(local_ekf_record_file_latest)],
                    output='log'
                )
            ]
        )
    )

    play_bag_cmd = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '-r', '1', '--start-offset', '120', ssd_rosbag],
        output='screen',
    )
    auto_shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=play_bag_cmd,
            on_exit=[
                EmitEvent(event=Shutdown(reason='Rosbag playback finished'))
            ]
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'saveLocal', default_value='true', description='Record locally'
            ),
            DeclareLaunchArgument(
                'saveSSD', default_value='true', description='Record on SSD'
            ),
            play_bag_cmd,
            auto_shutdown_handler,
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                parameters=[str(common_ekf_file), {'use_sim_time': True}],
                output="screen",
            ),
            Node(
                package="tauv_core",
                executable="imu_converter",
                name="imu_converter",
                output="screen",
            ),
            Node(
                package="tauv_core",
                executable="depth_converter",
                name="depth_converter",
                output="screen",
            ),
            Node(
                package="tauv_core",
                executable="dvl_converter",
                name="dvl_converter",
                output="screen",
            ),
            ExecuteProcess(
                condition=IfCondition(LaunchConfiguration('saveLocal')),
                cmd=[
                    'ros2', 'bag', 'record',
                    '-a',
                    '-s', 'mcap',
                    '--use-sim-time',
                    '-o',
                    str(local_ekf_record_file),
                ],
                output='screen',
            ),
            ExecuteProcess(
                condition=IfCondition(LaunchConfiguration('saveSSD')),
                cmd=[
                    'ros2', 'bag', 'record',
                    '-a',
                    '-s', 'mcap',
                    '--use-sim-time',
                    '-o',
                    str(ssd_ekf_record_file),
                ],
                output='screen',
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_imu',
                arguments=['0', '0', '0', '0', '0', '0', 'os/base_link', 'imu_link_xsens'],
                parameters=[{'use_sim_time': True}],
                output='screen'
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_depth',
                arguments=['0', '0', '0', '0', '0', '0', 'os/base_link', 'depth_link'],
                parameters=[{'use_sim_time': True}],
                output='screen'
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_dvl',
                arguments=['0', '0', '0', '1.5708', '0', '-3.14159', 'os/base_link', 'dvl_link'],
                parameters=[{'use_sim_time': True}],
                output='screen'
            ),
            post_run_latest_script
        ]
    )