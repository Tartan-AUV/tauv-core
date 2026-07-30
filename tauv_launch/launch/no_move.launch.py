import os
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            SetEnvironmentVariable, TimerAction, IncludeLaunchDescription)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    xsens_params_path = os.path.join(
        get_package_share_directory('xsens_mti_ros2_driver'),
        'param',
        'xsens_main.yaml'
    )
    dronecan_db_path = os.path.join(
        get_package_share_directory('tauv_dronecan'),
        'dronecan_dna.db'
    )
    common_share_dir = Path(get_package_share_directory("tauv_state_estimation"))
    common_ekf_file = common_share_dir / "config" / "ekfFUNNY.yaml"
    
    timestamp = datetime.now().strftime('%Y.%m.%d_%H.%M.%S')
    bag_path = Path("/tauv-mono/ros_ws/bags") / f"rosbag_osprey_{timestamp}"

    servo_tasks_path = os.path.join(
        get_package_share_directory('tauv_servo'),
        'config',
        'servo_tasks.yaml'
    )

    watchdog_params = {
        'esc_topic': '/esc_telemetry',
        'imu_topic': 'os/sensors/imu_xsens',
        'system_state_topic': 'watchdog/system_state',
        'heartbeat_frequency_hz': 1.0,
        'mission_timeout_s':480,
        'esc_timeout_s': 1.0,
        'stale_startup_grace_s': 5.0,
        'warning_temperature_c': 70.0,
        'error_temperature_c': 90.0,
        'error_voltage_v': 12.0,
        'roll_threshold_deg': 35.0,
        'pitch_threshold_deg': 35.0,
        'angular_velocity_threshold_radps': 3.0,
        'expected_esc_ids': [100, 101, 102, 103, 104, 105, 106, 107],
    }

    # --- Launch Description ---
    return LaunchDescription([
        DeclareLaunchArgument(
            'tune', 
            default_value='False', 
            description='Enable autotuning for the controller'
        ),

        # Global Logging Settings for Jetson Orin Performance
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # Container for FOG and EKF nodes to enable intra-process communication
        ComposableNodeContainer(
            name='sensor_fusion_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='tauv_kvh',
                    plugin='tauv_kvh::KvhNode',
                    name='kvh_node',
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ],
            output='screen',
        ),

        Node(
            package='tauv_depth',
            executable='depth',
            name='depth_sensor',
            output='screen',
            parameters=[{'i2c_bus': 7}]
        ),

        Node(
            package='tauv_servo',
            executable='servo_driver',
            name='servo',
            output='screen',
            parameters=[{
                'interface': 'can0',
                'bitrate': 1000000,
                'tasks_config': servo_tasks_path,
                'command_topic': '/servo/task',
                'status_topic': '/mission/status',
                'scan_max_id': 4,
                'torque_limit': 25.0,
                'position_limit_deg': 150.0,
                'angle_tolerance': 3.0,
                'verify_timeout': 3.0,
                'verify_poll': 0.1,
                'telem_rate_hz': 1.0,
                'telem_topic_prefix': '/servo/telem',
                'startup_task': '',
            }]
        ),
        # Node(
        #     package='tauv_dronecan',
        #     executable='can_driver',
        #     name='dronecan',
        #     output='screen',
        #     parameters=[{
        #         'interface': 'can1',
        #         'node_id': 12,
        #         'bitrate': 1000000,
        #         'esc_count': 8,
        #         'command_rate_hz': 100.0,
        #         'discovery_time_sec': 15.0,
        #         'dna_db_path': dronecan_db_path,
        #         'BIGARM':True
        #     }]
        # ),
        Node(
            package='xsens_mti_ros2_driver',
            executable='xsens_mti_node',
            name='xsens_mti_node',
            output='screen',
            parameters=[xsens_params_path]
        ),
        Node(
            package='dvl_a50',
            executable='dvl_a50_sensor', 
            name='dvl_a50',
            output='screen',
            parameters=[{'dvl_ip_address': '192.168.8.114',
                         'acoustic_enabled': False}]
        ),

        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{'port': 8765, 'address': '0.0.0.0'}]
        ),
        # ExecuteProcess(            
        #     cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(bag_path), '--all', '--exclude', '|'.join([
        #         '^/oak/rgb/image_raw$',
        #         '^/cloud_map$',
        #         '^/grid_map$',
        #         '^/grid_prob_map$',
        #         '^/mapData$',
        #         '^/mapGraph$',
        #     ])],
        #     output='screen',
        # ),
        
        # Node(
        #     package="tauv_watchdogs",
        #     executable="watchdog",
        #     name="watchdog",
        #     output="screen",
        #     parameters=[watchdog_params]
        # ),

        # Node(package="tauv_repackagers", executable="imu_converter", name="imu_converter", output="screen"),
        # Node(package="tauv_repackagers", executable="depth_converter", name="depth_converter", output="screen"),
        Node(package="tauv_repackagers", executable="dvl_converter", name="dvl_converter", output="screen"),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_xsens',
            arguments=['-0.1111', '0.0167',  '0.0469', '3.14159', '0', '0', 'os/base_link', 'imu_link_xsens'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_depth',
            arguments=['-0.1999', '-0.0635', '0.0803', '0', '0', '0', 'os/base_link', 'depth_link'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_dvl',
            arguments=['-0.1408', '0.0000', '0.0100', '-1.5708', '0.0', '3.14159', 'os/base_link', 'dvl_link'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_fog_gyro',
            arguments=['0.06184000', '0.1', '0.06922651',
                    '0.69843', '0.02752', '0.71476', '-0.02357',
                    'os/base_link', 'imu_link_fog_gyro'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_fog_accel_x',
            arguments=['0.09534000', '0.12620000', '0.06972651',
                    '0.69843', '0.02752', '0.71476', '-0.02357',
                    'os/base_link', 'imu_link_fog_accel_x'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_fog_accel_y',
            arguments=['0.09894000', '0.11169440', '0.04362071',
                    '0.69843', '0.02752', '0.71476', '-0.02357',
                    'os/base_link', 'imu_link_fog_accel_y'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_fog_accel_z',
            arguments=['0.10684000', '0.10049400', '0.04352071',
                    '0.69843', '0.02752', '0.71476', '-0.02357',
                    'os/base_link', 'imu_link_fog_accel_z'],
            output='screen'
        ),

        Node(
            package='tauv_controller',
            executable='controller',
            name='controller',
            parameters=[{'tune': LaunchConfiguration('tune')}],
            output='screen',
        ),
        Node(package='tauv_controller', executable='thruster_forces', name='thruster_forces', output='screen'),
        Node(package='tauv_controller', executable='thruster_rpms', name='thruster_rpms', output='screen'),

        Node(
            package='tauv_trajectory',
            executable='trajectory_planner',
            name='trajectory_planner',
            output='screen',
        ),

        # Node(
        #     package='tauv_mission',
        #     executable='mission_planner',
        #     name='mission_planner',
        #     parameters=[{'mission_file': 'example_mission.json'}],
        #     output='screen',
        # ),

        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg="Loading EKF component into container!!!!!!"),
                LoadComposableNodes(
                    target_container='sensor_fusion_container',
                    composable_node_descriptions=[
                        ComposableNode(
                            package="robot_localization",
                            plugin="robot_localization::EkfComponent",
                            name="ekf_filter_node",
                            parameters=[str(common_ekf_file)],
                            extra_arguments=[{'use_intra_process_comms': True}]
                        )
                    ]
                )
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('tauv_vision'), 'launch', 'driver_launcher.launch.py'
            ))
        )
    ])
