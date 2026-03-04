from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    # 1. Hardcoded paths for your Osprey bag
    input_bag = '/mnt/peely/rosbags/rosbag_osprey_2026.02.24_16.06.11'
    output_bag = '/mnt/peely/rosbags/rosbag_osprey_2026.02.24_16.06.11_no_tf'

    # 2. Start the recording process using simulated time
    record_process = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', 
            '-a', 
            '-x', '^/tf$', 
            '-s', 'mcap',
            '-o', output_bag,
            '--use-sim-time'  # <-- Forces recorder to use the bag's timestamps
        ],
        output='screen'
    )

    # 3. Start the playback process with a clock publisher and a slight delay
    play_process = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', input_bag, 
            # '-r', '10.0',     # Play at 10x speed
            '--clock',        # <-- Broadcasts the bag's time to the system
            '-d', '2'         # Wait 2 seconds before playing so the recorder doesn't miss start
        ],
        output='screen'
    )

    # 4. Clean shutdown when playback finishes
    shutdown_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=play_process,
            on_exit=[
                LogInfo(msg="Playback finished! Shutting down the recorder..."),
                EmitEvent(event=Shutdown())
            ]
        )
    )

    return LaunchDescription([
        record_process,
        play_process,
        shutdown_handler
    ])