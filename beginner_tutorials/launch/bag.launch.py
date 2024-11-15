from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent, LogInfo, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from datetime import datetime
import os

def generate_launch_description():
    # Get the current timestamp for the bag file name
    timestamp = datetime.now().strftime('%Y_%m_%d-%H_%M_%S')
    
    # Define the bag file path
    bag_path = os.path.join(
        os.getcwd(),
        'src',
        'my_beginner_tutorials',
        'results',
        f'rosbag2_{timestamp}'
    )
    
    # Create directory if it doesn't exist
    os.makedirs(os.path.dirname(bag_path), exist_ok=True)
    
    # Log the recording location
    start_log = LogInfo(
        msg=f'Starting recording to: {bag_path}'
    )
    
    # Use timeout command to limit recording to 15 seconds
    record_cmd = [
        'timeout', '--preserve-status', '15',
        'ros2', 'bag', 'record',
        '-a',  # Record all topics
        '--compression-mode', 'file',  # Enable compression
        '--compression-format', 'zstd',  # Use zstd compression
        '--storage', 'sqlite3',  # Use SQLite storage
        '--output', bag_path
    ]
    
    # Declare launch argument for enabling/disabling recording
    declare_record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='True',
        description='Flag to enable or disable ROS bag recording'
    )
    
    # Create the recording process
    recorder = ExecuteProcess(
        cmd=record_cmd,
        output='screen',
        name='rosbag2_recorder',
        condition=IfCondition(LaunchConfiguration('record_bag'))
    )
    
    # Create completion log
    complete_log = LogInfo(
        msg=f'Recording completed. Bag file saved to: {bag_path}'
    )
    
    # Create shutdown event
    shutdown_event = EmitEvent(
        event=Shutdown(
            reason='Recording completed successfully'
        )
    )
    
    # Register an event handler for clean shutdown
    exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=recorder,
            on_exit=[complete_log, shutdown_event]
        )
    )
    
    return LaunchDescription([
        declare_record_bag_arg,
        start_log,
        recorder,
        exit_handler
    ])
