from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the launch argument
    frequency_arg = DeclareLaunchArgument(
        'frequency',
        default_value='2.0',
        description='Publishing frequency in Hz'
    )

    # Create the talker node with the frequency parameter
    talker_node = Node(
        package='beginner_tutorials',  # Replace with your package name
        executable='talker_2',
        name='talker_2',
        parameters=[{
            'publish_frequency': LaunchConfiguration('frequency')
        }]
    )

    # Create the listener node
    listener_node = Node(
        package='beginner_tutorials',  # Replace with your package name
        executable='listener_2',
        name='listener_2'
    )

    return LaunchDescription([
        frequency_arg,
        talker_node,
        listener_node
    ])