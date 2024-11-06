# ROS2 Simple Publisher-Subscriber Demo

## Overview
This package demonstrates a basic ROS2 publisher-subscriber implementation where:
- A publisher node broadcasts string messages with an incrementing counter
- A subscriber node listens for these messages and displays them
- Both nodes use ROS2's logging system for output
- The code follows ROS2 coding standards and includes comprehensive Doxygen documentation

## Dependencies
- ROS 2 Humble
- Ubuntu 22.04
- C++14 or later
- CMake 3.8 or later
- colcon
- Standard ROS2 development tools (rclcpp, std_msgs)

## Building the Package

1. Create a ROS2 workspace (if not already created):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone this package into the src directory

3. Build the package with compile commands generation:
```bash
cd ~/ros2_ws
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```

4. Source the required setup files:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

## Running the Nodes

1. In terminal 1, run the publisher node:
```bash
ros2 run beginner_tutorials talker
```

2. In terminal 2, run the subscriber node:
```bash
ros2 run beginner_tutorials listener
```

Note: Make sure you've sourced the setup files in each new terminal:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

## Code Structure
- `publisher_member_function.cpp`: Implements the publisher node
- `subscriber_member_function.cpp`: Implements the subscriber node
- Both files include comprehensive Doxygen documentation and follow ROS2 coding standards

## Additional Features
- Doxygen documentation for all major components
- Proper ROS2 logging implementation
- Standard ROS2 message types (std_msgs/String)
- Apache 2.0 license compliance

## License
This package is licensed under the Apache License 2.0.

## Author
Navdeep Singh

## Additional Notes
- The publisher publishes messages every 500ms by default
- The subscriber uses ROS2's logging system to display received messages
- Both nodes can be terminated using Ctrl+C
- The code includes proper error handling and shutdown procedures

For any issues or questions, please refer to the ROS2 documentation or open an issue in the repository.


The main changes made:
- Changed `cpp_pubsub` to `beginner_tutorials` in the ros2 run commands
- All other content remains the same as it was package-name independent
