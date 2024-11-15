# ROS2 Publisher-Subscriber Package with TF2 and Testing

## Overview
This ROS2 package demonstrates a publisher-subscriber system that includes:
- Custom string message publishing
- TF2 frame broadcasting
- Service-based message modification
- Integration testing
- ROS bag recording and playback

## Prerequisites
- Ubuntu 22.04
- ROS 2 Humble
- colcon build tools
- Catch2 testing framework

## Building the Package
```bash
# Create a workspace (if not exists)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the package (replace with actual repo URL)
git clone <repository-url>

# Build
cd ~/ros2_ws
colcon build --packages-select beginner_tutorials

# Source the workspace
source install/setup.bash
```

## Running the Nodes
### 1. Run the Publisher Node
```bash
ros2 run beginner_tutorials talker_2
```

### 2. Run the Subscriber Node (in a new terminal)
```bash
ros2 run beginner_tutorials listener_2
```

### 3. Modify Message Content (in a new terminal)
```bash
ros2 service call /change_string example_interfaces/srv/SetBool "data: true"
```

## Inspecting TF Frames
### View Active Transforms
```bash
# Echo transforms between world and talk frames
ros2 run tf2_ros tf2_echo world talk
```

### Generate TF Tree Visualization
```bash
# Generate PDF of TF tree
ros2 run tf2_tools view_frames
```
The generated PDF will show the relationship between the 'world' and 'talk' frames.

## Running Tests
```bash
# Run integration tests
ros2 launch beginner_tutorials integration_test.launch.yaml
```

## Recording and Playing Back Data
### Record Using Launch File
```bash
# Enable recording
ros2 launch beginner_tutorials bag.launch.py record_bag:=True

# Disable recording
ros2 launch beginner_tutorials bag.launch.py record_bag:=False
```

### Inspect Recorded Bag File
```bash
# Replace with your bag's timestamp
ros2 bag info path/to/results/rosbag2_timestamp
```

### Playback Recording
1. Start the listener node:
```bash
ros2 run beginner_tutorials listener_2
```

2. In a new terminal, play the bag file:
```bash
ros2 bag play path/to/results/rosbag2_timestamp
```

## Known Assumptions
1. The TF broadcast is between 'world' (parent) and 'talk' (child) frames
2. ROS bag recordings are stored in the 'results' directory
3. The publish frequency is set to 2Hz by default
4. The bag recording duration is set to 15 seconds
5. The bag files use zstd compression and SQLite3 storage

## Additional Information
- The bag files are stored in: `~/ros2_ws/src/my_beginner_tutorials/results/`
- Default publishing frequency: 2.0 Hz
- TF transform parameters:
  - Translation: (1.0, 2.0, 0.5)
  - Rotation: Variable (around Z-axis)
