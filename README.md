# ROS2 Publisher-Subscriber Package

## Overview

This package implements a basic publisher-subscriber system in ROS2 with added functionality:

A publisher node that sends customizable string messages at a configurable frequency
A subscriber node that receives and validates these messages
A service to dynamically modify the publisher's message content
Comprehensive error handling and logging throughout
Launch file support with configurable publishing frequency

## Dependencies

ROS 2 Humble
colcon
example_interfaces package

## Installation
1. Create a ROS2 Workspace (if not already created)

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```


2. Clone the Package

```sh

git clone <repository-url> beginner_tutorials
cd ..
```

3. Build the Package

```sh

colcon build --packages-select beginner_tutorials
```
4. Source the Setup Files

```sh

source install/setup.bash

```
## Usage
## Running the Nodes Individually

    Start the Publisher Node:

```sh

ros2 run beginner_tutorials talker_2

```
    Start the Subscriber Node (in a new terminal):

```sh

ros2 run beginner_tutorials listener_2
```


## Using the Launch File

Run both nodes with default frequency (2.0 Hz):

```sh
ros2 launch beginner_tutorials talker_listener.launch.py
```


Run with custom publishing frequency (e.g., 5.0 Hz):

```sh

ros2 launch beginner_tutorials talker_listener.launch.py frequency:=5.0
```


## Using the Service

Change the message text:

```sh

# To set extended message
ros2 service call /change_string example_interfaces/srv/SetBool "data: true"
```

```sh
# To reset to default message
ros2 service call /change_string example_interfaces/srv/SetBool "data: false"
```

## Features
## Publisher Node

    Configurable publishing frequency via parameter
    Dynamic message modification through service
    Error handling for invalid frequency values
    Message counter with high count warnings
    Comprehensive logging at multiple levels

## Subscriber Node

    Message validation (empty message checks)
    Length validation (warns for messages > 256 characters)
    Error handling for message processing
    Detailed logging of received messages

## Error Handling

The package includes comprehensive error handling:

    Validation of publishing frequency
    Service request validation
    Message content validation
    Exception handling for node initialization
    Logging at DEBUG, INFO, WARN, ERROR, and FATAL levels

## License

This project is licensed under the Apache License 2.0 - see the file headers for details.
