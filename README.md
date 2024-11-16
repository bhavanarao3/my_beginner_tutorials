# my_beginner_tutorials
ENPM700 ROS2 Programming Assignment


This repository contains basic ROS2 tutorials involving the creation and use of publisher and subscriber member functions in C++ with ROS2.

## Programming Assignment 1

### Directory Structure

```
my_beginner_tutorials/
├── src/
│   ├── beginner_tutorials/
│   │   ├── src/
│   │   │   ├── publisher_member_function.cpp  # Publisher node that demonstrates ROS2 C++ publishing
│   │   │   ├── subscriber_member_function.cpp # Subscriber node that demonstrates ROS2 C++ subscription
```

#### `publisher_member_function.cpp`

This file contains a basic ROS2 C++ publisher node that demonstrates the use of a publisher for sending messages.

#### `subscriber_member_function.cpp`

This file contains a basic ROS2 C++ subscriber node that demonstrates the use of a subscriber to receive messages.

### ROS2 Dependencies

- **rclcpp**: ROS Client Library for C++.
- **std_msgs**: Standard message types for ROS, such as strings and integers.
- Other relevant dependencies (for building ROS2 nodes).

Make sure to install the ROS2 Humble version or the appropriate ROS2 distribution on your system and source the ROS2 setup script.

### How to Build and Run

In the root directory of this repository, use the following commands to build the ROS2 package:

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

#### Why Use `--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`?

- **`colcon build`**: This is the standard command for building ROS2 packages.
- **`--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`**: This additional flag ensures that the compilation commands are exported in a `compile_commands.json` file. This file is essential for static analysis tools like `clang-tidy` and `cpplint` to analyze the code properly. Without this file, the tools may not function correctly because they wouldn't have the necessary context about the build environment (compiler flags, include paths, etc.).

#### How to Run the Nodes

After building the package, run the publisher and subscriber nodes using the following commands:

1. **Run the publisher node:**
   ```bash
   ros2 run beginner_tutorials talker
   ```

2. **Run the subscriber node:**
   ```bash
   ros2 run beginner_tutorials listener
   ```

Ensure that the ROS2 environment is properly sourced before running the nodes.

### Static Code Analysis with `clang-tidy` and `cpplint`

We use static analysis tools to ensure code quality and consistency. Below are the commands to run these tools.

#### `clang-tidy` Command

Run `clang-tidy` with the following command to check for potential issues in the code:

```bash
clang-tidy -p ./ --extra-arg=-stdlib=libc++ --extra-arg=-I/usr/include/c++/11 --extra-arg=-I/usr/include/x86_64-linux-gnu/c++/11 $(find . -name "*.cpp" | grep -v "/build/")
```

- **`-p ./`**: Specifies the location of the `compile_commands.json` file.
- **`--extra-arg=-stdlib=libc++`**: Specifies the standard library to be used for analysis (in this case, `libc++`).
- **`--extra-arg=-I/usr/include/c++/11` and `--extra-arg=-I/usr/include/x86_64-linux-gnu/c++/11`**: Adds the necessary include paths for the C++ standard library and architecture-specific directories.
- **`$(find . -name "*.cpp" | grep -v "/build/")`**: Finds all `.cpp` files in the repository, excluding those in the `/build/` directory.

#### `cpplint` Command

Run `cpplint` to check the style of the C++ code using this command:

```bash
cpplint --filter="-legal/copyright" $(find . -name '*.cpp' | grep -v "/build/")
```

- **`--filter="-legal/copyright"`**: Excludes copyright-related checks.
- **`$(find . -name '*.cpp' | grep -v "/build/")`**: Finds all `.cpp` files, excluding the `/build/` directory.

### Additional Notes

- Make sure to have ROS2 dependencies installed and the ROS2 workspace properly set up before building and running the project.
- If you encounter any issues, ensure your system has all required tools (`clang-tidy`, `cpplint`, `colcon`, etc.) installed and that you are working in a properly sourced ROS2 environment.

## Programming Assignment 2

### ROS2 Minimal Publisher with Log Levels and Dynamic Conditions

This package provides a simple ROS2 node `minimal_publisher` that publishes messages containing the current time to a topic `custom_topic`. It also provides a service to toggle the base output string ("Current time" or "Custom time"). The node includes functionality to dynamically adjust logging levels and simulate different conditions (warnings, errors, and fatal errors) based on parameters provided at runtime.

### Features

- **Publish Frequency**: Controls how often the node publishes messages to `custom_topic`.
- **Log Levels**: Allows dynamic control of the log level to simulate different conditions and verbosity.
  - **DEBUG**: Detailed information for debugging.
  - **INFO**: General information about node operations.
  - **WARN**: Warnings about potential issues, like no subscribers.
  - **ERROR**: Errors, such as a publish frequency below a specified threshold.
  - **FATAL**: Critical issues that lead to node shutdown, such as an excessively low publish frequency.
- **Service to Toggle Base Output String**: A service (`toggle_base_output`) to change the base string used for publishing messages ("Current time" vs "Custom time").

### Dependencies

- ROS2 (any supported distribution like Humble, Foxy, or Galactic)
- `example_interfaces` package for `SetBool` service
- `std_msgs` for `String` message type

### Parameters

The following parameters can be passed at runtime to configure the behavior of the node:

- `publish_frequency` (int, default: 500): The frequency at which the node publishes messages to the topic in milliseconds.
- `frequency_threshold` (int, default: 200): A threshold below which an error will be logged if the `publish_frequency` is too low.
- `log_level` (int, default: 1): The log level to control verbosity:
  - `0`: DEBUG
  - `1`: INFO
  - `2`: WARN
  - `3`: ERROR
  - `4`: FATAL


### Cloning and Building the Package

To install this package in an existing ROS2 colcon workspace, follow these steps:

1. **Clone the Package**:
   ```bash
   cd ~/your_colcon_workspace/src
   git clone https://github.com/yourusername/beginner_tutorials.git

2. **Install Dependencies**:
   Make sure dependencies are installed:
   ```bash
   sudo apt update
   sudo apt install ros-humble-example-interfaces ros-humble-std-msgs
   ```

3. **Build the Workspace**:
   ```bash
   cd ~/your_colcon_workspace
   colcon build --packages-select beginner_tutorials
   ```

4. **Source the Workspace**:
   ```bash
   source ~/your_colcon_workspace/install/setup.bash
   ```


### Running the Node

1. **Launch the Node with Parameters**:
   You can launch the node and pass parameters to adjust its behavior. For example:
   ```bash
   ros2 launch beginner_tutorials minimal_pubsub_launch.py publish_frequency:=500 frequency_threshold:=200 log_level:=1
   ```


### Using the `toggle_base_output` Service

To change the base output string of the publisher using the `toggle_base_output` service:

1. **Call the Service**:
   ```bash
   ros2 service call /toggle_base_output example_interfaces/srv/SetBool "{data: true}"
   ```

   - Setting `data: true` will switch to "Custom time."
   - Setting `data: false` will revert to "Current time."


### Using the Launch File

The launch file (`minimal_pubsub_launch.py`) allows for an organized way to start the node with specified parameters:

1. **Launch with Default Parameters**:
   ```bash
   ros2 launch beginner_tutorials minimal_pubsub_launch.py
   ```

2. **Custom Launch with Parameter Override**:
   You can specify custom values for parameters directly in the launch command:
   ```bash
   ros2 launch beginner_tutorials minimal_pubsub_launch.py publish_frequency:=500 frequency_threshold:=200 log_level:=1
   ```

### Example Usage

1. **Launch the Node with Parameters**

   You can launch the node and control its behavior by passing parameters for publish frequency, frequency threshold, and log level.

   Example command:

   ```bash
   ros2 launch beginner_tutorials minimal_pubsub_launch.py publish_frequency:=500 frequency_threshold:=200 log_level:=1
   ```

## Programming Assignment 3

### ROS2 Talker Node and TF Integration

This project demonstrates the use of a ROS2 Talker node that publishes a TF frame, allowing the listener node to receive and process the transform data. It also includes instructions for running the Talker and Listener nodes, as well as recording and playing back ROS2 bags.

#### Steps

1. Modify the Talker Node to Publish a TF Frame
The Talker node has been modified to publish a TF frame. This frame can be visualized and used for further processing.

To run the modified Talker node:

```bash
ros2 run beginner_tutorials talker
```

2. Check TF Frames with tf2_echo
Once the Talker node is running and publishing the TF frame, you can use the tf2_echo command to listen to the transform.

To view the transform:

```bash
ros2 run tf2_ros tf2_echo <frame_from> <frame_to>
ros2 run tf2_ros tf2_echo world talk
```

3. View Frames with view_frames
To visualize the frames in a graph, you can use the view_frames tool:

```bash
ros2 run tf2_tools view_frames
```
This will generate a PDF file that shows the TF frame tree. The file will be saved in the results/Assignment3 directory.

4. Record the Talker Node with ros_bag_launch.py
You can record the running Talker node using ros_bag_launch.py. This will create a bag file that contains all the messages published by the Talker node.

To record:

```bash
ros2 launch beginner_tutorials ros_bag_launch.py
```

5. Playback the Bag and Run the Listener Node
After recording the bag, use ros2 bag play to play back the recorded messages. You can then run the Listener node to verify if it properly receives the transform data.

To play back the bag file:

```bash
ros2 bag play <path_to_your_bag_file>
```

And run the Listener node:

```bash
ros2 run beginner_tutorials listener
```
This will verify that the listener is correctly receiving and processing the transform data from the Talker node.

Directory Structure
```bash
results/Assignment3
    ├── frames.pdf  # Visualized frames from tf2_tools
    ├── ros2_bag # Folder
    ├── cpp_3.jpg #Cpplint result
    ├── clang_3.jpg #Clangtidy result
```