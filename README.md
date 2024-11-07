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

### Functionality

- **Publishing**: The node publishes the current time to `custom_topic` at the specified frequency (`publish_frequency`).
- **Subscriber Check**: If no active subscribers are present, a warning is logged.
- **Publish Frequency Error**: If the `publish_frequency` is lower than the `frequency_threshold`, an error message is logged.
- **Fatal Error**: If the `publish_frequency` is less than 100 ms, a fatal error will occur, and the node will be shut down.
- **Service**: The node provides a service (`toggle_base_output`) to toggle the base output string. You can change it to "Current time" or "Custom time" by calling the service.

### Example Usage

1. **Launch the Node with Parameters**

   You can launch the node and control its behavior by passing parameters for publish frequency, frequency threshold, and log level.

   Example command:

   ```bash
   ros2 launch beginner_tutorials minimal_pubsub_launch.py publish_frequency:=500 frequency_threshold:=200 log_level:=1

