# my_beginner_tutorials
ENPM700 ROS2 Programming Assignment


This repository contains basic ROS2 tutorials involving the creation and use of publisher and subscriber member functions in C++ with ROS2.

## Directory Structure

```
my_beginner_tutorials/
├── src/
│   ├── beginner_tutorials/
│   │   ├── src/
│   │   │   ├── publisher_member_function.cpp  # Publisher node that demonstrates ROS2 C++ publishing
│   │   │   ├── subscriber_member_function.cpp # Subscriber node that demonstrates ROS2 C++ subscription
```

### `publisher_member_function.cpp`

This file contains a basic ROS2 C++ publisher node that demonstrates the use of a publisher for sending messages.

### `subscriber_member_function.cpp`

This file contains a basic ROS2 C++ subscriber node that demonstrates the use of a subscriber to receive messages.

## ROS2 Dependencies

- **rclcpp**: ROS Client Library for C++.
- **std_msgs**: Standard message types for ROS, such as strings and integers.
- Other relevant dependencies (for building ROS2 nodes).

Make sure to install the ROS2 Humble version or the appropriate ROS2 distribution on your system and source the ROS2 setup script.

## How to Build and Run

In the root directory of this repository, use the following commands to build the ROS2 package:

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### Why Use `--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`?

- **`colcon build`**: This is the standard command for building ROS2 packages.
- **`--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`**: This additional flag ensures that the compilation commands are exported in a `compile_commands.json` file. This file is essential for static analysis tools like `clang-tidy` and `cpplint` to analyze the code properly. Without this file, the tools may not function correctly because they wouldn't have the necessary context about the build environment (compiler flags, include paths, etc.).

### How to Run the Nodes

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

## Static Code Analysis with `clang-tidy` and `cpplint`

We use static analysis tools to ensure code quality and consistency. Below are the commands to run these tools.

### `clang-tidy` Command

Run `clang-tidy` with the following command to check for potential issues in the code:

```bash
clang-tidy -p ./ --extra-arg=-stdlib=libc++ --extra-arg=-I/usr/include/c++/11 --extra-arg=-I/usr/include/x86_64-linux-gnu/c++/11 $(find . -name "*.cpp" | grep -v "/build/")
```

- **`-p ./`**: Specifies the location of the `compile_commands.json` file.
- **`--extra-arg=-stdlib=libc++`**: Specifies the standard library to be used for analysis (in this case, `libc++`).
- **`--extra-arg=-I/usr/include/c++/11` and `--extra-arg=-I/usr/include/x86_64-linux-gnu/c++/11`**: Adds the necessary include paths for the C++ standard library and architecture-specific directories.
- **`$(find . -name "*.cpp" | grep -v "/build/")`**: Finds all `.cpp` files in the repository, excluding those in the `/build/` directory.

### `cpplint` Command

Run `cpplint` to check the style of the C++ code using this command:

```bash
cpplint --filter="-legal/copyright" $(find . -name '*.cpp' | grep -v "/build/")
```

- **`--filter="-legal/copyright"`**: Excludes copyright-related checks.
- **`$(find . -name '*.cpp' | grep -v "/build/")`**: Finds all `.cpp` files, excluding the `/build/` directory.

## Additional Notes

- Make sure to have ROS2 dependencies installed and the ROS2 workspace properly set up before building and running the project.
- If you encounter any issues, ensure your system has all required tools (`clang-tidy`, `cpplint`, `colcon`, etc.) installed and that you are working in a properly sourced ROS2 environment.


