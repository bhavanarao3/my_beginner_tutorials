# CMake generated Testfile for 
# Source directory: /home/bhavana/my_beginner_tutorials/src/ros2_integration_test
# Build directory: /home/bhavana/my_beginner_tutorials/build/integration_test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(ExampleIntegration_TestYAML "/usr/bin/python3" "-u" "/opt/ros/humble/share/catch_ros2/cmake/../scripts/run_test.py" "/home/bhavana/my_beginner_tutorials/build/integration_test/test_results/integration_test/ExampleIntegration_TestYAML.xml" "--package-name" "integration_test" "--command" "ros2" "launch" "integration_test" "integration_test.launch.yaml" "result_file:=/home/bhavana/my_beginner_tutorials/build/integration_test/test_results/integration_test/ExampleIntegration_TestYAML.xml")
set_tests_properties(ExampleIntegration_TestYAML PROPERTIES  TIMEOUT "60" WORKING_DIRECTORY "/home/bhavana/my_beginner_tutorials/build/integration_test" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/catch_ros2/cmake/catch_ros2_add_integration_test.cmake;69;ament_add_test;/home/bhavana/my_beginner_tutorials/src/ros2_integration_test/CMakeLists.txt;54;catch_ros2_add_integration_test;/home/bhavana/my_beginner_tutorials/src/ros2_integration_test/CMakeLists.txt;0;")
