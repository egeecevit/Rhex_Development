# CMake generated Testfile for 
# Source directory: /home/ege/ros2_ws/src/ros-controls/ros2_control/hardware_interface_testing
# Build directory: /home/ege/ros2_ws/build/hardware_interface_testing
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_resource_manager "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ege/ros2_ws/build/hardware_interface_testing/test_results/hardware_interface_testing/test_resource_manager.gtest.xml" "--package-name" "hardware_interface_testing" "--output-file" "/home/ege/ros2_ws/build/hardware_interface_testing/ament_cmake_gmock/test_resource_manager.txt" "--command" "/home/ege/ros2_ws/build/hardware_interface_testing/test_resource_manager" "--gtest_output=xml:/home/ege/ros2_ws/build/hardware_interface_testing/test_results/hardware_interface_testing/test_resource_manager.gtest.xml")
set_tests_properties(test_resource_manager PROPERTIES  LABELS "gmock" REQUIRED_FILES "/home/ege/ros2_ws/build/hardware_interface_testing/test_resource_manager" TIMEOUT "60" WORKING_DIRECTORY "/home/ege/ros2_ws/build/hardware_interface_testing" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gmock/cmake/ament_add_gmock.cmake;106;ament_add_test;/opt/ros/humble/share/ament_cmake_gmock/cmake/ament_add_gmock.cmake;52;_ament_add_gmock;/home/ege/ros2_ws/src/ros-controls/ros2_control/hardware_interface_testing/CMakeLists.txt;37;ament_add_gmock;/home/ege/ros2_ws/src/ros-controls/ros2_control/hardware_interface_testing/CMakeLists.txt;0;")
add_test(test_resource_manager_prepare_perform_switch "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ege/ros2_ws/build/hardware_interface_testing/test_results/hardware_interface_testing/test_resource_manager_prepare_perform_switch.gtest.xml" "--package-name" "hardware_interface_testing" "--output-file" "/home/ege/ros2_ws/build/hardware_interface_testing/ament_cmake_gmock/test_resource_manager_prepare_perform_switch.txt" "--command" "/home/ege/ros2_ws/build/hardware_interface_testing/test_resource_manager_prepare_perform_switch" "--gtest_output=xml:/home/ege/ros2_ws/build/hardware_interface_testing/test_results/hardware_interface_testing/test_resource_manager_prepare_perform_switch.gtest.xml")
set_tests_properties(test_resource_manager_prepare_perform_switch PROPERTIES  LABELS "gmock" REQUIRED_FILES "/home/ege/ros2_ws/build/hardware_interface_testing/test_resource_manager_prepare_perform_switch" TIMEOUT "60" WORKING_DIRECTORY "/home/ege/ros2_ws/build/hardware_interface_testing" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gmock/cmake/ament_add_gmock.cmake;106;ament_add_test;/opt/ros/humble/share/ament_cmake_gmock/cmake/ament_add_gmock.cmake;52;_ament_add_gmock;/home/ege/ros2_ws/src/ros-controls/ros2_control/hardware_interface_testing/CMakeLists.txt;40;ament_add_gmock;/home/ege/ros2_ws/src/ros-controls/ros2_control/hardware_interface_testing/CMakeLists.txt;0;")
subdirs("gmock")
subdirs("gtest")
