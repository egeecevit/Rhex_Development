# CMake generated Testfile for 
# Source directory: /home/ege/ros2_ws/src/ros-controls/ros2_controllers/joint_trajectory_controller
# Build directory: /home/ege/ros2_ws/build/joint_trajectory_controller
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_trajectory "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ege/ros2_ws/build/joint_trajectory_controller/test_results/joint_trajectory_controller/test_trajectory.gtest.xml" "--package-name" "joint_trajectory_controller" "--output-file" "/home/ege/ros2_ws/build/joint_trajectory_controller/ament_cmake_gmock/test_trajectory.txt" "--command" "/home/ege/ros2_ws/build/joint_trajectory_controller/test_trajectory" "--gtest_output=xml:/home/ege/ros2_ws/build/joint_trajectory_controller/test_results/joint_trajectory_controller/test_trajectory.gtest.xml")
set_tests_properties(test_trajectory PROPERTIES  LABELS "gmock" REQUIRED_FILES "/home/ege/ros2_ws/build/joint_trajectory_controller/test_trajectory" TIMEOUT "60" WORKING_DIRECTORY "/home/ege/ros2_ws/build/joint_trajectory_controller" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gmock/cmake/ament_add_gmock.cmake;106;ament_add_test;/opt/ros/humble/share/ament_cmake_gmock/cmake/ament_add_gmock.cmake;52;_ament_add_gmock;/home/ege/ros2_ws/src/ros-controls/ros2_controllers/joint_trajectory_controller/CMakeLists.txt;59;ament_add_gmock;/home/ege/ros2_ws/src/ros-controls/ros2_controllers/joint_trajectory_controller/CMakeLists.txt;0;")
add_test(test_trajectory_controller "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ege/ros2_ws/build/joint_trajectory_controller/test_results/joint_trajectory_controller/test_trajectory_controller.gtest.xml" "--package-name" "joint_trajectory_controller" "--output-file" "/home/ege/ros2_ws/build/joint_trajectory_controller/ament_cmake_gmock/test_trajectory_controller.txt" "--command" "/home/ege/ros2_ws/build/joint_trajectory_controller/test_trajectory_controller" "--gtest_output=xml:/home/ege/ros2_ws/build/joint_trajectory_controller/test_results/joint_trajectory_controller/test_trajectory_controller.gtest.xml")
set_tests_properties(test_trajectory_controller PROPERTIES  LABELS "gmock" REQUIRED_FILES "/home/ege/ros2_ws/build/joint_trajectory_controller/test_trajectory_controller" TIMEOUT "220" WORKING_DIRECTORY "/home/ege/ros2_ws/build/joint_trajectory_controller" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gmock/cmake/ament_add_gmock.cmake;106;ament_add_test;/opt/ros/humble/share/ament_cmake_gmock/cmake/ament_add_gmock.cmake;52;_ament_add_gmock;/home/ege/ros2_ws/src/ros-controls/ros2_controllers/joint_trajectory_controller/CMakeLists.txt;63;ament_add_gmock;/home/ege/ros2_ws/src/ros-controls/ros2_controllers/joint_trajectory_controller/CMakeLists.txt;0;")
add_test(test_load_joint_trajectory_controller "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ege/ros2_ws/build/joint_trajectory_controller/test_results/joint_trajectory_controller/test_load_joint_trajectory_controller.gtest.xml" "--package-name" "joint_trajectory_controller" "--output-file" "/home/ege/ros2_ws/build/joint_trajectory_controller/ament_cmake_gmock/test_load_joint_trajectory_controller.txt" "--command" "/home/ege/ros2_ws/build/joint_trajectory_controller/test_load_joint_trajectory_controller" "--gtest_output=xml:/home/ege/ros2_ws/build/joint_trajectory_controller/test_results/joint_trajectory_controller/test_load_joint_trajectory_controller.gtest.xml")
set_tests_properties(test_load_joint_trajectory_controller PROPERTIES  LABELS "gmock" REQUIRED_FILES "/home/ege/ros2_ws/build/joint_trajectory_controller/test_load_joint_trajectory_controller" TIMEOUT "60" WORKING_DIRECTORY "/home/ege/ros2_ws/build/joint_trajectory_controller" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gmock/cmake/ament_add_gmock.cmake;106;ament_add_test;/opt/ros/humble/share/ament_cmake_gmock/cmake/ament_add_gmock.cmake;52;_ament_add_gmock;/home/ege/ros2_ws/src/ros-controls/ros2_controllers/joint_trajectory_controller/CMakeLists.txt;71;ament_add_gmock;/home/ege/ros2_ws/src/ros-controls/ros2_controllers/joint_trajectory_controller/CMakeLists.txt;0;")
add_test(test_trajectory_actions "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ege/ros2_ws/build/joint_trajectory_controller/test_results/joint_trajectory_controller/test_trajectory_actions.gtest.xml" "--package-name" "joint_trajectory_controller" "--output-file" "/home/ege/ros2_ws/build/joint_trajectory_controller/ament_cmake_gmock/test_trajectory_actions.txt" "--command" "/home/ege/ros2_ws/build/joint_trajectory_controller/test_trajectory_actions" "--gtest_output=xml:/home/ege/ros2_ws/build/joint_trajectory_controller/test_results/joint_trajectory_controller/test_trajectory_actions.gtest.xml")
set_tests_properties(test_trajectory_actions PROPERTIES  LABELS "gmock" REQUIRED_FILES "/home/ege/ros2_ws/build/joint_trajectory_controller/test_trajectory_actions" TIMEOUT "300" WORKING_DIRECTORY "/home/ege/ros2_ws/build/joint_trajectory_controller" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gmock/cmake/ament_add_gmock.cmake;106;ament_add_test;/opt/ros/humble/share/ament_cmake_gmock/cmake/ament_add_gmock.cmake;52;_ament_add_gmock;/home/ege/ros2_ws/src/ros-controls/ros2_controllers/joint_trajectory_controller/CMakeLists.txt;84;ament_add_gmock;/home/ege/ros2_ws/src/ros-controls/ros2_controllers/joint_trajectory_controller/CMakeLists.txt;0;")
subdirs("gmock")
subdirs("gtest")
