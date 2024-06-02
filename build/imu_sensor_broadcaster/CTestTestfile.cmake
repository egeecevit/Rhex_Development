# CMake generated Testfile for 
# Source directory: /home/dev-ros2/development/src/ros-controls/ros2_controllers/imu_sensor_broadcaster
# Build directory: /home/dev-ros2/development/build/imu_sensor_broadcaster
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_load_imu_sensor_broadcaster "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/dev-ros2/development/build/imu_sensor_broadcaster/test_results/imu_sensor_broadcaster/test_load_imu_sensor_broadcaster.gtest.xml" "--package-name" "imu_sensor_broadcaster" "--output-file" "/home/dev-ros2/development/build/imu_sensor_broadcaster/test_results/imu_sensor_broadcaster/test_load_imu_sensor_broadcaster.txt" "--command" "/home/dev-ros2/development/build/imu_sensor_broadcaster/test_load_imu_sensor_broadcaster" "--ros-args" "--params-file" "/home/dev-ros2/development/src/ros-controls/ros2_controllers/imu_sensor_broadcaster/test/imu_sensor_broadcaster_params.yaml" "--" "--gtest_output=xml:/home/dev-ros2/development/build/imu_sensor_broadcaster/test_results/imu_sensor_broadcaster/test_load_imu_sensor_broadcaster.gtest.xml")
set_tests_properties(test_load_imu_sensor_broadcaster PROPERTIES  TIMEOUT "60" WORKING_DIRECTORY "/home/dev-ros2/development/build/imu_sensor_broadcaster" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/generate_parameter_library/cmake/generate_parameter_library.cmake;160;ament_add_test;/home/dev-ros2/development/src/ros-controls/ros2_controllers/imu_sensor_broadcaster/CMakeLists.txt;55;add_rostest_with_parameters_gmock;/home/dev-ros2/development/src/ros-controls/ros2_controllers/imu_sensor_broadcaster/CMakeLists.txt;0;")
add_test(test_imu_sensor_broadcaster "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/dev-ros2/development/build/imu_sensor_broadcaster/test_results/imu_sensor_broadcaster/test_imu_sensor_broadcaster.gtest.xml" "--package-name" "imu_sensor_broadcaster" "--output-file" "/home/dev-ros2/development/build/imu_sensor_broadcaster/test_results/imu_sensor_broadcaster/test_imu_sensor_broadcaster.txt" "--command" "/home/dev-ros2/development/build/imu_sensor_broadcaster/test_imu_sensor_broadcaster" "--ros-args" "--params-file" "/home/dev-ros2/development/src/ros-controls/ros2_controllers/imu_sensor_broadcaster/test/imu_sensor_broadcaster_params.yaml" "--" "--gtest_output=xml:/home/dev-ros2/development/build/imu_sensor_broadcaster/test_results/imu_sensor_broadcaster/test_imu_sensor_broadcaster.gtest.xml")
set_tests_properties(test_imu_sensor_broadcaster PROPERTIES  TIMEOUT "60" WORKING_DIRECTORY "/home/dev-ros2/development/build/imu_sensor_broadcaster" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/generate_parameter_library/cmake/generate_parameter_library.cmake;160;ament_add_test;/home/dev-ros2/development/src/ros-controls/ros2_controllers/imu_sensor_broadcaster/CMakeLists.txt;68;add_rostest_with_parameters_gmock;/home/dev-ros2/development/src/ros-controls/ros2_controllers/imu_sensor_broadcaster/CMakeLists.txt;0;")
subdirs("gmock")
subdirs("gtest")
