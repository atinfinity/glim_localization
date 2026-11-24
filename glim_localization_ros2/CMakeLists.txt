cmake_minimum_required(VERSION 3.22)
project(glim_localization_ros2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(glim_localization REQUIRED)
find_package(glim_localization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glim REQUIRED)

if(NOT TARGET glim_localization::glim_localization)
  message(FATAL_ERROR "glim_localization::glim_localization target was not exported")
endif()

# LocalizationPublisher, EstimateBridge (glim::EstimationFrame to the publisher) and MapPublisher (~/map), shared by R1 and R2
add_library(localization_publisher SHARED src/localization_publisher.cpp src/estimate_bridge.cpp src/map_publisher.cpp)
target_compile_options(localization_publisher PRIVATE -Wall -Wextra)
target_include_directories(localization_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(localization_publisher PUBLIC
  glim_localization::glim_localization
  glim::glim
  Eigen3::Eigen
  ${diagnostic_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${glim_localization_msgs_TARGETS}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  tf2_ros::tf2_ros
)

# R1: loaded by glim_rosnode / glim_rosbag with dlopen, so the file name is fixed (see config/r1/config_ros.json).
add_library(glim_localization_extension MODULE src/localization_extension.cpp)
target_compile_options(glim_localization_extension PRIVATE -Wall -Wextra)
target_link_libraries(glim_localization_extension PRIVATE localization_publisher glim::glim)
set_target_properties(glim_localization_extension PROPERTIES PREFIX "lib" SUFFIX ".so")

# R2: the lifecycle node, as a component and as the executable glim_localization_node
add_library(localization_node SHARED src/localization_node.cpp)
target_compile_options(localization_node PRIVATE -Wall -Wextra)
target_link_libraries(localization_node PRIVATE
  localization_publisher
  glim::glim
  ament_index_cpp::ament_index_cpp
  ${lifecycle_msgs_TARGETS}
  rclcpp_components::component
  rclcpp_lifecycle::rclcpp_lifecycle
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
)
rclcpp_components_register_node(localization_node
  PLUGIN "glim_localization_ros2::LocalizationNode"
  EXECUTABLE glim_localization_node)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)
install(TARGETS glim_localization_extension LIBRARY DESTINATION lib)
install(TARGETS localization_node
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS localization_publisher
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # ROS isolation for parallel tests (colcon test passes ctest -j): each test that uses ROS runs through
  # run_test_isolated.py, which reserves a free ROS_DOMAIN_ID (1 to 100) for the time of the test. Unset
  # ROS_DOMAIN_ID before the tests (scripts/test_colcon.sh does), or every test shares the one that is set.
  find_package(ament_cmake_ros REQUIRED)
  set(ros_isolated_runner "${ament_cmake_ros_DIR}/run_test_isolated.py")

  # Keep DDS discovery on this host: tests in other containers on one network must not meet.
  ament_add_ros_isolated_gtest(test_message_constants test/test_message_constants.cpp
    ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)
  target_link_libraries(test_message_constants localization_publisher)

  ament_add_ros_isolated_gtest(test_localization_publisher test/test_localization_publisher.cpp
    ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)
  target_link_libraries(test_localization_publisher localization_publisher)

  ament_add_ros_isolated_gtest(test_map_publisher test/test_map_publisher.cpp
    ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)
  target_link_libraries(test_map_publisher localization_publisher)

  # No ROS spin: the gyro buffer of the R1 extension is plain C++
  ament_add_gtest(test_gyro_buffer test/test_gyro_buffer.cpp)
  target_link_libraries(test_gyro_buffer Eigen3::Eigen)
  target_include_directories(test_gyro_buffer PRIVATE include)

  # No ROS: the online check of a late initial pose (R2)
  ament_add_gtest(test_init_pose_delay test/test_init_pose_delay.cpp)
  target_include_directories(test_init_pose_delay PRIVATE include)

  # End to end over synthetic bags. They need the datasets made by scripts/test_colcon.sh (see the tests).
  # Jazzy has no isolated launch test helper; add_launch_test passes RUNNER on to ament_add_test.
  find_package(launch_testing_ament_cmake REQUIRED)
  # test_launch_*: the installed launch files (launch/), headless (rviz:=false)
  foreach(launch_test test_r1_extension test_r2_node test_r2_imu_gap test_launch_r1 test_launch_r2 test_launch_r2_component)
    add_launch_test(test/${launch_test}.py TIMEOUT 300 RUNNER "${ros_isolated_runner}"
      ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)
  endforeach()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  glim glim_localization glim_localization_msgs diagnostic_msgs geometry_msgs nav_msgs rclcpp sensor_msgs tf2_ros Eigen3)
ament_package()
