cmake_minimum_required(VERSION 3.22)
project(glim_localization_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LocalizationStatus.msg"
  "srv/Initialize.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

if(BUILD_TESTING)
  # ROS 2 code style (CONTRIBUTING.md): the linters of ament_lint_common. Here they check CMakeLists.txt
  # and package.xml; the generated code in the build directory is not linted.
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
