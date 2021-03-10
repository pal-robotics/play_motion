cmake_minimum_required(VERSION 3.5)
project(play_motion_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  action/PlayMotion.action
  msg/MotionInfo.msg
  srv/IsAlreadyThere.srv
  srv/ListMotions.srv
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
)

ament_package()
