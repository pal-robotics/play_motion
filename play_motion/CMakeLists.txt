cmake_minimum_required(VERSION 3.5)
project(play_motion)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(play_motion_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)

# catkin_python_setup()

# play_motion_helpers
add_library(play_motion_helpers SHARED
  src/play_motion_helpers.cpp
)

target_include_directories(play_motion_helpers PRIVATE
  include
)

ament_target_dependencies(play_motion_helpers
  diagnostic_msgs
  rclcpp
  trajectory_msgs
)

install(TARGETS play_motion_helpers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# play_motion
add_executable(play_motion
  src/approach_planner.cpp
  src/controller_updater.cpp
  src/move_joint_group.cpp
  src/play_motion.cpp
  src/play_motion_main.cpp
  src/play_motion_server.cpp
)

target_include_directories(play_motion PRIVATE
  include
)

target_link_libraries(play_motion
  play_motion_helpers
)

ament_target_dependencies(play_motion
  controller_manager_msgs
  moveit_ros_planning_interface
  play_motion_msgs
  rclcpp
  rclcpp_action
  trajectory_msgs
  control_msgs
)

install(TARGETS play_motion
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


# add_executable(run_motion src/run_motion_node.cpp)
# target_link_libraries(run_motion ${catkin_LIBRARIES})
# add_dependencies(run_motion ${catkin_EXPORTED_TARGETS})

#install(TARGETS run_motion
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
#install (PROGRAMS scripts/is_already_there.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#
#install(PROGRAMS scripts/move_joint scripts/run_motion_python_node.py
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(play_motion_msgs REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rcl_yaml_param_parser REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(sensor_msgs REQUIRED)

  ament_lint_auto_find_test_dependencies()

  # test configuration files
  install(
    FILES
      test/play_motion_helpers_poses.yaml
      test/rrbot/controller_manager.yaml
      test/rrbot/controllers.launch.py
      test/rrbot/joint_state_broadcaster.launch.py
      test/rrbot/joint_state_broadcaster.yaml
      test/rrbot/joint_trajectory_controller.launch.py
      test/rrbot/joint_trajectory_controller.yaml
      test/rrbot/rrbot.launch.py
      test/rrbot/rrbot.rviz
      test/rrbot/rrbot.xacro
    DESTINATION share/play_motion/test
  )

  # rrbot_system
  add_library(rrbot_system SHARED
    test/rrbot/rrbot_system.cpp
  )
  target_include_directories(rrbot_system PRIVATE
    include
  )
  ament_target_dependencies(rrbot_system
    hardware_interface
    pluginlib
    rclcpp
  )
  pluginlib_export_plugin_description_file(hardware_interface test/rrbot/rrbot_system.xml)
  install(
    TARGETS rrbot_system
    DESTINATION lib
  )

  # play_motion_helpers_test
  ament_add_gtest(play_motion_helpers_test
    test/play_motion_helpers_test.cpp
  )
  target_include_directories(play_motion_helpers_test PRIVATE
    include
  )
  ament_target_dependencies(play_motion_helpers_test
    ament_index_cpp
  )
  target_link_libraries(play_motion_helpers_test
    play_motion_helpers
  )


  #  find_package(controller_manager REQUIRED)
  #
  #  include_directories(SYSTEM ${controller_manager_INCLUDE_DIRS}
  #    ${joint_trajectory_controller_INCLUDE_DIRS})
  #
  #  add_executable(pm_rrbot test/rrbot.cpp)
  #  target_link_libraries(pm_rrbot ${catkin_LIBRARIES} ${controller_manager_LIBRARIES})
  #
  #  add_dependencies(tests pm_rrbot)
  #
  #  add_rostest_gtest(play_motion_test test/play_motion.test test/play_motion_test.cpp)
  #  add_dependencies(play_motion_test play_motion joint_trajectory_controller)
  #  target_link_libraries(play_motion_test ${catkin_LIBRARIES})

  # play_motion_test
  # @TODO this should be a launch test that also runs the rrbot
  ament_add_gtest(play_motion_test
    test/play_motion_test.cpp
  )
  ament_target_dependencies(play_motion_test
    play_motion_msgs
    rclcpp
    rclcpp_action
    sensor_msgs
  )

endif()

ament_export_include_directories(
  include
)

ament_export_dependencies(
)

ament_package()

