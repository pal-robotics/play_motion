cmake_minimum_required(VERSION 2.4.6)
project(play_motion)

if(USE_ROSBUILD)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  rosbuild_find_ros_package(actionlib_msgs)
  include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
  genaction()

  rosbuild_init()

  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  rosbuild_genmsg()

  rosbuild_add_boost_directories()

  rosbuild_add_executable(play_motion
      src/move_joint_group.cpp
      src/play_motion.cpp
      src/play_motion_server.cpp
      src/play_motion_main.cpp
      src/controller_updater.cpp)

  rosbuild_add_executable(rrbot test/rrbot.cpp)
  rosbuild_add_executable(play_motion_test test/play_motion_test.cpp)
  rosbuild_add_gtest_build_flags(play_motion_test)
  rosbuild_add_rostest(test/play_motion.test)

  rosbuild_link_boost(play_motion thread)

else()

  find_package(catkin REQUIRED COMPONENTS roscpp genmsg control_msgs actionlib
    controller_manager_msgs)

  add_action_files(DIRECTORY action FILES PlayMotion.action)
  generate_messages(DEPENDENCIES actionlib_msgs)

  catkin_package()

  include_directories(include ${catkin_INCLUDE_DIRS})

  add_executable(play_motion
    src/play_motion_main.cpp
    src/play_motion_server.cpp
    src/play_motion.cpp
    src/move_joint_group.cpp
    src/controller_updater.cpp)

  target_link_libraries(play_motion ${catkin_LIBRARIES})
  add_dependencies(play_motion ${PROJECT_NAME}_gencpp)

  install(TARGETS play_motion
     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )

endif()
