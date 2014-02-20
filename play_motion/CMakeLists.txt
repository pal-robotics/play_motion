cmake_minimum_required(VERSION 2.8.3)
project(play_motion)

find_package(catkin REQUIRED COMPONENTS roscpp play_motion_msgs actionlib sensor_msgs)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(INCLUDE_DIRS include
               LIBRARIES play_motion_helpers)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(play_motion_helpers src/play_motion_helpers.cpp)

target_link_libraries(play_motion_helpers ${catkin_LIBRARIES})
add_executable(play_motion
  src/play_motion_main.cpp
  src/play_motion_server.cpp
  src/play_motion.cpp
  src/move_joint_group.cpp
  src/controller_updater.cpp)

target_link_libraries(play_motion play_motion_helpers ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(play_motion play_motion_msgs_generate_messages_cpp)

install(TARGETS play_motion
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(TARGETS play_motion_helpers
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install (PROGRAMS scripts/is_already_there.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# tests
#######
if(CATKIN_ENABLE_TESTING)
  find_package(catkin COMPONENTS rostest actionlib controller_manager joint_trajectory_controller)

  include_directories(include ${catkin_INCLUDE_DIRS})

  add_executable(pm_rrbot test/rrbot.cpp)
  target_link_libraries(pm_rrbot ${catkin_LIBRARIES})

  add_dependencies(tests pm_rrbot)

  add_rostest_gtest(play_motion_test test/play_motion.test test/play_motion_test.cpp)
  add_dependencies(play_motion_test play_motion joint_trajectory_controller)
  target_link_libraries(play_motion_test ${catkin_LIBRARIES})

  add_rostest_gtest(play_motion_helpers_test test/play_motion_helpers.test test/play_motion_helpers_test.cpp)
  target_link_libraries(play_motion_helpers_test play_motion_helpers ${catkin_LIBRARIES})
endif()
