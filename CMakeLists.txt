cmake_minimum_required(VERSION 2.8.3)
project(play_motion)

find_package(catkin REQUIRED COMPONENTS roscpp genmsg actionlib)
find_package(Boost REQUIRED COMPONENTS thread)

add_action_files(DIRECTORY action FILES PlayMotion.action)
add_service_files(DIRECTORY srv FILES IsAlreadyThere.srv)
generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(play_motion
  src/play_motion_main.cpp
  src/play_motion_server.cpp
  src/play_motion.cpp
  src/move_joint_group.cpp
  src/controller_updater.cpp)

target_link_libraries(play_motion ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(play_motion ${PROJECT_NAME}_gencpp)

install(TARGETS play_motion
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

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
endif()
