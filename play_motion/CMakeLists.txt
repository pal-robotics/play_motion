cmake_minimum_required(VERSION 2.8.3)
project(play_motion)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(catkin REQUIRED COMPONENTS actionlib control_msgs controller_manager_msgs
  play_motion_msgs moveit_ros_planning_interface roscpp sensor_msgs
  diagnostic_msgs diagnostic_updater)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_python_setup()

catkin_package(INCLUDE_DIRS include
               LIBRARIES play_motion_helpers
               CATKIN_DEPENDS actionlib control_msgs controller_manager_msgs
               play_motion_msgs moveit_ros_planning_interface roscpp sensor_msgs)


include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_library(play_motion_helpers src/play_motion_helpers.cpp)

target_link_libraries(play_motion_helpers ${catkin_LIBRARIES})
add_executable(play_motion
  src/play_motion_main.cpp
  src/play_motion_server.cpp
  src/play_motion.cpp
  src/move_joint_group.cpp
  src/controller_updater.cpp
  src/approach_planner.cpp)

target_link_libraries(play_motion play_motion_helpers ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(play_motion play_motion_msgs_generate_messages_cpp)

add_executable(run_motion src/run_motion_node.cpp)
target_link_libraries(run_motion ${catkin_LIBRARIES})
add_dependencies(run_motion ${catkin_EXPORTED_TARGETS})

install(TARGETS play_motion
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(TARGETS play_motion_helpers 
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(TARGETS run_motion
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install (PROGRAMS scripts/is_already_there.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS scripts/move_joint scripts/run_motion_python_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

# tests
#######
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(controller_manager REQUIRED)

  include_directories(SYSTEM ${controller_manager_INCLUDE_DIRS}
    ${joint_trajectory_controller_INCLUDE_DIRS})

  add_executable(pm_rrbot test/rrbot.cpp)
  target_link_libraries(pm_rrbot ${catkin_LIBRARIES} ${controller_manager_LIBRARIES})

  add_dependencies(tests pm_rrbot)

  add_rostest_gtest(play_motion_test test/play_motion.test test/play_motion_test.cpp)
  add_dependencies(play_motion_test play_motion joint_trajectory_controller)
  target_link_libraries(play_motion_test ${catkin_LIBRARIES})
  add_rostest(test/move_joint.test)

  add_rostest_gtest(play_motion_helpers_test test/play_motion_helpers.test test/play_motion_helpers_test.cpp)
  target_link_libraries(play_motion_helpers_test play_motion_helpers ${catkin_LIBRARIES})
endif()
