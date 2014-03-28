cmake_minimum_required(VERSION 2.8.3)
project(play_motion_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation actionlib_msgs)

add_message_files(DIRECTORY msg FILES MotionInfo.msg)
add_action_files(DIRECTORY action FILES PlayMotion.action)
add_service_files(DIRECTORY srv FILES IsAlreadyThere.srv
                                      ListMotions.srv)
generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime actionlib_msgs)
