cmake_minimum_required(VERSION 2.8.3)
project(omnicamera_msgs)

# Find catkin macros and libraries, also other catkin packages.
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(FILES VirtualCameraConfig.msg)
add_service_files(FILES GetVirtualCameraConfig.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

