cmake_minimum_required(VERSION 2.8.3)
project(omnicamera)

# Find catkin macros and libraries, also other catkin packages.
find_package(catkin REQUIRED
  COMPONENTS
    camera_info_manager
    cv_bridge
    image_transport
    message_runtime
    nodelet
    omnicamera_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    camera_info_manager
    camera1394
    cv_bridge
    image_transport
    message_runtime
    nodelet
    omnicamera_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf
  LIBRARIES ${PROJECT_NAME}_plugins
)

include_directories(include ${catkin_INCLUDE_DIRS})

# Libraries
add_library(${PROJECT_NAME}_plugins
  src/imageColourCorrection_Nodelet.cpp
  src/omnicamera/auto_white_balance_nodelet.cpp
  src/omnicamera/LadybugPanoNodelet.cpp
  src/omnicamera/LadySaw.cpp
  src/omnicamera/LadySawNodelet.cpp
  src/omnicamera/LookupStitcher.cpp
  src/omnicamera/panolut_virtual_camera_common.cpp
  src/omnicamera/PanolutVirtualCameraNodelet.cpp
)
target_link_libraries(${PROJECT_NAME}_plugins ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_dependencies(${PROJECT_NAME}_plugins omnicamera_msgs_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_plugins ${catkin_EXPORTED_TARGETS})

# Nodes
add_executable(virtual_camera_display src/virtual_camera_display.cpp)
target_link_libraries(virtual_camera_display ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(virtual_camera_display ${catkin_EXPORTED_TARGETS})

add_executable(virtual_camera_conv src/virtual_camera_conv.cpp)
target_link_libraries(virtual_camera_conv ${catkin_LIBRARIES})
add_dependencies(virtual_camera_conv ${catkin_EXPORTED_TARGETS})

# Kinect depth view.
add_executable(depth_view src/depth_view.cpp)
target_link_libraries(depth_view ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(depth_view ${catkin_EXPORTED_TARGETS})

install(
  TARGETS ${PROJECT_NAME}_plugins
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(
  FILES ${PROJECT_NAME}_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

