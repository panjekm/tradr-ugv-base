cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_color)

find_package(catkin REQUIRED
  COMPONENTS
    cv_bridge
    image_geometry
    image_transport
    laser_geometry
    nodelet
    pcl_conversions
    roscpp
    sensor_msgs
    tf
)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    image_geometry
    image_transport
    laser_geometry
    nodelet
    pcl_conversions
    roscpp
    sensor_msgs
    tf
  DEPENDS OpenCV PCL
  LIBRARIES ${PROJECT_NAME}_plugins
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_plugins src/point_cloud_color/point_cloud2_color.cpp src/point_cloud_color/scan_to_point_cloud2.cpp)
target_link_libraries(${PROJECT_NAME}_plugins ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}_plugins
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(
  FILES ${PROJECT_NAME}_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

