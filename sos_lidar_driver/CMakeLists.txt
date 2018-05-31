cmake_minimum_required(VERSION 2.8.3)
project(sos_lidar_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV)

set(V4L2_LIBRARIES v4l2)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES sos_lidar_driver
   CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${V4L2_INCLUDE_DIRS}
  include
)

add_executable(sos_lidar_publisher src/sos_lidar_publisher.cpp src/sos_lidar.cpp)
add_dependencies(sos_lidar_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sos_lidar_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${V4L2_LIBRARIES})

install(TARGETS sos_lidar_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
