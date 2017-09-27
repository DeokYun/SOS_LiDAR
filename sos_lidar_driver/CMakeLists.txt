cmake_minimum_required(VERSION 2.8.3)
project(sos_lidar_driver)

##################### V4L2 package 찾아내는 코드 ##################################
## cmake 폴더 추가함
set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")
set(CMAKE_BUILD_TYPE Release)
##################################################################################

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  image_transport
  camera_info_manager
  sensor_msgs
  std_msgs
)

find_package(OpenCV)

find_package(V4L2 REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES sos_lidar_driver
   CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport
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
