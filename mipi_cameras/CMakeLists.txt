cmake_minimum_required(VERSION 2.8.3)
project(mipi_cameras)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  cv_bridge
  sensor_msgs
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
#   Service1.srv
#   Service2.srv
  camera.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
#   geometry_msgs#   sensor_msgs#   std_msgs
  std_msgs
)


catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES mipi_cameras
    CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs message_runtime
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mipi_cam_node src/cameraNode.cpp)
target_link_libraries(mipi_cam_node ${catkin_LIBRARIES})

add_executable(mipi_cam_node_fast src/cameraNodeV2.cpp)
target_link_libraries(mipi_cam_node_fast ${catkin_LIBRARIES})

add_executable(mipi_service_node src/mipi_service_node.cpp)
target_link_libraries(mipi_service_node ${catkin_LIBRARIES})
add_dependencies(mipi_service_node mipi_service_node_gencpp)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(mipi_cam_node ${OpenCV_LIBRARIES})
target_link_libraries(mipi_cam_node_fast ${OpenCV_LIBRARIES})
target_link_libraries(mipi_service_node ${OpenCV_LIBRARIES})
