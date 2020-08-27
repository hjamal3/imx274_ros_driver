cmake_minimum_required(VERSION 2.8.3)
project(imx274_ros_driver)

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

find_package(OpenCV REQUIRED)

add_executable(mipi_cam_node src/cameraNode.cpp)
target_link_libraries(mipi_cam_node 
          ${catkin_LIBRARIES}
          ${OpenCV_LIBRARIES}
)

#add_executable(mipi_sampler_driver_node src/mipi_sampler_driver.cpp)
#add_dependencies(mipi_sampler_driver_node  ${catkin_EXPORTED_TARGETS})
#target_link_libraries(mipi_sampler_driver_node 
#          ${catkin_LIBRARIES}
#          ${OpenCV_LIBRARIES}
#)
