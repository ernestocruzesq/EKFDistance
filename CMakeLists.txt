cmake_minimum_required(VERSION 2.8.3)
project(inverse_depth)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  #opencv2
  OpenCV
  roscpp
  rospy
  sensor_msgs
  std_msgs
  dynamic_reconfigure
)

find_package(OpenCV REQUIRED)
find_package(Qt4 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES Ar_Drone_keyboard
#  CATKIN_DEPENDS roscpp parrot_arsdk bebop_msgs cv_bridge sensor_msgs std_msgs image_transport
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

include( ${QT_USE_FILE} )
add_definitions( ${QT_DEFINITIONS} )

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

include_directories(${QT_INCLUDES} include include/inverse_depth)

## Declare a C++ executable
add_executable(inverse_depth src/inverse_d.cpp src/inverse_d_aux.cpp src/pose_translation.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(inverse_depth ${QT_LIBRARIES}
   ${catkin_LIBRARIES})



