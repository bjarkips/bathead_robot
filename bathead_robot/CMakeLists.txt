cmake_minimum_required(VERSION 2.8.3)
project(bathead_robot)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
#  gazebo_ros
  rosaria
#  gazebo_plugins
#  gazebo_dev
#  gazebo_msgs
  sensor_msgs)
find_package(gazebo REQUIRED)

include_directories(
  include ${catkin_INCLUDE_DIRS}include
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

link_directories(
  ${catkin_LIBRARY_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
#  gazebo_ros_sonar
  CATKIN_DEPENDS
  message_runtime
  tf
  #gazebo_msgs
  roscpp
  #geometry_msgs
  sensor_msgs
  #std_msgs
)

#add_library(gazebo_ros_sonar src/sonar_plugin_pub.cpp)
#target_link_libraries(gazebo_ros_sonar ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(gazebo_ros_sonar ${PROJECT_NAME}_gencfg)

#install(TARGETS
#  gazebo_ros_sonar
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

add_executable(bathead_control_node
  include/bathead_robot/bathead_control_node.h
  src/bathead_control_node.cpp
)
add_dependencies(bathead_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bathead_control_node
  ${catkin_LIBRARIES}
)
add_executable(bathead_control_node_sim
  include/bathead_robot/bathead_control_node.h
  src/bathead_control_node_sim.cpp
)
add_dependencies(bathead_control_node_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bathead_control_node_sim
  ${catkin_LIBRARIES}
)
# add_service_files(
#   FILES
# )
