cmake_minimum_required(VERSION 3.0.2)
project(virtual_navigation_layers)

# # Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  geometry_msgs
  move_base
  nav_core
  pluginlib
  sensor_msgs
  std_msgs
  visualization_msgs
  nav_msgs
  dynamic_reconfigure
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(

  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

generate_dynamic_reconfigure_options(config/VirtualLayer.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES virtual_navigation_layers
  CATKIN_DEPENDS
  roscpp
  costmap_2d
  geometry_msgs
  move_base
  nav_core
  pluginlib
  sensor_msgs
  std_msgs
  visualization_msgs
  nav_msgs
  dynamic_reconfigure
)

add_library(virtual_navigation_layers src/virtual_layer.cpp)
add_dependencies(virtual_navigation_layers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(virtual_navigation_layers ${catkin_LIBRARIES})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS virtual_navigation_layers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)