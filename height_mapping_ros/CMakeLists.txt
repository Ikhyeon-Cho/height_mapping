cmake_minimum_required(VERSION 3.0.2)
project(height_mapping)
add_compile_options(-std=c++17)

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2_ros
    grid_map_ros
    pcl_conversions
    height_mapping_core
    # height_mapping_io
    message_generation
)

add_service_files(
  FILES
  save_map.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    roscpp
    tf2_ros
    grid_map_ros
    pcl_conversions
    height_mapping_core
    # height_mapping_io
)

###########
## Build ##
###########
set(CMAKE_BUILD_TYPE Release)
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Node: Height mapping
add_executable(height_mapping_node 
  src/ros/height_mapping_node.cpp
  src/core/HeightMapper.cpp)
target_link_libraries(height_mapping_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Node: Global mapping
add_executable(global_mapping_node 
  src/ros/global_mapping_node.cpp
  src/core/HeightMapper.cpp
  src/core/GlobalMapper.cpp
)
target_link_libraries(global_mapping_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# Node: Sensor processor
add_executable(sensor_processor_node 
  src/ros/sensor_processor_node.cpp)
target_link_libraries(sensor_processor_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)