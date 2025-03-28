cmake_minimum_required(VERSION 3.0.2)
project(data_generation)
add_compile_options(-std=c++17)

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2_ros
    grid_map_ros
    pcl_conversions
    height_mapping_core
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

# Node: Data generation
add_executable(gt_generation_node 
  src/ros/gt_generation_node.cpp
  src/core/GlobalMapper.cpp
  src/core/HeightMapper.cpp
)
target_link_libraries(gt_generation_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Node: Ray generation
add_executable(ray_generation_node 
  src/ros/ray_generation_node.cpp
  src/core/GlobalMapper.cpp
  src/core/HeightMapper.cpp
)
target_link_libraries(ray_generation_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)