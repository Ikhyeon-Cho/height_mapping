cmake_minimum_required(VERSION 3.0.2)
project(height_mapping)
add_compile_options(-std=c++17)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2_ros
    pcl_conversions
    height_map_core
    height_map_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    roscpp
    tf2_ros
    pcl_conversions
    height_map_core
    height_map_msgs
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

add_executable(height_mapping_node src/height_mapping_node.cpp
  src/HeightMapping.cpp
)
target_link_libraries(height_mapping_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(height_map_visualization_node src/height_map_visualization_node.cpp
  src/HeightMapVisualization.cpp
)
target_link_libraries(height_map_visualization_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(global_mapping_node src/global_mapping_node.cpp
  src/GlobalMapping.cpp
)
target_link_libraries(global_mapping_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)