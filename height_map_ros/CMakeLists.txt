cmake_minimum_required(VERSION 3.0.2)
project(height_map)
add_compile_options(-std=c++17)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2_ros
    pcl_conversions
    grid_map_core
    grid_map_ros
    grid_map_msgs
    nav_msgs
    visualization_msgs
    height_map_core
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    # height_map
  CATKIN_DEPENDS 
    roscpp
    tf2_ros
    pcl_conversions
    grid_map_core
    grid_map_ros
    grid_map_msgs
    nav_msgs
    visualization_msgs
    height_map_core
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

# add_executable(global_terrain_mapping_node src/global_terrain_mapping_node.cpp
#   src/terrain_mapping/GlobalTerrainMapping.cpp
#   src/terrain_mapping/ElevationMap.cpp
#   src/terrain_mapping/DescriptorMap.cpp
#   src/terrain_mapping/ElevationMapRosConverter.cpp
# )
# target_link_libraries(global_terrain_mapping_node
#   ${PCL_LIBRARIES}
#   ${catkin_LIBRARIES}
# )

# add_executable(traversability_mapping_node src/traversability_mapping_node.cpp
#   src/height_map/TraversabilityMapping.cpp
#   src/height_map/TraversabilityMap.cpp
# )
# target_link_libraries(traversability_mapping_node
#   ${PCL_LIBRARIES}
#   ${catkin_LIBRARIES}
# )