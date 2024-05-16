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
    height_map_pcl
)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    roscpp
    tf2_ros
    pcl_conversions
    height_map_core
    height_map_msgs
    height_map_pcl
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

# Sensor processor node
add_executable(sensor_processor_node
  src/sensor_processing/sensor_processor_node.cpp
  src/sensor_processing/SensorProcessor.cpp
)
target_link_libraries(sensor_processor_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Height mapping node
add_executable(height_mapping_node
  src/height_mapping/height_mapping_node.cpp
  src/height_mapping/HeightMapping.cpp
)
target_link_libraries(height_mapping_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Global mapping node
add_executable(global_mapping_node 
  src/global_mapping/global_mapping_node.cpp
  src/global_mapping/GlobalMapping.cpp
)
target_link_libraries(global_mapping_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)