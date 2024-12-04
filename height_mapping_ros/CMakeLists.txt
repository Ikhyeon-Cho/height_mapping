cmake_minimum_required(VERSION 3.0.2)
project(height_mapping)
add_compile_options(-std=c++17)

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2_ros
    pcl_conversions
    height_mapping_core
    height_mapping_msgs
    height_mapping_io
    height_mapping_utils
)
catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    roscpp
    tf2_ros
    pcl_conversions
    height_mapping_core
    height_mapping_msgs
    height_mapping_io
    height_mapping_utils
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
  src/executables/sensor_processor_main.cpp
  src/nodes/SensorProcessorNode.cpp
  # src/SensorProcessor.cpp
)
target_link_libraries(sensor_processor_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Height mapping node
add_executable(height_mapping_node 
  src/executables/height_mapping_main.cpp
  src/nodes/HeightMappingNode.cpp
  src/HeightMapping.cpp
)
target_link_libraries(height_mapping_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Global mapping node
add_executable(global_mapping_node 
  src/executables/global_mapping_main.cpp
  src/nodes/GlobalMappingNode.cpp
  src/GlobalMapping.cpp
)
target_link_libraries(global_mapping_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
