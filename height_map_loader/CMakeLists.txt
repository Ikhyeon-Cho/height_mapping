cmake_minimum_required(VERSION 3.0.2)
project(height_map_loader)
add_compile_options(-std=c++14)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    height_map_core
    height_map_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    roscpp
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

add_executable(height_map_loader_node src/height_map_loader_node.cpp
  src/HeightMapLoader.cpp
)
target_link_libraries(height_map_loader_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)