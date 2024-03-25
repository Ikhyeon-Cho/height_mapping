cmake_minimum_required(VERSION 3.0.2)
project(height_map_msgs)
add_compile_options(-std=c++17)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    height_map_core
    visualization_msgs
    nav_msgs
    message_generation
    grid_map_msgs
    grid_map_cv
    grid_map_ros
)

add_service_files(
  FILES
  SaveLayerToImage.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    height_map_core
    visualization_msgs
    nav_msgs
    grid_map_msgs
    grid_map_cv
    grid_map_ros
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

add_library(${PROJECT_NAME}
  src/HeightMapMsgs.cpp
  src/HeightMapConverter.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)