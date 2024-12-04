cmake_minimum_required(VERSION 3.0.2)
project(height_mapping_utils)
add_compile_options(-std=c++17)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  tf2_eigen
  pcl_ros
  pcl_conversions
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    tf2_ros
    tf2_eigen
    pcl_ros
    pcl_conversions
  DEPENDS
    PCL
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Add an empty library target: to build header-only library
add_library(${PROJECT_NAME} src/empty.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)