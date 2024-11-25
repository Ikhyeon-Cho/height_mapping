cmake_minimum_required(VERSION 3.0.2)
project(height_mapping_io)
add_compile_options(-std=c++14)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    height_mapping_core
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
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

link_directories(${PCL_LIBRARY_DIRS})

# add_library(${PROJECT_NAME}

# )

# target_link_libraries(${PROJECT_NAME}
#   ${PCL_LIBRARIES}
#   ${catkin_LIBRARIES}
# )

