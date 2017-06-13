cmake_minimum_required(VERSION 2.8.3)
project(active_semantic_mapping)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system filesystem regex REQUIRED)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3 -std=c++11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE}")

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


## Declare a C++ library
# add_library(active_semantic_mapping
#   src/${PROJECT_NAME}/active_semantic_mapping.cpp
# )

# CYLINDER LIBS
add_library (cylinder_segmentation_hough src/cylinder_segmentation.cpp src/cylinder_segmentation_hough.cpp)
target_link_libraries (cylinder_segmentation_hough ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_library (cylinder_segmentation_ransac src/cylinder_segmentation.cpp src/cylinder_segmentation_ransac.cpp)
target_link_libraries (cylinder_segmentation_ransac ${PCL_LIBRARIES} ${catkin_LIBRARIES})
