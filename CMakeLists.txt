cmake_minimum_required(VERSION 2.8.3)
project(active_semantic_mapping)

find_package(PCL REQUIRED)
find_package(Boost COMPONENTS system filesystem regex REQUIRED)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3 -Wno-deprecated-declarations -std=c++11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE}")

include_directories(include
  ${PCL_INCLUDE_DIRS}
)


# CYLINDER LIBS
add_library (cylinder_segmentation_hough src/cylinder_segmentation.cpp src/cylinder_segmentation_hough.cpp)
target_link_libraries (cylinder_segmentation_hough ${PCL_LIBRARIES})

add_library (cylinder_segmentation_ransac src/cylinder_segmentation.cpp src/cylinder_segmentation_ransac.cpp)
target_link_libraries (cylinder_segmentation_ransac ${PCL_LIBRARIES})

# TESTS
add_executable(cylinder_generation test/cylinder_generation.cpp)
target_link_libraries(cylinder_generation ${PCL_LIBRARIES})

add_executable(sphere_generation test/sphere_generation.cpp)
target_link_libraries(sphere_generation ${PCL_LIBRARIES})

add_executable(cylinder_fitting_test test/cylinder_fitting_test.cpp)
target_link_libraries(cylinder_fitting_test ${PCL_LIBRARIES} cylinder_segmentation_hough cylinder_segmentation_ransac)




