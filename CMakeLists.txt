cmake_minimum_required(VERSION 2.8.3)
project(shape-fitting)

find_package(PCL REQUIRED)
find_package(Boost COMPONENTS system filesystem regex REQUIRED)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3 -Wno-comment -Wno-deprecated-declarations -std=c++11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE}")

include_directories(include
  test
  ${PCL_INCLUDE_DIRS}
)


# FITTING LIB
add_library (fitting_library src/cylinder_fitting.cpp src/cylinder_fitting_hough.cpp src/sphere_fitting_hough.cpp src/plane_fitting.cpp src/plane_fitting_ransac.cpp src/gaussian_sphere.cpp src/fitting_data.cpp)
target_link_libraries (fitting_library ${PCL_LIBRARIES})

# TESTS
add_executable(cylinder_generation test/cylinder_generation.cpp)
target_link_libraries(cylinder_generation ${PCL_LIBRARIES})

add_executable(sphere_generation test/sphere_generation.cpp)
target_link_libraries(sphere_generation ${PCL_LIBRARIES})

add_executable(cylinder_fitting_test test/cylinder_fitting_test.cpp)
target_link_libraries(cylinder_fitting_test ${PCL_LIBRARIES} fitting_library)

add_executable(sphere_fitting_test test/sphere_fitting_test.cpp)
target_link_libraries(sphere_fitting_test ${PCL_LIBRARIES} fitting_library)

#add_executable(tabletop_scenario_test test/tabletop_scenario_test.cpp)
#target_link_libraries(tabletop_scenario_test fitting_library)




