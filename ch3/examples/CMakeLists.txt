cmake_minimum_required( VERSION 3.0 )
set(CMAKE_CXX_FLAGS "-std=c++11")
include_directories("/usr/local/include/eigen3")
add_executable(cmake-build-debug/coordinateTransform coordinateTransform.cpp)

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
add_executable(cmake-build-debug/plotTrajectory plotTrajectory.cpp)
target_link_libraries(cmake-build-debug/plotTrajectory ${Pangolin_LIBRARIES})
