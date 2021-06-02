cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_FLAGS "-std=c++11 -mmacosx-version-min=11.2")
project(joinMap)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

include_directories( "/usr/local/include/eigen3" )

add_executable(joinMap joinMap.cpp)
target_link_libraries(joinMap ${OpenCV_LIBS} ${Pangolin_LIBRARIES})