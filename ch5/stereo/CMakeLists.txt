cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_FLAGS "-std=c++11 -O2")
project(stereoVision)
find_package(Pangolin REQUIRED)
# 寻找OpenCV库
find_package(OpenCV REQUIRED)
# 添加头文件
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(stereoVision stereoVision.cpp)
target_link_libraries(stereoVision ${OpenCV_LIBS} ${Pangolin_LIBRARIES})