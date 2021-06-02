cmake_minimum_required(VERSION 3.20)
set(CMAKE_CXX_FLAGS "-std=c++11 -mmacosx-version-min=11.2")
project(imageBasics)

add_executable(imageBasics imageBasics.cpp)
# 链接OpenCV库
find_package(OpenCV REQUIRED)
target_link_libraries(imageBasics ${OpenCV_LIBS})

add_executable(undistortImage undistortImage.cpp)
# 链接OpenCV库
target_link_libraries(undistortImage ${OpenCV_LIBS})
