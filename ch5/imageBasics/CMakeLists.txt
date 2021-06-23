cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_FLAGS "-std=c++11 -mmacosx-version-min=11.2")
project(imageBasics)

# 添加Eigen头文件
include_directories("/usr/local/include/eigen3")
add_executable(imageBasics imageBasics.cpp)
# 链接OpenCV库
find_package(OpenCV REQUIRED)
target_link_libraries(imageBasics ${OpenCV_LIBS})

add_executable(undistortImage undistortImage.cpp)
# 链接OpenCV库
target_link_libraries(undistortImage ${OpenCV_LIBS})

add_executable(cvundistort cvundistort.cpp)
# 链接OpenCV库
target_link_libraries(cvundistort ${OpenCV_LIBS})

add_executable(cmpCVandVINs cmpCVandVINs.cpp)
# 链接OpenCV库
target_link_libraries(cmpCVandVINs ${OpenCV_LIBS})

add_executable(cmpVINS cmpVINS.cpp)
# 链接OpenCV库
target_link_libraries(cmpVINS ${OpenCV_LIBS})
