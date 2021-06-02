cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_FLAGS "-std=c++11 -mmacosx-version-min=11.2")
project(useSophus)

# 为使用 sophus，需要使用find_package命令找到它
find_package(sophus REQUIRED)

# Eigen
include_directories("/usr/local/include/eigen3")
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
add_executable(useSophus useSophus.cpp)
target_link_libraries(useSophus ${Sophus_LIBS})

add_subdirectory(example)
