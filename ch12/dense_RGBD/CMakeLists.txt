cmake_minimum_required(VERSION 3.0)
project(mapping)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++14 -O2  -mmacosx-version-min=11.2")

# opencv 
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# eigen 
include_directories("/usr/local/include/eigen3/")
include_directories("/opt/homebrew/Cellar/")
include_directories("/opt/homebrew/Cellar/vtk@8.2/8.2.0_6/include/vtk-8.2")


set(OCTOMAP_INCLUDE_DIRS "/usr/local/Cellar/Cellar/octomap/1.9.7/include")
set(OCTOMAP_LIBRARIES "/usr/local/Cellar/octomap/1.9.7/lib/liboctomap.dylib")

set(PCL_INCLUDE_DIRS "/usr/local/Cellar/pcl/1.11.1_8/include/pcl-1.11/")
set(PCL_LIBRARIES "/usr/local/Cellar/pcl/1.11.1_8/lib")

# pcl 
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# octomap 
#find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

add_executable(pointcloud_mapping pointcloud_mapping.cpp)
target_link_libraries(pointcloud_mapping ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(octomap_mapping octomap_mapping.cpp)
target_link_libraries(octomap_mapping ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(surfel_mapping surfel_mapping.cpp)
target_link_libraries(surfel_mapping ${OpenCV_LIBS} ${PCL_LIBRARIES})
