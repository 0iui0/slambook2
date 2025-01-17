cmake_minimum_required(VERSION 3.0)
project(pose_graph)

set(CMAKE_CXX_FLAGS "-std=c++14 -O2")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# Eigen
include_directories("/usr/local/include")
include_directories("/usr/local/include/eigen3")

# sophus 
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# g2o 
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

add_executable(pose_graph_g2o_SE3 pose_graph_g2o_SE3.cpp)
target_link_libraries(pose_graph_g2o_SE3
        ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} /usr/local/lib/libg2o_types_slam3d.dylib ${CHOLMOD_LIBRARIES}
        )

add_executable(pose_graph_g2o_lie pose_graph_g2o_lie_algebra.cpp)
target_link_libraries(pose_graph_g2o_lie
        ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY}
        ${CHOLMOD_LIBRARIES}
        ${Sophus_LIBRARIES}
        )

