cmake_minimum_required(VERSION 3.16)
project(kitti_lio CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)

add_executable(kitti_lio
    src/main.cpp
    src/kitti_reader.cpp
    src/imu_reader.cpp
    src/preprocessor.cpp
    src/eskf.cpp
    src/icp.cpp
)

target_include_directories(kitti_lio PRIVATE
    include
    ${EIGEN3_INCLUDE_DIR}
)

target_compile_options(kitti_lio PRIVATE -O3 -march=native)