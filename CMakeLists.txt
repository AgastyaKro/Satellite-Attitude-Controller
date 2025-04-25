cmake_minimum_required(VERSION 3.14)
project(SatelliteSim)

set(CMAKE_CXX_STANDARD 17)

#add eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(SatelliteSim
main.cpp
Satellite.cpp
Controller.cpp
Quaternion.cpp)

target_link_libraries(SatelliteSim Eigen3::Eigen)
