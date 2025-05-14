cmake_minimum_required(VERSION 3.10)
project(SatelliteLQR)

set(CMAKE_CXX_STANDARD 17)

# Include Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(include)

add_executable(satellite_sim
        main.cpp
        src/Satellite.cpp
        src/LQRController.cpp
)

