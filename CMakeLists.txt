cmake_minimum_required(VERSION 3.10)
project(SatelliteSim)

set(CMAKE_CXX_STANDARD 17)

# Find Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Include the src/ directory for headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

# Add all source files from src/
add_executable(SatelliteSim
        src/main.cpp
        src/Satellite.cpp
        src/LQRController.cpp
        src/Controller.cpp
        src/Quaternion.cpp
        src/ReactionWheel.cpp
        src/WheelController.cpp
)
