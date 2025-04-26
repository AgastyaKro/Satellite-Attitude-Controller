#include "Satellite.hpp"
// #include <iostream> // for debugging later w/ std::cout

Satellite::Satellite() : orientation(Eigen::Quaterniond::Identity()), // w,x,y,z = 1,0,0,0
                         angular_velocity(Eigen::Vector3d::Zero()), // x,y,z = 0,0,0
                         inertia(Eigen::Matrix3d::Identity())
                         {}


