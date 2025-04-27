#include "Satellite.hpp"
// #include <iostream> // for debugging later w/ std::cout

Satellite::Satellite() : orientation(Eigen::Quaterniond::Identity()), // w,x,y,z = 1,0,0,0
                         angular_velocity(Eigen::Vector3d::Zero()), // x,y,z = 0,0,0
                         inertia(Eigen::Matrix3d::Identity()) // 3 by 3 matrix, w/ diag as 1,1,1 else 0
                                                              // it's equally hard to rotate across x,y,z axis
                         {}


void Satellite::applyTorque(const Eigen::Vector3d& torque, double dt) { // torque + how long it was applied for
        Eigen::Vector3d angular_acceleration = inertia.inverse() * torque; // T/I
        angular_velocity += angular_acceleration * dt;
    }