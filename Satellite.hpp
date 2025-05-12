#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include "ReactionWheel.hpp"


class Satellite {
    public:
        Satellite();

        void applyWheelTorque(int wheel_index, double torque, double dt);
        void update(double dt); // updates the satellite orientation 

        void setTargetOrientation(const Eigen::Quaterniond& target);
        Eigen::Quaterniond getOrientation() const;
        Eigen::Vector3d getAngularVelocity() const;

    private:

        Eigen::Quaterniond orientation;     // current orientation
        Eigen::Vector3d angular_velocity;   // in rad/s
        Eigen::Quaterniond target_orientation; 
        std::vector<ReactionWheel> wheels; // 1 wheel per axis, x,y,z
        Eigen::Matrix3d inertia_tensor_inv; // moment of inertia tensor inverse

        void applyBodyTorque(const Eigen::Vector3d& torque, double dt);


};