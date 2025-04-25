#pragma once
#include <Eigen/Dense>

class Satellite {
    public:
        Satellite();

        void applyTorque(const Eigen::Vector3d& torque, double dt); // gets our angular velocity
        void update(double dt); // updates the satellite orientation 

        void setTargetOrientation(const Eigen::Quaterniond& target);
        Eigen::Quaterniond getOrientation() const;
        Eigen::Vector3d getAngularVelocity() const;

    private:

        Eigen::Quaterniond orientation; // current orientation
        Eigen::Vector3d angular_velocity;
        Eigen::Matrix3d inertia;
        Eigen::Quaterniond target_orientation;

        Eigen::Vector3d computeControlTorque(); // PID output







};