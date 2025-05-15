// Satellite.hpp
#ifndef SATELLITE_HPP
#define SATELLITE_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "ReactionWheel.hpp"
#include <array>

class Satellite {
public:
    Satellite();

    void update(double dt, const Eigen::Vector3d& controlTorque);
    const Eigen::Quaterniond& getOrientation() const;
    const Eigen::Vector3d& getAngularVelocity() const;
    void setTargetOrientation(const Eigen::Quaterniond& target_orientation);

    Eigen::VectorXd computeStateError() const;

    const Eigen::Matrix3d& getInverseInertiaMatrix() const;



private:
    Eigen::Quaterniond orientation_;      // Current orientation as quaternion
    Eigen::Vector3d angular_velocity_;    // Angular velocity in body frame
    Eigen::Quaterniond target_orientation_;

    Eigen::Matrix3d inertia_matrix_;
    Eigen::Matrix3d inverse_inertia_matrix_;
};

#endif // SATELLITE_HPP
