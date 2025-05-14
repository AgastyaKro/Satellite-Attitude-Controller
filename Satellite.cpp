// Satellite.cpp
#include "Satellite.hpp"
#include <Eigen/Geometry>

Satellite::Satellite() {
    orientation_ = Eigen::Quaterniond::Identity();
    angular_velocity_.setZero();

    inertia_matrix_ = Eigen::Matrix3d::Identity();
    inertia_matrix_(0, 0) = 230;
    inertia_matrix_(1, 1) = 240;
    inertia_matrix_(2, 2) = 25;
    inverse_inertia_matrix_ = inertia_matrix_.inverse();
}

void Satellite::setTargetOrientation(const Eigen::Quaterniond& target_orientation) {
    target_orientation_ = target_orientation.normalized();
}

const Eigen::Quaterniond& Satellite::getOrientation() const {
    return orientation_;
}

const Eigen::Vector3d& Satellite::getAngularVelocity() const {
    return angular_velocity_;
}

Eigen::VectorXd Satellite::computeStateError() const {
    Eigen::Quaterniond error_quaternion = target_orientation_ * orientation_.inverse();
    if (error_quaternion.w() < 0) error_quaternion.coeffs() *= -1;

    Eigen::AngleAxisd error_axis_angle(error_quaternion);
    Eigen::VectorXd state_error(6);
    state_error.segment<3>(0) = error_axis_angle.angle() * error_axis_angle.axis();
    state_error.segment<3>(3) = angular_velocity_;
    return state_error;
}

void Satellite::update(double timestep, const Eigen::Vector3d& control_torque) {
    Eigen::Vector3d angular_acceleration = inverse_inertia_matrix_ *
                                           (control_torque - angular_velocity_.cross(inertia_matrix_ * angular_velocity_));

    angular_velocity_ += angular_acceleration * timestep;

    Eigen::Vector3d delta_theta = angular_velocity_ * timestep;
    double angle = delta_theta.norm();
    Eigen::Quaterniond delta_quaternion = angle < 1e-8 ? Eigen::Quaterniond::Identity()
                                                       : Eigen::Quaterniond(Eigen::AngleAxisd(angle, delta_theta.normalized()));

    orientation_ = (delta_quaternion * orientation_).normalized();
}

