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
    Eigen::VectorXd error(6);

    Eigen::Quaterniond q_err = target_orientation_ * orientation_.conjugate();
    q_err.normalize();

    if (q_err.w() < 0) q_err.coeffs() *= -1;
    Eigen::AngleAxisd aa_err(q_err);
    Eigen::Vector3d orientation_error = aa_err.angle() * aa_err.axis();

    // Reversed order: [ω; orientation_error]
    error.head<3>() = angular_velocity_;
    error.tail<3>() = orientation_error;

    return error;
}


void Satellite::update(double dt, const Eigen::Vector3d& torque) {
    // Angular acceleration: α = I⁻¹ * τ
    // Apply damping torque: τ_damp = -D * ω
    Eigen::Matrix3d damping = 0.05 * Eigen::Matrix3d::Identity();
    Eigen::Vector3d angular_accel = inverse_inertia_matrix_ * (torque - damping * angular_velocity_);

    angular_velocity_ += angular_accel * dt;

    // Quaternion integration: dq = Δθ × orientation
    Eigen::Vector3d delta_theta = angular_velocity_ * dt;
    double angle = delta_theta.norm();

    if (angle > 1e-8) {
        Eigen::Vector3d axis = delta_theta.normalized();
        Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, axis));
        orientation_ = (dq * orientation_).normalized(); // apply rotation
    }
}


const Eigen::Matrix3d& Satellite::getInverseInertiaMatrix() const {
    return inverse_inertia_matrix_;
}

