#include "Controller.hpp"

Controller::Controller(double kp, double kd, double ki) 
    : kp_(kp), kd_(kd), ki_(ki), accumulated_error_(Eigen::Vector3d::Zero()) {}

Eigen::Vector3d Controller::compute_torque(
    const Eigen::Quaterniond& current_orientation,
    const Eigen::Quaterniond& target_orientation,
    const Eigen::Vector3d& angular_velocity,
    double dt
) {
    Eigen::Quaterniond error_quat = target_orientation * current_orientation.conjugate();
    if (error_quat.w() < 0) 
        error_quat.coeffs() *= -1; // shortest path for quat math

    Eigen::Vector3d orientation_error = error_quat.vec();

    // Accumulate integral of orientation error
    accumulated_error_ += orientation_error * dt;

    // PID 
    Eigen::Vector3d p_term = kp_ * orientation_error;
    Eigen::Vector3d i_term = ki_ * accumulated_error_;
    Eigen::Vector3d d_term = kd_ * angular_velocity;

    return -p_term - d_term - i_term;
}