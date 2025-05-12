#include "WheelController.hpp"

WheelController::WheelController(double kp, double ki, double kd) 
                                : kp_(kp), ki_(ki), kd_(kd), accumulated_error_(Eigen::Vector3d::Zero()){}
                    


Eigen::Vector3d WheelController::computeWheelTorques(
    const Eigen::Quaterniond& current,
    const Eigen::Quaterniond& target,
    const Eigen::Vector3d& angular_velocity,
    double dt
) {
    // Compute rotation difference (shortest rotation from current → target)
    Eigen::Quaterniond error_quat = target * current.conjugate();
    if (error_quat.w() < 0) error_quat.coeffs() *= -1; // fastest path

    Eigen::AngleAxisd axis_angle(error_quat);
    Eigen::Vector3d orientation_error = axis_angle.axis() * axis_angle.angle();

    // Stop accumulating error when near target
    if (axis_angle.angle() < 1e-3 && angular_velocity.norm() < 1e-3) {
        accumulated_error_.setZero();
        return Eigen::Vector3d::Zero(); // No more torque needed
    }

    // Integral accumulation with clamping
    accumulated_error_ += orientation_error * dt;
    double max_integral = 0.1;
    if (accumulated_error_.norm() > max_integral)
        accumulated_error_ = accumulated_error_.normalized() * max_integral;

    // PID terms
    Eigen::Vector3d p_term = kp_ * orientation_error;
    Eigen::Vector3d i_term = ki_ * accumulated_error_;
    Eigen::Vector3d d_term = kd_ * angular_velocity;

    // Negative feedback
    return -p_term - i_term - d_term;
}
