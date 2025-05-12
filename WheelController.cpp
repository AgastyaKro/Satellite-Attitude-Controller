#include "WheelController.hpp"

WheelController::WheelController(double kp, double ki, double kd) 
                                : kp_(kp), ki_(ki), kd_(kd), accumulated_error_(Eigen::Vector3d::Zero()){}
                    


Eigen::Vector3d WheelController::computeWheelTorques(
    const Eigen::Quaterniond& current,
    const Eigen::Quaterniond& target,
    const Eigen::Vector3d& angular_velocity,
    double dt
){
    // computing orientation error
    Eigen::Quaterniond error_quat = target * current.conjugate();
    if (error_quat.w() < 0)
        error_quat.coeffs() *= -1;
    
    Eigen::Vector3d orientation_error = error_quat.vec();

    // adding the error to accumulated error (for integral of PID)
    accumulated_error_ += orientation_error * dt;

    double max_integral = 0.1; // cap how much error builds up
    if (accumulated_error_.norm() > max_integral)
    accumulated_error_ = accumulated_error_.normalized() * max_integral;

    // PID terms
    Eigen::Vector3d p_term = kp_ * orientation_error;
    Eigen::Vector3d i_term = ki_ * accumulated_error_;
    Eigen::Vector3d d_term = kd_ * angular_velocity;

    return -p_term - i_term - d_term;

}