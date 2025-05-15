#include "WheelController.hpp"
#include <cmath>

WheelController::WheelController(double kR, double kOmega)
        : kR_(kR), kOmega_(kOmega) {}

Eigen::Vector3d WheelController::computeWheelTorques(
        const Eigen::Quaterniond& current,
        const Eigen::Quaterniond& target,
        const Eigen::Vector3d& angularVelocity,
        double /* deltaTime */
) {
    // 1. Compute relative rotation error: Q_err = Q_target * Q_current.inverse()
    Eigen::Quaterniond qErr = target * current.conjugate();
    if (qErr.w() < 0) qErr.coeffs() *= -1;

    // 2. Convert to axis-angle error vector
    double angle = 2.0 * std::acos(std::clamp(qErr.w(), -1.0, 1.0));
    double sin_half = std::sqrt(1.0 - qErr.w() * qErr.w());
    Eigen::Vector3d axis;
    if (sin_half < 1e-6)
        axis = Eigen::Vector3d::Zero();
    else
        axis = qErr.vec() / sin_half;
    Eigen::Vector3d eR = axis * angle;

    // 3. Angular velocity error: e_omega = ω (assuming desired ω = 0)
    Eigen::Vector3d eOmega = angularVelocity;

    // 4. Control torque
    Eigen::Vector3d torque = -kR_ * eR - kOmega_ * eOmega;

    const double maxTorque = 2.0;
    if (torque.norm() > maxTorque)
        torque = torque.normalized() * maxTorque;

    return torque;
}
