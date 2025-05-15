#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>

class WheelController {
public:
    WheelController(double kR, double kOmega);

    Eigen::Vector3d computeWheelTorques(
            const Eigen::Quaterniond& current,
            const Eigen::Quaterniond& target,
            const Eigen::Vector3d& angularVelocity,
            double deltaTime
    );

private:
    double kR_;       // attitude gain
    double kOmega_;   // angular velocity gain
};
