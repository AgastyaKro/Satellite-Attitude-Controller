#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>

class WheelController{

    public:
        WheelController(double kp, double ki, double kd);

    Eigen::Vector3d computeWheelTorques(
        const Eigen::Quaterniond& current,
        const Eigen::Quaterniond& target,
        const Eigen::Vector3d& angular_velocity,
        double dt
    );

    void resetIntegral();


private:
        double kp_;
        double ki_;
        double kd_;
        Eigen::Vector3d accumulated_error_;

};