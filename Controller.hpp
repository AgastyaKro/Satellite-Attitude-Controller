#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>

class Controller {
    public:
        Controller(double kp, double kd, double ki);

        Eigen::Vector3d compute_torque(
            const Eigen::Quaterniond& current_orientation,
            const Eigen::Quaterniond& target_orientation,
            const Eigen::Vector3d& angular_velocity,
            double dt
        );

    private:
        double kp_;
        double kd_;
        double ki_;

        Eigen::Vector3d accumulated_error_;
};