#pragma once
#include <Eigen/Dense>

class Satellite {
    public:
        Satellite();

        void applyTorque(const Eigen::Vector3d& torque, double dt);
        



};