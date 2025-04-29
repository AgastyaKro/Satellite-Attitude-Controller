#include "Satellite.hpp"
#include <Eigen/Geometry>
#include <iostream>

int main() {
    Satellite sat; // facing forward no spin 

    // target 90 degress around the Y-axis
    Eigen::AngleAxisd rot(M_PI / 2, Eigen::Vector3d::UnitY());
    Eigen::Quaterniond target(rot);
    sat.setTargetOrientation(target);

    const double dt = 0.01;
    for (int i = 0; i < 500; i++){
        Eigen::Vector3d torque = Eigen::Vector3d(0, 0.01, 0);
        sat.applyTorque(torque,dt);
        sat.update(dt);

        auto q = sat.getOrientation();
        std::cout << "t=" << i * dt << " | Orientation (w,x,y,z): " << q.w() << ',' << q.x() 
                    << ','  << q.y() << ','  << q.z() << "\n";
    }
}
