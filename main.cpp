#include "Satellite.hpp"
#include "Controller.hpp"
#include <Eigen/Geometry>
#include <iostream>

int main() {
    Satellite sat; // facing forward no spin 

    // target 90 degress around the Y-axis
    Eigen::AngleAxisd rot(M_PI / 2, Eigen::Vector3d::UnitY());
    Eigen::Quaterniond target(rot);
    sat.setTargetOrientation(target);

    // may need tunning
    double kp = 2.0; 
    double kd = 1.2;
    double ki = 0.5;
    Controller controller(kp, kd, ki);


    const double dt = 0.01;
    for (int i = 0; i < 500; i++){
        Eigen::Vector3d torque = controller.compute_torque(
            sat.getOrientation(),
            target,
            sat.getAngularVelocity(),
            dt
        );
        sat.applyTorque(torque,dt);
        sat.update(dt);

        auto q = sat.getOrientation();
        std::cout << "t=" << i * dt << " | Orientation (w,x,y,z): " << q.w() << ',' << q.x() 
                    << ','  << q.y() << ','  << q.z() << "\n";
    }
}
