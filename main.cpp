
#include "Satellite.hpp"
#include "WheelController.hpp"
#include <iostream>
#include <iomanip>


int main() {
  Satellite sat;

  // target is 90 degress rotated around the y-axis
  Eigen::AngleAxisd rot(M_PI / 2, Eigen::Vector3d::UnitY());
  Eigen::Quaterniond target(rot);
  sat.setTargetOrientation(target);

  // PID gains
  double kp = .4;
  double ki = 0.01;
  double kd = .6;
  WheelController controller(kp,ki,kd);

  const double dt = .1;

  for (int i = 0; i < 2000; i++){
    // get torque for each reaction wheel axis
    Eigen::Vector3d torques = controller.computeWheelTorques(
        sat.getOrientation(), target, sat.getAngularVelocity(), dt
    );

    double max_torque = 0.05; // can only rotate so much
    torques = torques.cwiseMax(-max_torque).cwiseMin(max_torque);

    // apply each torque to corresponding wheel axis

    for (int axis = 0; axis < 3; axis++){
        sat.applyWheelTorque(axis, torques[axis], dt);
    }

    // update satellite's orientation based on new angular vel from updated wheel torques
    sat.update(dt);

    // Log orientation quaternion
    Eigen::Quaternion q = sat.getOrientation();
    if (q.w() < 0) q.coeffs() *= -1;  

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "t=" << i * dt << " | Orientation (w,x,y,z): "
              << q.w() << ", " << q.x() << ", "
              << q.y() << ", " << q.z() << "\n";

    std::cout << "  [DEBUG] angularDistance: " << target.angularDistance(q)
          << " | angularVelocity.norm(): " << sat.getAngularVelocity().norm() << "\n";

    if (target.angularDistance(q) < 1e-1 && sat.getAngularVelocity().norm() < 1e-1) {
    std::cout << "Target reached. Stopping.\n";
    break;
}

  }
  return 0;

}


