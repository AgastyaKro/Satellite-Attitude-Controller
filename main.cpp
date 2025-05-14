#include "Satellite.hpp"
#include "WheelController.hpp"
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <iomanip>

int main() {
    Satellite sat;

    // Target is 90 degrees around Y axis
    Eigen::Quaterniond target(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()));
    sat.setTargetOrientation(target);

    WheelController controller(/*kR=*/6.0, /*kOmega=*/4.0);
    const double dt = 1.0;

    std::ofstream out("output.csv");
    out << "time,w,x,y,z,wx,wy,wz,angleError\n";

    for (double t = 0.0; t <= 150.0; t += dt) {
        Eigen::Quaterniond current = sat.getOrientation();
        Eigen::Vector3d omega = sat.getAngularVelocity();

        Eigen::Vector3d torques = controller.computeWheelTorques(current, target, omega, dt);
        for (int i = 0; i < 3; ++i)
            sat.applyWheelTorque(i, torques[i], dt);

        sat.update(dt);

        Eigen::Quaterniond errQ = target * current.conjugate();
        if (errQ.w() < 0.0) errQ.coeffs() *= -1;
        double angleError = 2.0 * std::acos(std::clamp(errQ.w(), -1.0, 1.0));

        if (static_cast<int>(t) % 10 == 0) {
            std::cout << std::fixed << std::setprecision(1)
                      << "t=" << t
                      << "  orient=(w:" << current.w()
                      << ", x:" << current.x()
                      << ", y:" << current.y()
                      << ", z:" << current.z()
                      << ")  angVel=(x:" << omega[0]
                      << ", y:" << omega[1]
                      << ", z:" << omega[2]
                      << ")  angleError=" << angleError << "\n";
        }

        out << std::fixed << std::setprecision(5)
            << t << "," << current.w() << "," << current.x() << "," << current.y() << "," << current.z() << ","
            << omega[0] << "," << omega[1] << "," << omega[2] << ","
            << angleError << "\n";
    }

    out.close();
    std::system("cp output.csv .."); // optional
    return 0;
}
