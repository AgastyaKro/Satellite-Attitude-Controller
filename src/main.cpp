#include "Satellite.hpp"
#include "LQRController.hpp"
#include <fstream>
#include <iostream>
#include <iomanip> // for std::setprecision

std::ofstream debug_log("SatelliteDebug.csv");

int main() {
    Satellite satellite;

    // Target orientation: 90 degrees about Y axis
    Eigen::Quaterniond target_orientation(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()));
    satellite.setTargetOrientation(target_orientation);

    // Inverse inertia matrix used for B matrix
    Eigen::Matrix3d B_phys = satellite.getInverseInertiaMatrix();

    // Linearized system dynamics around zero error state
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6, 3);

    // θ_dot = -ω
    A.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

    // ω_dot = -D*ω + B*u
    A.block<3, 3>(3, 3) = -0.05 * Eigen::Matrix3d::Identity();
    B.block<3, 3>(3, 0) = B_phys;


    debug_log << "B matrix =\n" << B << "\n";

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, 6);
    Q.block<3,3>(0,0) = 10.0 * Eigen::Matrix3d::Identity(); // orientation error
    Q.block<3,3>(3,3) = 1.0 * Eigen::Matrix3d::Identity();  // angular velocity

    Eigen::MatrixXd R = 0.01 * Eigen::Matrix3d::Identity(); // torque penalty


    LQRController controller(A, B, Q, R);

    const double timestep = 0.01;
    const int steps = 1000;
    std::ofstream log_file("satellite_output.csv");
    log_file << "time,qw,qx,qy,qz,wx,wy,wz\n";

    for (int i = 0; i < steps; ++i) {
        double time = i * timestep;

        Eigen::VectorXd state_error = satellite.computeStateError();
        Eigen::Vector3d control_torque = controller.computeTorque(state_error);
        satellite.update(timestep, control_torque);

        const Eigen::Quaterniond& q = satellite.getOrientation();
        const Eigen::Vector3d& w = satellite.getAngularVelocity();

        if (i % 5 == 0) {
            debug_log << std::fixed << std::setprecision(5)
                      << "t=" << time << ", q=[" << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << "], "
                      << "w=[" << w.x() << "," << w.y() << "," << w.z() << "], "
                      << "torque=[" << control_torque.transpose() << "], "
                      << "error=[" << state_error.transpose() << "]\n";
        }

        // Optional: early termination if converged
        if (state_error.norm() < 1e-3 && w.norm() < 1e-3) {
            std::cout << "[INFO] Stabilized at time " << time << "s\n";
            break;
        }
    }

    log_file.close();
    debug_log.close();
    return 0;
}
