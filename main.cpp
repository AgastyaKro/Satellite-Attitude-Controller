#include "Satellite.hpp"
#include "WheelController.hpp"
#include <iostream>
#include <iomanip>
#include <limits>

int main() {
    Satellite sat;

    // Target: 90 degrees rotated around Y-axis
    Eigen::AngleAxisd rot(M_PI / 2, Eigen::Vector3d::UnitY());
    Eigen::Quaterniond target(rot);
    sat.setTargetOrientation(target);

    // PID tuning
    double kp = 0.25;
    double ki = 0.005;
    double kd = .1;
    WheelController controller(kp, ki, kd);

    const double dt = 0.1;
    const double max_pid_torque = 0.02;
    const double max_brake_torque = 1.5;  // increased

    double prev_error = std::numeric_limits<double>::infinity();

    for (int i = 0; i < 2000; i++) {
        Eigen::Quaterniond q = sat.getOrientation();
        if (q.w() < 0) q.coeffs() *= -1;

        double error = target.angularDistance(q);
        double velocity = sat.getAngularVelocity().norm();

        Eigen::Vector3d torques;

        // Trigger braking early if overshooting near target
        bool near_target = prev_error < 0.6;
        bool overshooting = error > prev_error;



        if (overshooting && near_target) {
            std::cout << "Overshoot near target. Braking.\n";
            Eigen::Vector3d braking = -15.0 * sat.getAngularVelocity();  // stronger braking
            torques = braking.cwiseMax(-max_brake_torque).cwiseMin(max_brake_torque);
            controller.resetIntegral();
        } else {
            // Normal PID torque with light damping
            torques = controller.computeWheelTorques(q, target, sat.getAngularVelocity(), dt);
            // Add damping to suppress oscillations
            double damping_gain = std::clamp(velocity / 5.0, 0.2, 2.5);
            Eigen::Vector3d damping = -damping_gain * sat.getAngularVelocity();
            torques += damping;
            // Clamp total torque to safe limits
            torques = torques.cwiseMax(-max_pid_torque).cwiseMin(max_pid_torque);
        }

        // Abort if spinning too fast
        if (velocity > 10) {
            std::cout << "Too fast. Aborting.\n";
            break;
        }

        // Apply torques to each axis
        for (int axis = 0; axis < 3; axis++) {
            sat.applyWheelTorque(axis, torques[axis], dt);
        }

        // Update orientation
        sat.update(dt);

        // Log state
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "t=" << i * dt << " | Orientation (w,x,y,z): "
                  << q.w() << ", " << q.x() << ", "
                  << q.y() << ", " << q.z() << "\n";
        std::cout << "  [DEBUG] angularDistance: " << error
                  << " | angularVelocity.norm(): " << velocity << "\n";

        prev_error = error;

        // Stop if close to target and nearly stationary
        if (error < 0.05 && velocity < 0.1) {
            std::cout << "Target reached. Stopping.\n";
            break;
        }
    }

    return 0;
}
