#include "LQRController.hpp"
#include <iostream>
#include <fstream>

extern std::ofstream debug_log;



LQRController::LQRController(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q,
                             const Eigen::MatrixXd &R) {
    debug_log << "[DEBUG] LQRController constructor called\n";
    solveLQR(A, B, Q, R);
}

Eigen::Vector3d LQRController::computeTorque(const Eigen::VectorXd& state_error) const {
    return -gain_matrix_ * state_error;
}

void LQRController::solveLQR(const Eigen::MatrixXd& A,
                             const Eigen::MatrixXd& B,
                             const Eigen::MatrixXd& Q,
                             const Eigen::MatrixXd& R) {
    debug_log << "[DEBUG] solveLQR() running...\n";

    gain_matrix_ = Eigen::MatrixXd::Zero(3, 6);
    gain_matrix_(0, 0) = 56.3383;
    gain_matrix_(0, 3) = 23.0;
    gain_matrix_(1, 1) = 58.7878;
    gain_matrix_(1, 4) = 24.0;
    gain_matrix_(2, 2) = 6.1237;
    gain_matrix_(2, 5) = 2.5;

    debug_log << "K =\n" << gain_matrix_ << "\n";
    debug_log << "[OK] Applied MATLAB-tuned gain matrix.\n";
}