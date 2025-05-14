#include "LQRController.hpp"


LQRController::LQRController(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q,
                             const Eigen::MatrixXd &R) {
    solveLQR(A, B, Q, R);
}

Eigen::Vector3d LQRController::computeTorque(const Eigen::VectorXd& state_error) const {
    return -gain_matrix_ * state_error;
}

void LQRController::solveLQR(const Eigen::MatrixXd& A,
                             const Eigen::MatrixXd& B,
                             const Eigen::MatrixXd& Q,
                             const Eigen::MatrixXd& R) {
    Eigen::MatrixXd P = Q;
    const double tolerance = 1e-8;

    for (int i = 0; i < 1000; ++i) {
        Eigen::MatrixXd BT_P = B.transpose() * P;
        Eigen::MatrixXd P_next = A.transpose() * P * A - A.transpose() * P * B *
                                                         (R + BT_P * B).inverse() * BT_P * A + Q;

        if ((P_next - P).norm() < tolerance)
            break;

        P = P_next;
    }

    gain_matrix_ = (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
}