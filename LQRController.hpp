// LQRController.hpp
#pragma once
#include <Eigen/Dense>

class LQRController {
public:
    LQRController(const Eigen::MatrixXd& A,
                  const Eigen::MatrixXd& B,
                  const Eigen::MatrixXd& Q,
                  const Eigen::MatrixXd& R);

    Eigen::Vector3d computeTorque(const Eigen::VectorXd& state_error) const;

private:
    Eigen::MatrixXd gain_matrix_;
    void solveLQR(const Eigen::MatrixXd& A,
                  const Eigen::MatrixXd& B,
                  const Eigen::MatrixXd& Q,
                  const Eigen::MatrixXd& R);
};
