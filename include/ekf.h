#pragma once
#include <Eigen/Dense>

class EKF {
public:
    EKF(int state_dim, int meas_dim);

    void init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0,
              const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);

    void predict(double dt);
    void update(const Eigen::VectorXd& z, const std::string& sensor_type);

    Eigen::VectorXd getState() const { return x_; }
    Eigen::MatrixXd getCovariance() const { return P_; }

private:
    int state_dim_, meas_dim_;
    Eigen::VectorXd x_;   // [x, y, vx, vy]
    Eigen::MatrixXd P_;   // covariance
    Eigen::MatrixXd Q_;   // process noise
    Eigen::MatrixXd R_;   // measurement noise

    Eigen::MatrixXd computeJacobian();
};
