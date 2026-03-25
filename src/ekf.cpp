#include "ekf.h"
#include <cmath>

EKF::EKF(int state_dim, int meas_dim)
    : state_dim_(state_dim), meas_dim_(meas_dim) {}

void EKF::init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0,
               const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
    x_ = x0;
    P_ = P0;
    Q_ = Q;
    R_ = R;
}

void EKF::predict(double dt) {
    // State transition: constant velocity model
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    F(0, 2) = dt;
    F(1, 3) = dt;
    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q_;
}

Eigen::MatrixXd EKF::computeJacobian() {
    // Jacobian of radar measurement function h(x)
    double px = x_(0), py = x_(1), vx = x_(2), vy = x_(3);
    double rho = sqrt(px*px + py*py);
    double rho2 = rho * rho;
    double rho3 = rho * rho2;

    Eigen::MatrixXd Hj(3, 4);
    if (fabs(rho) < 1e-6) return Hj.setZero();

    Hj << px/rho,           py/rho,           0,      0,
         -py/rho2,          px/rho2,          0,      0,
          py*(vx*py-vy*px)/rho3, px*(vy*px-vx*py)/rho3, px/rho, py/rho;
    return Hj;
}

void EKF::update(const Eigen::VectorXd& z, const std::string& sensor_type) {
    Eigen::MatrixXd H;
    Eigen::VectorXd y;

    if (sensor_type == "lidar") {
        // Linear measurement: directly observe x, y
        H = Eigen::MatrixXd::Zero(2, 4);
        H(0, 0) = 1;
        H(1, 1) = 1;
        y = z - H * x_;
    } else {
        // Radar: nonlinear measurement (rho, phi, rho_dot)
        H = computeJacobian();
        double px = x_(0), py = x_(1), vx = x_(2), vy = x_(3);
        double rho = sqrt(px*px + py*py);
        double phi = atan2(py, px);
        double rho_dot = (px*vx + py*vy) / std::max(rho, 1e-6);
        Eigen::VectorXd h(3);
        h << rho, phi, rho_dot;
        y = z - h;
        // Normalize angle
        while (y(1) >  M_PI) y(1) -= 2*M_PI;
        while (y(1) < -M_PI) y(1) += 2*M_PI;
    }

    Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    x_ = x_ + K * y;
    P_ = (Eigen::MatrixXd::Identity(state_dim_, state_dim_) - K * H) * P_;
}
