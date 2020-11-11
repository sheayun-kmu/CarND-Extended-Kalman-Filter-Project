#include <cmath>
#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd K = P_ * H_.transpose() * S.inverse();
    // Calculate new estimates.
    x_ = x_ + K * (z - H_ * x_);
    P_ = P_ - K * H_ * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    Tools tools;
    // Calculate Jacobian from the predicted state and use it.
    MatrixXd Hj = tools.CalculateJacobian(x_);
    MatrixXd S = Hj * P_ * Hj.transpose() + R_;
    MatrixXd K = P_ * Hj.transpose() * S.inverse();
    // Estimate y = z - h(x')
    float px = x_[0], py = x_[1], vx = x_[2], vy = x_[3];
    float rho = sqrt(px * px + py * py);
    float phi = atan2(py, px);
    float rho_dot = (px * vx + py * vy) / rho;
    VectorXd hx = VectorXd(3);
    hx << rho, phi, rho_dot;
    VectorXd y = z - hx;
    // Normalize phi so that it is between -pi and pi.
    phi = y[1];
    float sign = phi > 0 ? 1.0 : -1.0;
    float mag = abs(phi);
    while (mag > M_PI) mag -= M_PI;
    y[1] = sign * mag;
    // Calculate new estimates.
    x_ = x_ + K * y;
    P_ = P_ - K * Hj * P_;
}
