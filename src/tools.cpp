#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    // Initialize the RMSE vector with all zeros.
    rmse << 0, 0, 0, 0;

    // Check the validity of the following inputs:
    // (1) The estimation vector size should not be zero:
    if (estimations.size() == 0) return rmse;
    // (2) The estimation vector size should equal that of the GT vector:
    if (estimations.size() != ground_truth.size()) return rmse;

    // Accumulate the squared residuals.
    VectorXd acc(4);
    acc << 0, 0, 0, 0;
    for (int i = 0; i < estimations.size(); i++) {
        VectorXd squared = (estimations[i] - ground_truth[i]).array()
                        * (estimations[i] - ground_truth[i]).array();
        acc += squared;
    }
    // Calculate the mean.
    VectorXd mean = acc / estimations.size();
    // Calculate the squared root.
    rmse = mean.array().sqrt();

    // Return the result.
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    MatrixXd Hj(3, 4);
    // Recover state parameters.
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    // Check division by zero.
    if (px == 0 && py == 0) {
        std::cout << "Error: both px and py are zero while trying to"
                  << " calculate the Jacobian." << std::endl;
        return Hj;
    }
    // Prepare calculation.
    float c1 = px * px + py * py;
    float c2 = sqrt(c1);
    float c3 = c1 * c2;
    // Fill in the matrix;
    Hj << px / c2, py / c2, 0, 0,
          -py / c1, px / c1, 0, 0,
          py * (vx * py - vy * px) / c3, px * (vy * px - vx * py) / c3,
          px / c2, py / c2;
    return Hj;
}
