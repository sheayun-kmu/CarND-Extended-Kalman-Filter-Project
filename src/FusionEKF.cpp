#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);

    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
                0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;

    // Vector and matrices for initializing EKF
    VectorXd x_init = VectorXd(4);
    MatrixXd P_init = MatrixXd(4, 4);
    MatrixXd F_init = MatrixXd(4, 4);
    MatrixXd H_init = MatrixXd(2, 4);
    MatrixXd Q_init = MatrixXd(4, 4);

    // Initialize state covariance matrix
    P_init << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0,
              0, 0, 0, 1000;
    // System model - dt not yet taken into account
    F_init << 1, 0, 1, 0,
              0, 1, 0, 1,
              0, 0, 1, 0,
              0, 0, 0, 1;
    // transformation from state variables to measurement
    H_init << 1, 0, 0, 0,
              0, 1, 0, 0;
    // Covariance matrix of process noise - dt not yet taken into account
    Q_init << 1, 0, 1, 0,
              0, 1, 0, 1,
              1, 0, 1, 0,
              0, 1, 0, 1;

    ekf_.Init(x_init, P_init, F_init, H_init, R_laser_, Q_init);

    // Set the process noise constants
    noise_ax_ = 9.0;
    noise_ay_ = 9.0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    /**
     * Initialization
     */
    if (!is_initialized_) {
        // first measurement
        cout << "EKF: " << endl;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            // Convert radar data in polar to cartesian coordinates.
            // (Note rho_dot from the very first measurements is not used.)
            float rho = measurement_pack.raw_measurements_[0];
            float phi = measurement_pack.raw_measurements_[1];
            // float rho_dot = measurement_pack.raw_measurements_[2];
            // Initialize state variables.
            float px = rho * cos(phi);
            float py = rho * sin(phi);
            ekf_.x_ << px, py, 0, 0;
            // Record timestamp.
            previous_timestamp_ = measurement_pack.timestamp_;
            // Output debugging message.
            cout << "From radar" << endl << ekf_.x_ << endl;
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            // Initialize state variables.
            float px = measurement_pack.raw_measurements_[0];
            float py = measurement_pack.raw_measurements_[1];
            ekf_.x_ << px, py, 0, 0;
            // Record timestamp.
            previous_timestamp_ = measurement_pack.timestamp_;
            // Output debugging message.
            cout << "From laser" << endl << ekf_.x_ << endl;
        }

        // Done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    /**
     * Prediction
     */

    // Update the state transition matrix (time measured in seconds).
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;
    ekf_.F_(0, 2) = ekf_.F_(1, 3) = dt;
    // Set the process noise covariance.
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;

    ekf_.Q_ << dt4 * noise_ax_ / 4.0, 0, dt3 * noise_ax_ / 2.0, 0,
               0, dt4 * noise_ay_ / 4.0, 0, dt3 * noise_ay_ / 2.0,
               dt3 * noise_ax_ / 2.0, 0, dt2 * noise_ax_, 0,
               0, dt3 * noise_ay_ / 2.0, 0, dt2 * noise_ay_;

    ekf_.Predict();

    /**
     * Update
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        VectorXd z = VectorXd(3);
        z << measurement_pack.raw_measurements_[0],
             measurement_pack.raw_measurements_[1],
             measurement_pack.raw_measurements_[2];
        // Setup measurement noise covariance.
        ekf_.R_ = R_radar_;
        ekf_.UpdateEKF(z);
    } else {
        // Laser updates
        VectorXd z = VectorXd(2);
        z << measurement_pack.raw_measurements_[0],
             measurement_pack.raw_measurements_[1];
        // Setup measurement noise covariance.
        ekf_.R_ = R_laser_;
        ekf_.Update(z);
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
