#include <iostream>
#include <fstream>
#include <vector>
#include "measurement_package.h"
#include "FusionEKF.h"
#include "tools.h"

using std::cout;
using std::cerr;
using std::endl;
using std::ifstream;
using std::istringstream;
using std::string;
using std::vector;
using Eigen::VectorXd;

int main() {
    vector<MeasurementPackage> measurement_pack_list;
    FusionEKF fusionEKF;
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;
    vector<VectorXd> rmse_list;
    Tools tools;

    // Hardcoded input file with laser and radar measurements
    string in_file_name = "../data/obj_pose-laser-radar-synthetic-input.txt";
    ifstream in_file(in_file_name.c_str(), ifstream::in);
    if (!in_file.is_open()) {
        cerr << "Cannot open input file: " << in_file_name << endl;
        exit(-1);
    }

    string line;
    // Set i to get only a designated number of lines are processed.
    int i = 0;
    int n = 0;
    while (getline(in_file, line) && (n == 0 || i < n)) {
        MeasurementPackage meas_package;
        istringstream iss(line);
        string sensor_type;
        iss >> sensor_type;
        int64_t timestamp;
        // Accumulate to measurement_pack_list
        cout << "Measurement #" << i + 1 << ", type: " << sensor_type << endl;
        if (sensor_type.compare("L") == 0) { // laser measurement
            // Read measurements
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float x, y;
            iss >> x >> y;
            meas_package.raw_measurements_ << x, y;
        } else if (sensor_type.compare("R") == 0) { // radar measurement
            // Read measurements
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = VectorXd(3);
            float rho, theta, rho_dot;
            iss >> rho >> theta >> rho_dot;
            meas_package.raw_measurements_ << rho, theta, rho_dot;
        }
        iss >> timestamp;
        meas_package.timestamp_ = timestamp;
        measurement_pack_list.push_back(meas_package);

        // Accumulate to ground_truth
        float x_gt, y_gt, vx_gt, vy_gt;
        iss >> x_gt >> y_gt >> vx_gt >> vy_gt;
        VectorXd gt_values(4);
        gt_values << x_gt, y_gt, vx_gt, vy_gt;
        ground_truth.push_back(gt_values);

        // Process the measurement
        fusionEKF.ProcessMeasurement(meas_package);
        VectorXd estimate(4);
        double px = fusionEKF.ekf_.x_[0];
        double py = fusionEKF.ekf_.x_[1];
        double vx = fusionEKF.ekf_.x_[2];
        double vy = fusionEKF.ekf_.x_[3];
        estimate << px, py, vx, vy;
        estimations.push_back(estimate);

        // Calculate RMSE
        VectorXd rmse = tools.CalculateRMSE(estimations, ground_truth);
        cout << "RMSE:" << endl << rmse << endl;
        rmse_list.push_back(rmse);

        i++;
    }
    in_file.close();

    // Post-mortem analysis of RMSE
    float max_rmse_px = 0, max_rmse_py = 0;
    float max_rmse_vx = 0, max_rmse_vy = 0;
    int max_rmse_px_index = 0, max_rmse_py_index = 0;
    int max_rmse_vx_index = 0, max_rmse_vy_index = 0;
    for (int i = 0; i < rmse_list.size(); i++) {
        VectorXd x = rmse_list[i];
        float rmse_px = x[0];
        float rmse_py = x[1];
        float rmse_vx = x[2];
        float rmse_vy = x[3];
        if (rmse_px > max_rmse_px) {
            max_rmse_px = rmse_px; max_rmse_px_index = i;
        }
        if (rmse_py > max_rmse_py) {
            max_rmse_py = rmse_py; max_rmse_py_index = i;
        }
        if (rmse_vx > max_rmse_vx) {
            max_rmse_vx = rmse_vx; max_rmse_vx_index = i;
        }
        if (rmse_vy > max_rmse_vy) {
            max_rmse_vy = rmse_vy; max_rmse_vy_index = i;
        }
    }
    cout << "RMSE Summary" << endl;
    cout << "Max RMSE (px) = " << max_rmse_px
         << ", at " << max_rmse_px_index + 1 << endl;
    cout << "Max RMSE (py) = " << max_rmse_py
         << ", at " << max_rmse_py_index + 1 << endl;
    cout << "Max RMSE (vx) = " << max_rmse_vx
         << ", at " << max_rmse_vx_index + 1 << endl;
    cout << "Max RMSE (vy) = " << max_rmse_vy
         << ", at " << max_rmse_vy_index + 1 << endl;
}
