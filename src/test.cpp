#include <iostream>
#include <fstream>
#include <vector>
#include "measurement_package.h"
#include "FusionEKF.h"

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
    while (getline(in_file, line) && i <= 3) {
        MeasurementPackage meas_package;
        istringstream iss(line);
        string sensor_type;
        iss >> sensor_type;
        int64_t timestamp;
        if (sensor_type.compare("L") == 0) { // laser measurement
            // Read measurements
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float x, y;
            iss >> x;
            iss >> y;
            meas_package.raw_measurements_ << x, y;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
            i++;
        } else if (sensor_type.compare("R") == 0) { // radar measurement
            // Skip radar measurements
            continue;
        }
    }

    FusionEKF fusionEKF;

    for (auto& x : measurement_pack_list) {
        fusionEKF.ProcessMeasurement(x);
    }
}
