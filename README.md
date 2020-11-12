# Extended Kalman Filter Project

### Udacity Self-Driving Car Engineer Nanodegree Program

--

The goals of this project are the following:

* Write (in C++) classes implementing the Extended Kalman Filter estimating 2-D position and velocity of a moving object based on lidar and radar measurements.
* Validate the estimation using a [simulator](https://github.com/udacity/self-driving-car-sim/releases).
* Verify the filter's performance based on RMSE (root mean squared error).

[//]: # (Image References)
[overview]: ./images/overview.png
[result-1]: ./images/dataset-1-result.png
[result-2]: ./images/dataset-2-result.png

## Rubric Points

The rubric points considered in the project can be found [here](https://review.udacity.com/#!/rubrics/748/view).

## Program Build & Execution

In the project root directory, execute (in a shell) the following sequence of commands.

```
# mkdir build && cd build
# cmake .. && make
```

The program is built in the form of an API server that listens to the port 4567, which can be invoked by the following command (in the `build` subdirectory):

```
# ./ExtendedKF
```

## Program Behavior

The simulator sends requests with parameters being lidar and radar measurements. The lidar measurement and radar measurement have different data format. The data format is shown below:

```
#L(for laser) meas_px meas_py timestamp gt_px gt_py gt_vx gt_vy
#R(for radar) meas_rho meas_phi meas_rho_dot timestamp gt_px gt_py gt_vx gt_vy
```

For example, the following two lines are excerpts from the beginning of the [input file](./data/obj_pose-laser-radar-synthetic-input.txt), which is known to correspond to the simulator's measurement sequences:

```
L	3.122427e-01	5.803398e-01	1477010443000000	6.000000e-01	6.000000e-01	5.199937e+00	0	0	6.911322e-03
R	1.014892e+00	5.543292e-01	4.892807e+00	1477010443050000	8.599968e-01	6.000449e-01	5.199747e+00	1.796856e-03	3.455661e-04	1.382155e-02
```

The server responds to requests from the client (i.e., the simulator) with (1) the position estimated by the Extended Kalman Filter, and (2) the error defined as RMSE. The simulator, upon receiving the response, plots the estimated positions accordingly.

## Program Structure

The server program consists of (1) four C++ source files, (2) four header files, and (3) `Eigen` library. The following is the list of C++ source files:

```
main.cpp
FusionEKF.cpp
kalman_filter.cpp
tools.cpp
```

The main driver file (`main.cpp`) implements the server using [uWebSocketIO](https://github.com/uWebSockets/uWebSockets). In processing requests, it uses `class FusionEKF` (implemented in `FusionEKF.cpp`) to process measurement data received from the client (by the `ProcessMeasurement()` method), which in turn performs the EKF's prediction and measurement update steps, where the latter is different for ladar measurements and radar measurements cases. These two measurement updates are implemented by `class KalmanFilter` (in `kalman_filter.cpp`), by the methods `Update()` and `UpdateEKF()`, respectively, whereas the same method `Predict()` is used for both types of measurement. Finally, `class Tools` (implemented in `tools.cpp`) provides methods for calculatioing RMSE and Jacobian.

## Test Driver

A file `test.cpp` that was not included in the original repository is newly written to test the EKF against the input data contained in the test data file [`./data/obj_pose-laser-radar-synthetic-input.txt`](./data/obj_pose-laser-radar-synthetic-input.txt). It reads each line from the input data file and feeds them to the EKF as the simulator would. Estimations and RMSE are collected and displayed for debugging purposes.

## Implementation of the Extended Kalman Filter

The EKF implemented in the project follows the procedures explained in the lectures, which is illustrated by the following figure:

![Overall Procedure][overview]

The steps taken by the filter are:

* Upon receiving the initial (very first) measurement data, the filter initializes its own state and covariance matrices. Initialization using lidar measurement data is straightforward, while we must calculate `px` and `py` by trigonometric function in case of radar measurement data.
* For subsequent measurement data, two steps are taken: (1) prediction of state variables and covariance matrix, and (2) update using the current measurements. The lidar measurement gives new values for `px` and `py` directlry, which makes the implementation straightforward, while the radar measurement gives `rho`, `phi`, and `rho_dot` (expressed in polar coordinates, along with a velocity component). Therefore, the update operation is divided into (1) standard KF update for the lidar data, and (2) EKF update for the radar data.

In the prediction step, a linear transform is used for both lidar and radar measurements. It is implemented in `KalmanFilter::Predict()` in a straightforward manner:

```
void KalmanFilter::Predict() {
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}
```

where matrices `F_` and `Q_` are updated (based on the elapsed time between measurement and the given process noise covariance) before each time this method is called.

In the update step, first measurememt data package is initialized (note that the dimension and semantic of this measurement is dependent on the sensor type). In addition, measurement noise characterstics are captured using the covariance matrix `R_`, also depending on the sensor type (actual values are given beforehand). Finally, in the lidar measurement case, the method `KalmanFilter::Update()` is called:

```
void KalmanFilter::Update(const VectorXd &z) {
    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd K = P_ * H_.transpose() * S.inverse();
    // Calculate new estimates.
    x_ = x_ + K * (z - H_ * x_);
    P_ = P_ - K * H_ * P_;
}
```

which faithfully implements the standard Kalman Filter update operation. On the other hand, for radar measurement case, a different method `KalmanFilter::UpdateEKF()` is called:

```
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
```

which uses the Jacobian matrix for the previous state to determine the matrix `K` (Kalman gain) that is to be used in the measurement update. Note that a separate non-linear function is used instead of the matrix `H_` in obtaining `y`, which is used for calculation of new estimates of `x_`.

## Validation

Using the above mentioned test driver, the performance of the implemented EKF is measured. When fed the same data as the simulator does, the filter's response is tracked and displayed. Overall RMSE of the filter was:

```
0.0972256
0.0853761
 0.450855
 0.439588
```

where the data correspond to the RMSE of `px`, `py`, `vx`, and `vy`, respectively. This result conforms to the criteria specified in the [project rubric](https://review.udacity.com/#!/rubrics/748/view), which states that the values should be less than or equal to `[.11, .11, .52, .52]`.

Using the simulator on both a local machine (running Ubuntu 16.04) and the workspace provided by Udacity (on a virtual machine), the filter's performance was actually measured. The following figure illustrates the result when "Dataset 1" is used:

![Result for Dataset 1][result-1]

The final RMSE values are consistent with the previous test, with slight difference resulting presumably resulting from numerical precision. The green circles corresponds to the vehicle's position estimated by the Extended Kalman Filter, which is shown to closely follow the actual trajectory taken by the simulated vehicle.

Finally, a separate set of simulation was run with "Dataset 2", where the same data is fed backwards (with a radar measurement being the first one).

![Result for Dataset 2][result-2]

Again, the overall RMSE was `[.0726, .0967, .4582, .4971]`, which satisfies the criteria set by the [project rubric](https://review.udacity.com/#!/rubrics/748/view).

## Discussion

In the initial attempts, although the results given by the test driver (run on a local machine) confirmed the EKF did a good job in estimating the vehicle's position and velocity (verified by small values of RMSE for each), both the RMSE values and the estimated trajectory were unsatisfactory when the simulator was executed on the virtual machine provided by Udacity. After a set of debugging sessions (which compared the estimates of the state variable and covariance matrix from the same test driver executed on the local machine and on the virtual machine), the reason was found and taken care of.

While the following line gave the absolute value (in `float`) on the local machine,

```
float mag = abs(phi);
```

it turned out that on the virtual machine it gave `0` for `mag` when `phi` was approximately `-0.57`. After some additional tests, it was found out that the compilers on the local machine and on the virtual machine used different namespaces and this resulted in different behavior on different environments. Presumably the C++ `abs()` function on the virtual machine resolved to `int abs(int)` even though the argument `phi` has a `float` type. After adding the following line (near the beginning of `kalman_filter.cpp`)

```
using std::abs;
```

the problem went away and the same results were obtained by the test driver on the local machine and on the virtual machine. Then the simulations were done again and the results were verified as discussed above.

Further investigation would have revealed the different namespaces in which `abs()` was resolved; it was not done due to the deadline of the project submission approaching (and passed, in fact, at the time of this writeup).
