#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3; // fixed this value from 30

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.2; // fixed this value from 30 

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.1;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.1;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.0003;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.1;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  // state dimention
  n_x_ = 5;
  // Augumented state dimention
  n_aug_ = 7;
  // Sigma point spreading parameter
  lambda_ = 3 - n_x_;
  //
  is_initialized_ = false;
  //
  previous_timestamp_ = 0;
 


}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (!is_initialized_){
  //float v=0.1 // estimate velocity for now
  
  x_ = 0.1, 0.1, 0.1, 0.1, 0.1; //pos1, pos2, vel_abs, yaw_angle, yaw_rate
  /*
    x << 5.7441,
         1.3800,
         2.2049,
         0.5015,
         0.3528;
  */
  previous_timestamp_ = meas_package.timestamp_;
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float theta = meas_package.raw_measurements_[1];
      float px = meas_package.raw_measurements_[0]*cos(theta);
      float py = meas_package.raw_measurements_[0]*sin(theta);
      if(fabs(px) < 0.00001 or fabs(py) < 0.00001){
        //cout<< "px or py is 0"<<"\n";
        //px = 0.00001;
        //py = 0.00001;
      return;
    
  }
      float ro_dot = meas_package.raw_measurements_(2);
      float phi = meas_package.raw_measurements_(1);
      x_ << px, py,v, ro_dot * cos(phi), ro_dot * sin(phi),0;

      //ekf_.x_ << px, py, 0, 0,0;
}

 else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state with 0,0,0,0,0 value
      */
      float px_laser = meas_package.raw_measurements_[0];
      float py_laser = meas_package.raw_measurements_[1];

      if(fabs(px_laser) < 0.00001 or fabs(py_laser) < 0.00001){
        //cout<< "px_laser or py_laser is 0"<<"\n";
        //px_laser = 0.00001;
        //py_laser = 0.00001;
        return;
      }
        x_ << px_laser, py_laser,0, 0, 0;
      }
    is_initialized_ = true;
    return;
}
float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds

Prediction(dt);
if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
UpdateRadar(meas_package);
}
else{
UpdateLidar(meas_package);
}

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  //set example covariance matrix
 // MatrixXd P_ = MatrixXd(n_x, n_x);
  P_ << 0.1, 0, 0, 0, 0,
        0, 0.1, 0, 0, 0,
        0, 0, 0.1, 0, 0,
        0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0.1;

  //create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x, 2 * n_x + 1);

  //calculate square root of P
  MatrixXd A = P_.llt().matrixL();

  //set first column of sigma point matrix
  Xsig.col(0)  = x_;

  //set remaining sigma points
  for (int i = 0; i < n_x; i++)
  {
    Xsig.col(i+1)     = x + sqrt(lambda+n_x) * A.col(i);
    Xsig.col(i+1+n_x) = x - sqrt(lambda+n_x) * A.col(i);
  }

 
 
 
 
 
 
 //*Xsig_out = Xsig;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
