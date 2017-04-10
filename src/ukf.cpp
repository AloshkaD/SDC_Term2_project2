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
 
  //set measurement dimension, radar can measure r, phi, and r_dot
  //int n_z = 3;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */

void UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (!is_initialized_) {
      x_ << 0.1, 0.1, 0.1, 0.1, 0.1; //pos1, pos2, vel_abs, yaw_angle, yaw_rate
  /*
    x << 5.7441,
         1.3800,
         2.2049,
         0.5015,
         0.3528;
  */
    P_ << 0.1, 0, 0, 0, 0,
          0, 0.1, 0, 0, 0,
          0, 0, 0.1, 0, 0,
          0, 0, 0, 0.1, 0,
          0, 0, 0, 0, 0.1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float theta = measurement_pack.raw_measurements_[1];
      float px = measurement_pack.raw_measurements_[0]*cos(theta);
      float py = measurement_pack.raw_measurements_[0]*sin(theta);
      if(fabs(px) < 0.00001 or fabs(py) < 0.00001){
        //cout<< "px or py is 0"<<"\n";
        //px = 0.00001;
        //py = 0.00001;
      return;
    
  }
      float ro_dot = measurement_pack.raw_measurements_(2);
      float phi = measurement_pack.raw_measurements_(1);
      x_ << px, py,ro_dot * cos(phi), ro_dot * sin(phi),0;

      //ekf_.x_ << px, py, 0, 0,0;
}

    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state with 0,0,0,0,0 value
      */
      float px_laser = measurement_pack.raw_measurements_[0];
      float py_laser = measurement_pack.raw_measurements_[1];

      if(fabs(px_laser) < 0.00001 or fabs(py_laser) < 0.00001){
        //cout<< "px_laser or py_laser is 0"<<"\n";
        //px_laser = 0.00001;
        //py_laser = 0.00001;
        return;
      }
        x_ << px_laser, py_laser,0, 0, 0;
      }
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
}

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1e6;
  previous_timestamp_ = measurement_pack.timestamp_;

  Prediction(dt);
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(measurement_pack);
  } else {
    UpdateLidar(measurement_pack);
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

  //create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

  //calculate square root of P
  MatrixXd A = P_.llt().matrixL();

  //set first column of sigma point matrix
  Xsig.col(0)  = x_;

  //set remaining sigma points
  for (int i = 0; i < n_x_; i++)
  {
    Xsig.col(i+1)     = x_ + sqrt(lambda_+n_x_) * A.col(i);
    Xsig.col(i+1+n_x_) = x_ - sqrt(lambda_+n_x_) * A.col(i);
  }

   //create augmented mean vector
  VectorXd x_aug = VectorXd(7);
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
 
 //create augmented mean state
  x_aug.head(5) = x_;
  
  x_aug(5) = 0;
   
  x_aug(6) = 0;
  

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

   //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }
  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);
  //double delta_t = 0.1; //time diff in sec
  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;
//avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }
    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;
    
    //write predicted sigma point into right column
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
  }
  Xsig_pred_ = Xsig_pred;

  weights_ = VectorXd(2 *  n_aug_ + 1);
  weights_.fill(0.0);

  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {
    weights_(i) = 0.5 / (n_aug_ + lambda_);
  }

  x_.fill(0.0);
  for (int i = 0; i<2*n_aug_+1; i++)
  {
      x_ = x_+weights_(i)*Xsig_pred_.col(i);
  }

  P_.fill(0.0);
  for (int i = 0; i<2*n_aug_+1; i++)
  {
      VectorXd x_pred_minus_x = Xsig_pred_.col(i) - x_;
      while (x_pred_minus_x(3)> M_PI) x_pred_minus_x(3)-=2.*M_PI;
      while (x_pred_minus_x(3)<-M_PI) x_pred_minus_x(3)+=2.*M_PI;
      P_ = P_ + weights_(i) * x_pred_minus_x * x_pred_minus_x.transpose();
  }
  return;
}


double UKF::Update(MeasurementPackage meas_package, int n_z, MatrixXd Zsig, MatrixXd R, VectorXd z) 
{
  //create example vector for mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  //z_pred << 0.0, 0.0, 0.0;
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  //create example matrix for predicted measurement covariance
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    /*
    //create vector for weights
    VectorXd weights = VectorXd(2*n_aug_+1);
    // set weights
    double weight_0 = lambda_/(lambda_+n_aug_);
    weights(0) = weight_0;
    for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
     double weight = 0.5/(n_aug_+lambda_);
     weights(i) = weight;
    }
    */
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  //MatrixXd R = MatrixXd(n_z,n_z);
 // R <<    std_radr*std_radr, 0, 0,
  //        0, std_radphi*std_radphi, 0,
   //       0, 0,std_radrd*std_radrd;
  S = S + R;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
//calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
//Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
  return z_diff.transpose() * S.inverse() * z_diff;
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
  if (x_(0) == 0) return;
  int n_z =2;
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    Zsig(0,i) = p_x;
    Zsig(1,i) = p_y;
  }

  MatrixXd R = MatrixXd(n_z,n_z);
  R << std_laspx_*std_laspx_, 0,
       0, std_laspx_*std_laspx_;

  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_(0),
      meas_package.raw_measurements_(1);

  NIS_laser_ = Update(meas_package, n_z, Zsig, R, z);

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
  //create matrix for sigma points in measurement space
  if (x_(0) == 0) return;
  int n_z =3;
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
     VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_(0),
       meas_package.raw_measurements_(1),
       meas_package.raw_measurements_(2);

  NIS_radar_ = Update(meas_package,n_z, Zsig, R, z);
}
