#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


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

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;


  ekf_.P_ = MatrixXd(4,4);
  ekf_.P_ << 10,  0,   0,   0,
             0,   10,  0,   0,
             0,   0,   10,  0,
             0,   0,   0,   10;

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
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */
     

    // first measurement

    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    cout << "measurement_type: " << measurement_pack.sensor_type_ << endl;
    
    // initialize position and velocity
    float px = 0.0;
    float py = 0.0;
    float vx = 0.0;
    float vy = 0.0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      float rho     = measurement_pack.raw_measurements_[0];
      float phi     = measurement_pack.raw_measurements_[1];
      float rho_dot = measurement_pack.raw_measurements_[2];

      cout << "rho: " << rho << endl;
      cout << "phi: " << phi << endl;
      cout << "rho_dot: " << rho_dot << endl;

      // Convert each coordinate
      px = rho * cos(phi);
      py = rho * sin(phi);
      vx = rho_dot * cos(phi);
      vy = rho_dot * sin(phi);

      ekf_.x_ << px, py, vx, vy;
    
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
        px = measurement_pack.raw_measurements_[0];
        py = measurement_pack.raw_measurements_[1];

        ekf_.x_ << px, py, 0, 0;

    }
    
    if (fabs(ekf_.x_(0)) < 0.0001 and fabs(ekf_.x_(1)) < 0.0001){
      ekf_.x_(0) = 0.0001;
      ekf_.x_(1) = 0.0001;
    }
    
    cout << "EKF init: " << ekf_.x_ << endl;
    // Save initial timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
   // calculation of time elapsed
   float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
   previous_timestamp_ = measurement_pack.timestamp_;
   
   // update the state transition matrix F according to the new elapsed time
   ekf_.F_ = MatrixXd(4, 4);
   ekf_.F_ << 1,  0,  dt, 0,
             0,  1,  0,  dt,
             0,  0,  1,  0,
             0,  0,  0,  1;
   
   float dt_2 = dt * dt;
   float dt_3 = dt_2 * dt / 2.0;
   float dt_4 = dt_3 * dt / 2.0;

   float noise_ax{9.0};
   float noise_ay{9.0};
   
   // set the process covariance matrix Q
   ekf_.Q_ = MatrixXd(4,4);
   ekf_.Q_ << dt_4 * noise_ax, 0, dt_3 * noise_ax, 0,
              0, dt_4 * noise_ay, 0, dt_3 * noise_ay,
              dt_3 * noise_ax, 0, dt_2 * noise_ax, 0,
              0, dt_3 * noise_ay, 0, dt_2 * noise_ay;            
    
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    try {
        ekf_.R_ = R_radar_;
        ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } catch (...) {
        return;
    }

  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    // TODO: Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
