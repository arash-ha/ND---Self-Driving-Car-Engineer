#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {

  x_ = x_in; // Object state
  P_ = P_in; // Object covariance matrix
  F_ = F_in; // State transiction matrix
  H_ = H_in; // Measurement matrix
  R_ = R_in; // Measurement covariance matrix
  Q_ = Q_in; // Process covariance matrix
}

void KalmanFilter::Predict() {
  
   // TODO: predict the state
   x_ = F_ *x_;
   P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
   //measurement update
   VectorXd z_pred = H_ * x_;
   VectorXd y = z - z_pred;
   MatrixXd PHt = P_ * H_.transpose();
   MatrixXd S = H_ * PHt + R_;
   MatrixXd K = PHt * S.inverse();
   
   //new state
   x_ = x_ + (K * y);
   long x_size = x_.size();
   MatrixXd I = MatrixXd::Identity(x_size, x_size);
   P_ = (I - K * H_) * P_;
   
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
   // get components of predicted state
   float px = x_(0);
   float py = x_(1);
   float vx = x_(2);
   float vy = x_(3);
   
   // get components of radar measurement space
   float rho = sqrt(px * px + py * py);
   float phi = atan2(py, px);
   float rho_dot = (rho != 0 ? (px * vx + py * vy) / rho : 0);
   
   // define predicted position and speed
   VectorXd z_pred(3);
   z_pred << rho, phi, rho_dot;
   
   // measurement update
   VectorXd y = z - z_pred;
   
   // normalize angle
   double width = 2 * M_PI;
   double offsetValue = y(1) + M_PI;
   y(1) = (offsetValue - (floor(offsetValue / width) * width)) - M_PI;
   
   MatrixXd PHt = P_ * H_.transpose();
   MatrixXd S = H_ * PHt + R_;
   MatrixXd K = PHt * S.inverse();
   
   // new state
   x_ = x_ + (K * y);
   int x_size = x_.size();
   MatrixXd I = MatrixXd::Identity(x_size,x_size);
   P_ = (I - K * H_) * P_;
   
}
