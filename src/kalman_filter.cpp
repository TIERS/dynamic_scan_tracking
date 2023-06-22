// Code adapted from https://github.com/udacity/CarND-Extended-Kalman-Filter-Project

#include <dynamic_scan_tracking/kalman_filter.h>

#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::predict(float delta_T)
{

  // Update state transition matrix F for a constant velocity motion model
  F_.setIdentity();
  F_(0, 3) = delta_T;
  F_(1, 4) = delta_T;
  F_(2, 5) = delta_T;

  // Update process noise covariance matrix Q for a constant velocity motion model
  Q_.setZero();
  Q_(0, 0) = Q_(1, 1) = Q_(2, 2) = pow(delta_T, 4)/4;
  Q_(0, 3) = Q_(1, 4) = Q_(2, 5) = pow(delta_T, 3)/2;
  Q_(3, 0) = Q_(4, 1) = Q_(5, 2) = pow(delta_T, 3)/2;
  Q_(3, 3) = Q_(4, 4) = Q_(5, 5) = pow(delta_T, 2);
  q_noise = 3;
  Q_ = Q_ * q_noise;

  x_ = F_ * x_; // predict new estimate
  P_ = F_ * P_ * F_.transpose() + Q_; // update new state estimation covariance matrix

}

void KalmanFilter::predictEKF(float delta_T)
{
  float std_a_ = 0.05;
  float std_yawdd_ = 0.05;

  // Use the Constant Turn Rate and Velocity Motion Model to predict new state
  // extract values for better readability
  double p_x = x_(0);
  double p_y = x_(1);
  double p_z = x_(2);
  double v = x_(3);
  double yaw = x_(4);
  double yawd = x_(5);

  //predicted state values
  double px_p, py_p, pz_p;
  double yawd_p;

  //avoid division by zero
  if (fabs(yawd) > 0.001)
  {
      px_p = p_x + v/yawd * ( sin (yaw + yawd * delta_T) - sin(yaw));
      py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw + yawd * delta_T) );
      yawd_p = yawd;
  }
  else
  {
      px_p = p_x + v * delta_T * cos(yaw);
      py_p = p_y + v * delta_T * sin(yaw);
      yawd_p = 0.001;
  }

  pz_p = p_z + v * delta_T;
  double v_p = v;
  double yaw_p = yaw + yawd * delta_T;

  // Calculate the Jacobian of the Transition Matrix F for CTRV model
  F_.setIdentity();
  F_(0, 3) = (1.0 / yawd_p) * (sin(yawd_p * delta_T + yaw_p) - sin(yaw_p));
  F_(0, 4) = (v_p / yawd_p) * (cos(yawd_p * delta_T + yaw_p) - cos(yaw_p));
  F_(0, 5) = (delta_T * v_p / yawd_p)*cos(yawd_p * delta_T + yaw_p) - (v_p / pow(yawd_p, 2))*(sin(yawd_p * delta_T + yaw_p) - sin(yaw_p));
  F_(1, 3) = (1.0 / yawd_p) * (cos(yaw_p) - cos(yawd_p * delta_T + yaw_p));
  F_(1, 4) = (v_p / yawd_p) * (sin(yawd_p * delta_T + yaw_p) - sin(yaw_p));
  F_(1, 5) = (delta_T * v_p / yawd_p) * sin(yawd_p * delta_T + yaw_p) - (v_p / pow(yawd_p, 2))*(cos(yaw_p) - cos(yawd_p * delta_T + yaw_p));
 
  // Update process noise covariance matrix Q for a constant velocity motion model
  Q_.setZero();

  Q_(0, 0) = std_a_ * std_a_;
  Q_(1, 1) = std_a_ * std_a_;
  Q_(2, 2) = std_a_ * std_a_;
  Q_(3, 3) = std_a_ * std_a_;
  Q_(4, 4) = std_yawdd_ * std_yawdd_;
  Q_(5, 5) = std_yawdd_ * std_yawdd_;
  
  P_ = F_ * P_ * F_.transpose() + Q_; // update new state estimation covariance matrix
}

void KalmanFilter::update(const VectorXd &z)
{
  VectorXd z_pred = H_ * x_; // compute predicted observation. Project estimated state THROUGH observation space
  
  VectorXd y = z - z_pred; // residual

  MatrixXd S = H_ * P_ * H_.transpose() + R_;

  MatrixXd K = P_ * H_.transpose() * S.inverse();

  // Estimate new state
  x_ = x_ + K * y;

  MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());
  P_ = (I - K * H_) * P_;

}