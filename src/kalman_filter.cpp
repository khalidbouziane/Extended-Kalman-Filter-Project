#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_*x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_*P_*Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_*x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd K = P_*Ht*S.inverse();

  MatrixXd I = MatrixXd::Identity(4, 4);

  // new state
  x_ = x_ + K*y;
  P_ = (I - K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt(px*px+py*py);
  float theta = atan2(py, px);
  float rho_dot = (px*vx + py*vy)/rho;

  if(fabs(rho) < 0.0001){
    rho_dot = 0;
  }

  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, rho_dot;

  VectorXd y = z - z_pred;

  // angle normalization
  y(1) = atan2(sin(y(1)), cos(y(1)));

  //std::cout << "EKF -> Calculate y done." << std::endl;
  MatrixXd Ht = H_.transpose();
  //std::cout << "EKF -> H_ = " << H_ << std::endl;
  MatrixXd S = H_*P_*Ht + R_;
  //std::cout << "EKF -> S_ = " << S << std::endl;
  MatrixXd K = P_*Ht*S.inverse();
  //std::cout << "EKF -> K = " << K << std::endl;

  MatrixXd I = MatrixXd::Identity(4, 4);

  // new state
  x_ = x_ + K*y;
  P_ = (I - K*H_)*P_;
  //std::cout << "EKF -> new state done." << std::endl;

}
