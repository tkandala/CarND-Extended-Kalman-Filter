#include <iostream>
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  //std::cout << "Predict()" << std::endl;

  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  //std::cout << "Update()" << std::endl;

  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(4, 4);
  P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  //std::cout << "PredictEKF()" << std::endl;

  MatrixXd h(3,1);

  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt(px*px+py*py);
  float phi = atan2(py, px);

  if(rho < 0.0001){
      std::cout << "UpdateEKF () - Error - Division by Zero" << std::endl;
      return;
  }

  float rho_dot = (px*vx + py*vy)/rho;

  h(0,0) = rho;
  h(1,0) = phi;
  h(2,0) = rho_dot;

  //std::cout << "UpdateEKF () - finished calculating h" << std::endl;

  VectorXd y = z - h;

  y(1) = atan2(sin(y(1)), cos(y(1)));

  //std::cout << "UpdateEKF () - finished calculating y" << std::endl;
  MatrixXd Ht = H_.transpose();
  //std::cout << "UpdateEKF () - finished calculating Ht" << std::endl;
  MatrixXd S = H_ * P_ * Ht + R_;
  //std::cout << "UpdateEKF () - finished calculating S" << std::endl;
  MatrixXd Si = S.inverse();
  //std::cout << "UpdateEKF () - finished calculating Si" << std::endl;
  MatrixXd K = P_ * Ht * Si;
  //std::cout << "UpdateEKF () - finished calculating K" << std::endl;

  x_ = x_ + (K * y);
  //std::cout << "UpdateEKF () - finished calculating x_" << std::endl;
  MatrixXd I = MatrixXd::Identity(4, 4);
  P_ = (I - K * H_) * P_;

  //std::cout << "UpdateEKF () - finished calculating P_" << std::endl;

}
