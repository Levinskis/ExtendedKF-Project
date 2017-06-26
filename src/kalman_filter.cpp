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

  x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {

  //Measurment
  VectorXd z_ = H_ * x_;
  VectorXd y = z - z_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

  meas = VectorXd(3);

  float v_len = sqrt(pow(x_[0],  2) + pow(x_[1], 2));
  meas_ << v_len, atan2(x_[1], x_[0]), (x_[0]*x_[2] + x_[1]*x_[3]) / v_len;
  VectorXd y = z - meas_;

  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * Ht * S.inverse();

  //Estimation
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;


}
