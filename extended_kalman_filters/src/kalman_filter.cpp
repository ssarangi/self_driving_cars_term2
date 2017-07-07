#include "kalman_filter.h"
#include "tools.h"

#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in,
                        MatrixXd &P_in,
                        MatrixXd &F_in,
                        MatrixXd &H_in,
                        MatrixXd &R_laser_in,
                        MatrixXd &R_Radar_in,
                        MatrixXd &Q_in) {
  x_  = x_in;
  P_  = P_in;
  F_  = F_in;
  H_  = H_in;
  R_laser  = R_laser_in;
  R_radar = R_Radar_in;
  Q_  = Q_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x_;
  P_ = UpdateCommon(H_, P_, R_laser, y);
}

void KalmanFilter::UpdateEKF(const Tools& tools, const VectorXd &z) {
  MatrixXd Hj = tools.CalculateJacobian(x_);

  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  double rho = sqrt(px * px + py * py);
  double phi = atan2(py, px);
  double rho_dot = (px * vx + py * vy) / (rho);

  //check division by zero
  if(fabs(rho) < 0.0001){
    std::cout << "UpdateEKF () - Error - Divide by zero,  skipping current RADAR update calculation.\n\n";
    return;
  }

  VectorXd h = VectorXd(3);
  h << rho, phi, rho_dot;

  VectorXd y = z - h;

  y(1) = NormalizeAngle(y(1));
  P_ = UpdateCommon(Hj, P_, R_radar, y);
}

MatrixXd KalmanFilter::UpdateCommon(const MatrixXd &H, MatrixXd &P, const MatrixXd &R, const VectorXd &y) {
  MatrixXd Ht = H.transpose();
  MatrixXd PHt = P * Ht;
  MatrixXd S = H * PHt + R;
  MatrixXd K = PHt * S.inverse();

  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  MatrixXd Pd = (I - K * H) * P;
  return Pd;
}
