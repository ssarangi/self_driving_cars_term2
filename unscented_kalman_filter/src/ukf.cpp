#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <stdexcept>

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
  std_a_ = 0.2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.6;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.08;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  n_x_ = 5;

  n_aug_ = 7;

  n_z_lidar_ = 2;

  n_z_radar_ = 3;

  lambda_ = 3 - n_aug_;

  is_initialized_ = false;
}

UKF::~UKF() {}

MatrixXd UKF::GenerateSigmaPoints(const VectorXd x, const MatrixXd P) {
  // Create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  // Create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  // Create Sigma Point Matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  x_aug.head(5) = x;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // Create the Augmented Covariance Matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5, 5) = P;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  // Create Square Root Matrix
  MatrixXd L = P_aug.llt().matrixL();

  // Create the augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; ++i) {
    Xsig_aug.col(i+1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  return Xsig_aug;
}

MatrixXd UKF::PredictSigmaPoints(const MatrixXd& Xsig_aug, const double delta_t) {
  // Create Matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // Predict the sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    double p_x = Xsig_aug(0, i);
    double p_y = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    // Predicted State values
    double px_p, py_p;

    if (fabs(yawd) > 0.001) {
      px_p = p_x + v/yawd * (sin (yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v/yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    } else {
      px_p = p_x + v * delta_t * cos(yaw);
      py_p = p_y + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    // Add the noise parameters
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;

    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    //write predicted sigma point into right column
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
  }

  return Xsig_pred;
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  // Check if the filter has been initialized or not
  if (!is_initialized_) {
    std::cout << "Unscented Kalman Initialization" << std::endl;
    // Initialize the state
    x_ << 1, 1, 1, 1, 1;

    // Initialize the State Covariance Matrix.
    P_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;

    weights_ = VectorXd(2 * n_aug_ + 1);
    weights_.fill(1/(2*(lambda_ + n_aug_)));
    weights_(0) = lambda_/(lambda_ + n_aug_);

    // First measurement
    double px, py, v, yaw, yaw_rt;

    if (meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER) {
      px = meas_package.raw_measurements_[0];
      py = meas_package.raw_measurements_[1];
      v = 0;
      yaw = 0;
      yaw_rt = 0;
    } else if (meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR) {
      // Convert radar from polar coordinates to cartesian coordinates and initialize state
      double rho = meas_package.raw_measurements_(0);
      double phi = meas_package.raw_measurements_(1);
      double rhodot = meas_package.raw_measurements_(2);

      px = rho * cos(phi);
      py = rho * sin(phi);
      v = 0;
      yaw = 0;
      yaw_rt = 0;
    } else {
      throw std::runtime_error("Invalid Sensor Type encountered");
    }

    if (fabs(px) < 0.00001) {
      std::cout << "Px is too small. Increasing to 0.01" << std::endl;
      px = 0.01;
    }

    if (fabs(py) < 0.00001) {
      std::cout << "Py is too small. Increasing to 0.01" << std::endl;
      py = 0.01;
    }

    x_ << px, py, v, yaw, yaw_rt;
    previous_timestamp_ = meas_package.timestamp_;
    is_initialized_ = true;
  }

  double delta_t = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = meas_package.timestamp_;

  if (meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER && use_laser_) {
    Prediction(delta_t);
    UpdateLidar(meas_package);
  } else if (meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR && use_radar_) {
    Prediction(delta_t);
    UpdateRadar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  MatrixXd Xsig_aug = GenerateSigmaPoints(x_, P_);

  Xsig_pred_ = PredictSigmaPoints(Xsig_aug, delta_t);

  // Predict the State mean
  x_.fill(0);
  for(int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  // Predict State Covariance Matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    VectorXd x_pred_minus_x = Xsig_pred_.col(i) - x_;

    if (x_pred_minus_x(3) > M_PI)
      x_pred_minus_x(3) -= 2.0 * M_PI;

    if (x_pred_minus_x(3) < -M_PI)
      x_pred_minus_x(3) += 2.0 * M_PI;

    P_ = P_ + weights_(i) * x_pred_minus_x * x_pred_minus_x.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  if (x_(0) == 0)
    return;

  MatrixXd Zsig = MatrixXd(n_z_lidar_, 2 * n_aug_ + 1);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);

    Zsig(0, i) = p_x;
    Zsig(1, i) = p_y;
  }

  // Mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_lidar_);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // Measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_lidar_, n_z_lidar_);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd z_diff = Zsig.col(i) - z_pred;

    while (z_diff(1) > M_PI) z_diff(1) -= 2.0 * M_PI;
    while (z_diff(1) > M_PI) z_diff(1) += 2.0 * M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // Measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z_lidar_, n_z_lidar_);
  R << std_laspx_ * std_laspx_, 0,
       0, std_laspy_ * std_laspy_;
  S = S + R;

  MatrixXd Tc = MatrixXd(n_x_, n_z_lidar_);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd z_diff = Zsig.col(i) - z_pred;

    while (z_diff(1) > M_PI) z_diff(1) -= 2.0 * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.0 * M_PI;

    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    while (x_diff(3) > M_PI) x_diff(3) -= 2.0 * M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2.0 * M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  MatrixXd K = Tc * S.inverse();
  VectorXd z = VectorXd(n_z_lidar_);
  z << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1);

  VectorXd z_diff = z - z_pred;

  while (z_diff(1) > M_PI) z_diff(1) -= 2.0 * M_PI;
  while (z_diff(1) < -M_PI) z_diff(1) += 2.0 * M_PI;

  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  if (x_(0) == 0)
    return;

  MatrixXd Zsig = MatrixXd(n_z_radar_, 2 * n_aug_ + 1);

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);
    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);
    Zsig(1, i) = atan2(p_y, p_x);
    Zsig(2, i) = (p_x*v1 + p_y * v2) / sqrt(p_x*p_x + p_y*p_y);
  }

  // Mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_radar_);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  MatrixXd S = MatrixXd(n_z_radar_, n_z_radar_);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd z_diff = Zsig.col(i) - z_pred;

    while (z_diff(1) > M_PI) z_diff(1) -= 2.0 * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.0 * M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  MatrixXd R = MatrixXd(n_z_radar_, n_z_radar_);
  R << std_radr_ * std_radr_, 0, 0,
       0, std_radphi_ * std_radphi_, 0,
       0, 0, std_radrd_ * std_radr_;

  S = S + R;

  MatrixXd Tc = MatrixXd(n_x_, n_z_radar_);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd z_diff = Zsig.col(i) - z_pred;

    while (z_diff(1) > M_PI) z_diff(1) -= 2.0 * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.0 * M_PI;

    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    while (x_diff(3) > M_PI) x_diff(3) -= 2.0 * M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2.0 * M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  MatrixXd K = Tc * S.inverse();
  VectorXd z = VectorXd(n_z_radar_);
  z << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), meas_package.raw_measurements_(2);
  VectorXd z_diff = z - z_pred;

  while (z_diff(1) > M_PI) z_diff(1) -= 2.0 * M_PI;
  while (z_diff(1) < -M_PI) z_diff(1) += 2.0 * M_PI;

  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
}
