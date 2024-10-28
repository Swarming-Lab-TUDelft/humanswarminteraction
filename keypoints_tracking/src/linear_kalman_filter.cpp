/* TU Delft Swarm Lab - Human Swarm Interaction Project
 * Description: Simple linear Kalman filter implementation.
 * Author: Alexander James Becoy
 * Date: 2024-10-28
 */

#include <keypoints_tracking/linear_kalman_filter.hpp>

using namespace HumanSwarmInteraction;

LinearKalmanFilter::LinearKalmanFilter(const std::array<double, STATE_SIZE>& initial_state,
  const std::array<double, STATE_SIZE>& covariances, const std::array<double, STATE_SIZE>& process_noises, 
  const std::array<double, MEASUREMENT_SIZE>& observation_noises)
{
  initializeObservationMatrix();
  initializeState(initial_state);
  initializeCovarianceMatrix(covariances);
  initializeProcessNoiseMatrix(process_noises);
  initializeObservationNoiseMatrix(observation_noises);
}

LinearKalmanFilter::LinearKalmanFilter(const std::vector<double>& initial_state, 
  const std::vector<double>& covariances, std::vector<double>& process_noises,
  const std::vector<double>& observation_noises)
{
  initializeObservationMatrix();
  initializeState(initial_state);
  initializeCovarianceMatrix(covariances);
  initializeProcessNoiseMatrix(process_noises);
  initializeObservationNoiseMatrix(observation_noises);
}

void LinearKalmanFilter::filter(const double& x, const double& y, 
  const double& w, const double& h, const double& dt)
{
  // Predict next state and covariance
  predict(dt);

  // Update state and covariance
  Eigen::VectorXd z(MEASUREMENT_SIZE);
  z << x, y, w, h;
  update(z);
}

std::array<double, OUTPUT_SIZE> LinearKalmanFilter::getCentroidCoordinate() const
{
  std::array<double, OUTPUT_SIZE> centroid;
  centroid[0] = m_x(STATE_HORIZONTAL_CENTROID_COORDINATE);
  centroid[1] = m_x(STATE_VERTICAL_CENTROID_COORDINATE);
  return centroid;
}

void LinearKalmanFilter::predict(const double& dt)
{
  // Get state transition matrix given variable time step
  Eigen::MatrixXd A = initializeStateTransitionMatrix(dt);

  // Predict next state and covariance
  m_x.noalias() = A * m_x;
  m_P.noalias() = A * m_P * A.transpose() + m_Q;
}

void LinearKalmanFilter::update(const Eigen::VectorXd& z)
{
  // Calculate Kalman gain
  Eigen::MatrixXd S = m_H * m_P * m_H.transpose() + m_R;
  Eigen::MatrixXd K = m_P * m_H.transpose() * S.inverse();

  // Update state and covariance
  m_x.noalias() += K * (z - m_H * m_x);
  m_P.noalias() -= K * m_H * m_P;
}

Eigen::MatrixXd LinearKalmanFilter::initializeStateTransitionMatrix(const double& dt) const
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
  for (int i = 0; i < MEASUREMENT_SIZE; i++) {
    A(i, i + MEASUREMENT_SIZE) = dt;
  }
  return A;
}

void LinearKalmanFilter::initializeObservationMatrix()
{
  m_H = Eigen::MatrixXd::Zero(STATE_SIZE, MEASUREMENT_SIZE);
  for (int i = 0; i < MEASUREMENT_SIZE; i++) {
    m_H(i, i) = 1;
  }
}

void LinearKalmanFilter::initializeState(const std::array<double, STATE_SIZE>& initial_state)
{
  m_x = Eigen::Map<const Eigen::VectorXd>(initial_state.data(), STATE_SIZE);
}

void LinearKalmanFilter::initializeCovarianceMatrix(const std::array<double, STATE_SIZE>& covariances)
{
  m_P = Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE);
  m_P.diagonal() = Eigen::Map<const Eigen::VectorXd>(covariances.data(), STATE_SIZE);
}

void LinearKalmanFilter::initializeProcessNoiseMatrix(const std::array<double, STATE_SIZE>& process_noises)
{
  m_Q = Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE);
  m_Q.diagonal() = Eigen::Map<const Eigen::VectorXd>(process_noises.data(), STATE_SIZE);
}

void LinearKalmanFilter::initializeObservationNoiseMatrix(const std::array<double, MEASUREMENT_SIZE>& observation_noises)
{
  m_R = Eigen::MatrixXd::Zero(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
  m_R.diagonal() = Eigen::Map<const Eigen::VectorXd>(observation_noises.data(), MEASUREMENT_SIZE);
}

void LinearKalmanFilter::initializeState(const std::vector<double>& initial_state)
{
  m_x = Eigen::Map<const Eigen::VectorXd>(initial_state.data(), STATE_SIZE);
}

void LinearKalmanFilter::initializeCovarianceMatrix(const std::vector<double>& covariances)
{
  m_P = Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE);
  m_P.diagonal() = Eigen::Map<const Eigen::VectorXd>(covariances.data(), STATE_SIZE);
}

void LinearKalmanFilter::initializeProcessNoiseMatrix(const std::vector<double>& process_noises)
{
  m_Q = Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE);
  m_Q.diagonal() = Eigen::Map<const Eigen::VectorXd>(process_noises.data(), STATE_SIZE);
}

void LinearKalmanFilter::initializeObservationNoiseMatrix(const std::vector<double>& observation_noises)
{
  m_R = Eigen::MatrixXd::Zero(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
  m_R.diagonal() = Eigen::Map<const Eigen::VectorXd>(observation_noises.data(), MEASUREMENT_SIZE);
}
