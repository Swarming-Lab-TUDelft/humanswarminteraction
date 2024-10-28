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

Eigen::MatrixXd LinearKalmanFilter::initializeStateTransitionMatrix(const double& dt) const
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
  A(STATE_HORIZONTAL_CENTROID_COORDINATE, STATE_HORIZONTAL_CENTROID_VELOCITY) = dt;
  A(STATE_VERTICAL_CENTROID_COORDINATE, STATE_VERTICAL_CENTROID_VELOCITY) = dt;
  A(STATE_TRACKING_WINDOW_HALF_WIDTH, STATE_TRACKING_WINDOW_WIDTH_VELOCITY) = dt;
  A(STATE_TRACKING_WINDOW_HALF_HEIGHT, STATE_TRACKING_WINDOW_HEIGHT_VELOCITY) = dt;
  return A;
}

void LinearKalmanFilter::initializeObservationMatrix()
{
  m_H = Eigen::MatrixXd::Zero(STATE_SIZE, MEASUREMENT_SIZE);
  m_H << 1, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0;
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
