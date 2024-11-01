/* TU Delft Swarm Lab - Human Swarm Interaction Project
 * Description: Simple linear Kalman filter implementation.
 * Author: Alexander James Becoy
 * Date: 2024-10-28
 */

#ifndef KEYPOINTS_TRACKING__LINEAR_KALMAN_FILTER_HPP_
#define KEYPOINTS_TRACKING__LINEAR_KALMAN_FILTER_HPP_

#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#define STATE_SIZE 4
#define STATE_HORIZONTAL_CENTROID_COORDINATE    0
#define STATE_VERTICAL_CENTROID_COORDINATE      1
#define STATE_HORIZONTAL_CENTROID_VELOCITY      2
#define STATE_VERTICAL_CENTROID_VELOCITY        3

#define MEASUREMENT_SIZE 2
#define MEASUREMENT_HORIZONTAL_CENTROID_COORDINATE    0
#define MEASUREMENT_VERTICAL_CENTROID_COORDINATE      1

#define OUTPUT_SIZE 2

namespace HumanSwarmInteraction
{
  class LinearKalmanFilter
  {
  public:
    LinearKalmanFilter(const std::array<double, STATE_SIZE>& initial_state,
      const std::array<double, STATE_SIZE>& covariances, const std::array<double, STATE_SIZE>& process_noises, 
      const std::array<double, MEASUREMENT_SIZE>& observation_noises, const double& dt);
    LinearKalmanFilter(const std::vector<double>& initial_state, 
      const std::vector<double>& covariances, std::vector<double>& process_noises,
      const std::vector<double>& observation_noises, const double& dt);
    ~LinearKalmanFilter() = default;

    std::array<double, OUTPUT_SIZE> getCentroidCoordinate() const;
    void predict();
    void update(const double& x, const double& y);

    void setInitialState(const double& x, const double& y, const double& vx, const double& vy);

  private:
    void initializeStateTransitionMatrix(const double& dt);
    void initializeObservationMatrix();

    void initializeState(const std::array<double, STATE_SIZE>& initial_state);
    void initializeCovarianceMatrix(const std::array<double, STATE_SIZE>& covariances);
    void initializeProcessNoiseMatrix(const std::array<double, STATE_SIZE>& process_noises);
    void initializeObservationNoiseMatrix(const std::array<double, MEASUREMENT_SIZE>& observation_noises);

    void initializeState(const std::vector<double>& initial_state);
    void initializeCovarianceMatrix(const std::vector<double>& covariances);
    void initializeProcessNoiseMatrix(const std::vector<double>& process_noises);
    void initializeObservationNoiseMatrix(const std::vector<double>& observation_noises);

    Eigen::VectorXd m_x;
    Eigen::MatrixXd m_P;
    Eigen::MatrixXd m_A;    // Parameterizable depending on dt
    // Eigen::MatrixXd m_B; // Not used
    Eigen::MatrixXd m_H;    // Fixed
    Eigen::MatrixXd m_Q;    // Parameterizable
    Eigen::MatrixXd m_R;    // Parameterizable

  };  // class LinearKalmanFilter
} // namespace HumanSwarmInteraction

#endif  // KEYPOINTS_TRACKING__LINEAR_KALMAN_FILTER_HPP_
