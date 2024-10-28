/* TU Delft Swarm Lab - Human Swarm Interaction Project
 * Description: Simple linear Kalman filter implementation.
 * Author: Alexander James Becoy
 * Date: 2024-10-28
 */

#ifndef KEYPOINTS_TRACKING__LINEAR_KALMAN_FILTER_HPP_
#define KEYPOINTS_TRACKING__LINEAR_KALMAN_FILTER_HPP_

#include <Eigen/Dense>

#define STATE_SIZE 8
#define STATE_HORIZONTAL_CENTROID_COORDINATE    0
#define STATE_VERTICAL_CENTROID_COORDINATE      1
#define STATE_TRACKING_WINDOW_HALF_WIDTH        2
#define STATE_TRACKING_WINDOW_HALF_HEIGHT       3
#define STATE_HORIZONTAL_CENTROID_VELOCITY      4
#define STATE_VERTICAL_CENTROID_VELOCITY        5
#define STATE_TRACKING_WINDOW_WIDTH_VELOCITY    6
#define STATE_TRACKING_WINDOW_HEIGHT_VELOCITY   7

#define MEASUREMENT_SIZE 4
#define MEASUREMENT_HORIZONTAL_CENTROID_COORDINATE    0
#define MEASUREMENT_VERTICAL_CENTROID_COORDINATE      1
#define MEASUREMENT_TRACKING_WINDOW_HALF_WIDTH        2
#define MEASUREMENT_TRACKING_WINDOW_HALF_HEIGHT       3

namespace HumanSwarmInteraction
{
  class LinearKalmanFilter
  {
    public:
      LinearKalmanFilter();
      ~LinearKalmanFilter() = default;

      void initialize(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0, const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& H, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
      void predict();
      void update(const Eigen::VectorXd& z);

      Eigen::VectorXd getState();
      Eigen::MatrixXd getCovariance();

    private:
      Eigen::VectorXd m_x;
      Eigen::MatrixXd m_P;
      Eigen::MatrixXd m_A;
      Eigen::MatrixXd m_B;
      Eigen::MatrixXd m_H;
      Eigen::MatrixXd m_Q;
      Eigen::MatrixXd m_R;

  };  // class LinearKalmanFilter
} // namespace HumanSwarmInteraction

#endif  // KEYPOINTS_TRACKING__LINEAR_KALMAN_FILTER_HPP_
