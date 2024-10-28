/* TU Delft Swarm Lab - Human Swarm Interaction Project
 * Description: Track and filter a single keypoint linearly.
 * Author: Alexander James Becoy
 * Date: 2024-10-28
 */

#ifndef KEYPOINTS_TRACKING__SINGLE_KEYPOINT_TRACKER_NODE_HPP_
#define KEYPOINTS_TRACKING__SINGLE_KEYPOINT_TRACKER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <human_swarm_interaction_interfaces/msg/pose_keypoints_stamped.hpp>

#include <vector>
#include <string>

#include <keypoints_tracking/linear_kalman_filter.hpp>

namespace HumanSwarmInteraction
{
  class SingleKeypointTrackerNode : public rclcpp::Node
  {
    public:
      SingleKeypointTrackerNode();
      ~SingleKeypointTrackerNode();

    private:
      void keypointsCallback(const human_swarm_interaction_interfaces::msg::PoseKeypointsStamped::SharedPtr msg);
      void publishKeypoints(double x, double y) const;

      void initializeLinearKalmanFilter();
      void initializeKeyPointsSubscriber();
      void initializeKeyPointsPublisher();
      void configureParameters();

      std::unique_ptr<LinearKalmanFilter> m_kalman_filter;
      std::string m_keypoint_name;
      double m_tracking_window_width;
      double m_tracking_window_height;
      std_msgs::msg::Header * m_last_msg_header;

      rclcpp::Subscription<human_swarm_interaction_interfaces::msg::PoseKeypointsStamped>::SharedPtr m_keypoints_sub;
      rclcpp::Publisher<human_swarm_interaction_interfaces::msg::PoseKeypointsStamped>::SharedPtr m_keypoints_pub;

  };  // class SingleKeypointTrackerNode
} // namespace HumanSwarmInteraction

#endif  // KEYPOINTS_TRACKING__SINGLE_KEYPOINT_TRACKER_NODE_HPP_