/* TU Delft Swarm Lab - Human Swarm Interaction Project
 * Description: Track and filter a single keypoint linearly.
 * Author: Alexander James Becoy
 * Date: 2024-10-28
 */

#ifndef KEYPOINTS_TRACKING__SINGLE_KEYPOINT_TRACKER_NODE_HPP_
#define KEYPOINTS_TRACKING__SINGLE_KEYPOINT_TRACKER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <human_swarm_interaction_interfaces/msg/pose_keypoints_stamped.hpp>

#include <vector>
#include <string>

namespace HumanSwarmInteraction
{
  class SingleKeypointTrackerNode : public rclcpp::Node
  {
    public:
      SingleKeypointTrackerNode();
      ~SingleKeypointTrackerNode();

    private:
      void keypointsCallback(const human_swarm_interaction_interfaces::msg::PoseKeypointsStamped::SharedPtr msg);
      void publishKeypoints() const;

      // Configurations
      void initializeKeyPointsSubscriber();
      void initializeKeyPointsPublisher();
      void configureParameters();

      // Parameters
      std::string m_keypoint_name;

      rclcpp::Subscription<human_swarm_interaction_interfaces::msg::PoseKeypointsStamped>::SharedPtr m_keypoints_sub;
      rclcpp::Publisher<human_swarm_interaction_interfaces::msg::PoseKeypointsStamped>::SharedPtr m_keypoints_pub;

  };  // class SingleKeypointTrackerNode
} // namespace HumanSwarmInteraction

#endif  // KEYPOINTS_TRACKING__SINGLE_KEYPOINT_TRACKER_NODE_HPP_